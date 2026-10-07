using System.Numerics;

namespace TeacupHost.Core.GCode;

/// <summary>Machine limits used for the print time estimate.</summary>
public sealed class ToolpathOptions
{
    /// <summary>Printing acceleration, mm/s² (printer.p3steel.h: ACCELERATION).</summary>
    public double Acceleration { get; set; } = 1000;
    /// <summary>Travel acceleration, mm/s² (ACCELERATION_TRAVEL).</summary>
    public double TravelAcceleration { get; set; } = 1000;
    /// <summary>Retract acceleration, mm/s² (ACCELERATION_RETRACT).</summary>
    public double RetractAcceleration { get; set; } = 5000;
    /// <summary>Junction deviation, mm (M205 J).</summary>
    public double JunctionDeviation { get; set; } = 0.013;
    /// <summary>Maximum speeds per axis, mm/s (MAXIMUM_FEEDRATE_* / 60).</summary>
    public double MaxSpeedX { get; set; } = 150;
    public double MaxSpeedY { get; set; } = 150;
    public double MaxSpeedZ { get; set; } = 4;
    public double MaxSpeedE { get; set; } = 25;
    /// <summary>Feedrate before the first F word, mm/min.</summary>
    public double DefaultFeedrate { get; set; } = 1500;
    /// <summary>Arc chord tolerance and segment length limits, mm (like the firmware).</summary>
    public double ArcTolerance { get; set; } = 0.01;
    public double ArcSegmentMin { get; set; } = 0.1;
    public double ArcSegmentMax { get; set; } = 1.0;
}

/// <summary>
/// Interprets G-code into a <see cref="Toolpath"/>: tracks absolute/relative
/// modes, expands arcs, splits layers and estimates the time of every move
/// with a trapezoidal speed profile and junction deviation corners.
/// </summary>
public sealed class ToolpathBuilder
{
    private readonly ToolpathOptions _opt;
    private readonly List<ToolpathSegment> _segments = new();
    private readonly List<ToolpathLayer> _layers = new();

    // Interpreter state.
    private Vector3 _pos;
    private double _e;
    private bool _absXyz = true;
    private bool _absE = true;
    private double _feed;               // mm/min
    private double _unit = 1.0;         // 25.4 after G20
    private double _accel;
    private double _travelAccel;
    private double _retractAccel;
    private double _jd;
    private FeatureType _feature = FeatureType.Extrude;
    private bool _typeComments;

    // Layer tracking.
    private float _layerZ = float.NaN;
    private int _firstAfterExtrusion;   // First segment after the last extrusion.
    private float _heightHint;          // From ";HEIGHT:" comments.

    // Time estimate.
    private double _time;               // End of the last finished segment.
    private double _pendingDelay;       // Dwell and retracts before the next move.
    private int _open = -1;             // Segment whose duration isn't known yet.
    private double _openEntry;
    private Vector3 _openDir;
    private double _openSpeed;
    private double _openAccel;

    private double _filament;
    private Vector3 _min = new(float.MaxValue);
    private Vector3 _max = new(float.MinValue);
    private int _line;

    public ToolpathBuilder(ToolpathOptions? options = null)
    {
        _opt = options ?? new ToolpathOptions();
        _feed = _opt.DefaultFeedrate;
        _accel = _opt.Acceleration;
        _travelAccel = _opt.TravelAcceleration;
        _retractAccel = _opt.RetractAcceleration;
        _jd = _opt.JunctionDeviation;
    }

    public static Toolpath Build(IReadOnlyList<string> lines, ToolpathOptions? options = null,
        CancellationToken cancel = default, IProgress<double>? progress = null)
    {
        var b = new ToolpathBuilder(options);
        int n = lines.Count;
        int step = Math.Max(1, n / 100);
        for (int i = 0; i < n; i++)
        {
            if (i % step == 0)
            {
                cancel.ThrowIfCancellationRequested();
                progress?.Report((double)i / n);
            }
            b.ProcessLine(i, lines[i]);
        }
        return b.Finish(n);
    }

    public void ProcessLine(int lineIndex, string text)
    {
        _line = lineIndex;
        if (text.Length == 0)
            return;
        var cmd = GCodeCommand.Parse(text);
        if (cmd.Comment != null && cmd.IsEmpty)
            ProcessComment(cmd.Comment);
        if (cmd.IsEmpty)
            return;

        if (cmd.Letter == 'G')
        {
            switch (cmd.Code)
            {
                case 0:
                case 1:
                    Linear(cmd);
                    break;
                case 2:
                case 3:
                    Arc(cmd, cmd.Code == 2);
                    break;
                case 4:
                    // Dwell: P in ms, S in s. The machine stops before it.
                    CloseOpen(0);
                    _pendingDelay += cmd.Get('P') / 1000.0 + cmd.Get('S');
                    break;
                case 20:
                    _unit = 25.4;
                    break;
                case 21:
                    _unit = 1.0;
                    break;
                case 28:
                    Home(cmd);
                    break;
                case 90:
                    _absXyz = true;
                    _absE = true;
                    break;
                case 91:
                    _absXyz = false;
                    _absE = false;
                    break;
                case 92:
                    if (cmd.TryGet('X', out double x)) _pos.X = (float)(x * _unit);
                    if (cmd.TryGet('Y', out double y)) _pos.Y = (float)(y * _unit);
                    if (cmd.TryGet('Z', out double z)) _pos.Z = (float)(z * _unit);
                    if (cmd.TryGet('E', out double e)) _e = e * _unit;
                    if (!cmd.Has('X') && !cmd.Has('Y') && !cmd.Has('Z') && !cmd.Has('E'))
                    {
                        _pos = Vector3.Zero;
                        _e = 0;
                    }
                    break;
            }
        }
        else if (cmd.Letter == 'M')
        {
            switch (cmd.Code)
            {
                case 82:
                    _absE = true;
                    break;
                case 83:
                    _absE = false;
                    break;
                case 204:
                    if (cmd.TryGet('S', out double s)) { _accel = s; _travelAccel = s; }
                    if (cmd.TryGet('P', out double p)) _accel = p;
                    if (cmd.TryGet('T', out double t)) _travelAccel = t;
                    if (cmd.TryGet('R', out double r)) _retractAccel = r;
                    break;
                case 205:
                    if (cmd.TryGet('J', out double j) && j > 0) _jd = j;
                    break;
            }
        }
    }

    private void ProcessComment(string c)
    {
        if (c.StartsWith("TYPE:", StringComparison.OrdinalIgnoreCase))
        {
            _feature = ParseFeature(c[5..]);
            _typeComments = true;
        }
        else if (c.StartsWith("HEIGHT:", StringComparison.OrdinalIgnoreCase) &&
                 float.TryParse(c.AsSpan(7), System.Globalization.NumberStyles.Float,
                     System.Globalization.CultureInfo.InvariantCulture, out float h))
        {
            _heightHint = h;
        }
    }

    /// <summary>Map slicer feature names (PrusaSlicer, Cura, SuperSlicer, Simplify3D).</summary>
    public static FeatureType ParseFeature(string name)
    {
        string s = name.Trim().ToLowerInvariant();
        if (s.Contains("support")) return FeatureType.Support;
        if (s.Contains("skirt") || s.Contains("brim")) return FeatureType.Skirt;
        if (s.Contains("external") || s.Contains("outer") || s.Contains("overhang")) return FeatureType.OuterWall;
        if (s.Contains("perimeter") || s.Contains("wall")) return FeatureType.InnerWall;
        if (s.Contains("bridge")) return FeatureType.Bridge;
        if (s.Contains("gap")) return FeatureType.GapFill;
        if (s.Contains("top") || s.Contains("ironing")) return FeatureType.TopSolid;
        if (s.Contains("solid") || s.Contains("skin") || s.Contains("bottom")) return FeatureType.SolidInfill;
        if (s.Contains("fill")) return FeatureType.Infill;
        if (s.Contains("custom") || s.Contains("wipe")) return FeatureType.Custom;
        return FeatureType.Extrude;
    }

    private void Home(GCodeCommand cmd)
    {
        bool any = cmd.Has('X') || cmd.Has('Y') || cmd.Has('Z');
        var target = _pos;
        if (!any || cmd.Has('X')) target.X = 0;
        if (!any || cmd.Has('Y')) target.Y = 0;
        if (!any || cmd.Has('Z')) target.Z = 0;
        if (target != _pos)
            AddMove(target, 0, _opt.MaxSpeedX * 0.5);
        _pos = target;
    }

    private void Linear(GCodeCommand cmd)
    {
        if (cmd.TryGet('F', out double f) && f > 0)
            _feed = f * _unit;

        var target = _pos;
        if (cmd.TryGet('X', out double x)) target.X = (float)(_absXyz ? x * _unit : _pos.X + x * _unit);
        if (cmd.TryGet('Y', out double y)) target.Y = (float)(_absXyz ? y * _unit : _pos.Y + y * _unit);
        if (cmd.TryGet('Z', out double z)) target.Z = (float)(_absXyz ? z * _unit : _pos.Z + z * _unit);
        double de = 0;
        if (cmd.TryGet('E', out double e))
        {
            double eNew = _absE ? e * _unit : _e + e * _unit;
            de = eNew - _e;
            _e = eNew;
        }

        if (target == _pos)
        {
            if (de != 0)
            {
                // Retract or unretract: takes time, no geometry.
                double speed = Math.Min(_feed / 60.0, _opt.MaxSpeedE);
                CloseOpen(0);
                _pendingDelay += TrapezoidTime(Math.Abs(de), 0, speed, 0, _retractAccel);
            }
            return;
        }
        AddMove(target, de, _feed / 60.0);
        _pos = target;
    }

    private void Arc(GCodeCommand cmd, bool clockwise)
    {
        if (cmd.TryGet('F', out double f) && f > 0)
            _feed = f * _unit;

        var start = _pos;
        var end = _pos;
        if (cmd.TryGet('X', out double x)) end.X = (float)(_absXyz ? x * _unit : _pos.X + x * _unit);
        if (cmd.TryGet('Y', out double y)) end.Y = (float)(_absXyz ? y * _unit : _pos.Y + y * _unit);
        if (cmd.TryGet('Z', out double z)) end.Z = (float)(_absXyz ? z * _unit : _pos.Z + z * _unit);
        double deTotal = 0;
        if (cmd.TryGet('E', out double e))
        {
            double eNew = _absE ? e * _unit : _e + e * _unit;
            deTotal = eNew - _e;
            _e = eNew;
        }

        double cx, cy;
        if (cmd.Has('I') || cmd.Has('J'))
        {
            cx = start.X + cmd.Get('I') * _unit;
            cy = start.Y + cmd.Get('J') * _unit;
        }
        else if (cmd.TryGet('R', out double r) && r != 0)
        {
            r *= _unit;
            double dx = end.X - start.X, dy = end.Y - start.Y;
            double d = Math.Sqrt(dx * dx + dy * dy);
            if (d < 1e-9 || d > 2 * Math.Abs(r) + 1e-6)
            {
                AddMove(end, deTotal, _feed / 60.0);
                _pos = end;
                return;
            }
            double h = Math.Sqrt(Math.Max(0, r * r - d * d / 4));
            // Center left of the chord for CCW with positive R, right for CW.
            double sign = (clockwise ^ (r < 0)) ? -1 : 1;
            cx = start.X + dx / 2 - sign * h * dy / d;
            cy = start.Y + dy / 2 + sign * h * dx / d;
        }
        else
        {
            // Bad parameters: the firmware doesn't move.
            return;
        }

        double radius = Math.Sqrt((start.X - cx) * (start.X - cx) + (start.Y - cy) * (start.Y - cy));
        double a0 = Math.Atan2(start.Y - cy, start.X - cx);
        double a1 = Math.Atan2(end.Y - cy, end.X - cx);
        double sweep = a1 - a0;
        if (clockwise)
        {
            if (sweep >= -1e-9) sweep -= 2 * Math.PI;
        }
        else
        {
            if (sweep <= 1e-9) sweep += 2 * Math.PI;
        }
        int turns = (int)cmd.Get('P');
        if (turns > 0)
            sweep += (clockwise ? -2 : 2) * Math.PI * turns;

        double arcLen = Math.Abs(sweep) * radius;
        double seg = radius > _opt.ArcTolerance
            ? 2 * Math.Sqrt(Math.Max(0, 2 * radius * _opt.ArcTolerance - _opt.ArcTolerance * _opt.ArcTolerance))
            : _opt.ArcSegmentMin;
        seg = Math.Clamp(seg, _opt.ArcSegmentMin, _opt.ArcSegmentMax);
        int n = Math.Max(1, (int)Math.Ceiling(arcLen / seg));
        n = Math.Min(n, 10000);

        var prev = start;
        for (int i = 1; i <= n; i++)
        {
            Vector3 p;
            if (i == n)
            {
                p = end;
            }
            else
            {
                double t = (double)i / n;
                double a = a0 + sweep * t;
                p = new Vector3((float)(cx + radius * Math.Cos(a)), (float)(cy + radius * Math.Sin(a)),
                    (float)(start.Z + (end.Z - start.Z) * t));
            }
            AddMove(p, deTotal / n, _feed / 60.0, prev);
            prev = p;
        }
        _pos = end;
    }

    private void AddMove(Vector3 target, double de, double feed, Vector3? from = null)
    {
        var start = from ?? _pos;
        var delta = target - start;
        double len = delta.Length();
        if (len < 1e-6)
            return;

        bool extrude = de > 1e-6;
        // Axis speed limits, like the firmware does it.
        double speed = Math.Max(feed, 0.1);
        speed = LimitAxis(speed, len, delta.X, _opt.MaxSpeedX);
        speed = LimitAxis(speed, len, delta.Y, _opt.MaxSpeedY);
        speed = LimitAxis(speed, len, delta.Z, _opt.MaxSpeedZ);
        if (de != 0)
            speed = LimitAxis(speed, len, (float)de, _opt.MaxSpeedE);

        // Layers: a new layer starts with the moves leading to the first
        // extrusion at a new height.
        if (extrude)
        {
            if (float.IsNaN(_layerZ) || Math.Abs(target.Z - _layerZ) > 1e-4f)
                StartLayer(target.Z);
            _filament += de;
            _min = Vector3.Min(_min, Vector3.Min(start, target));
            _max = Vector3.Max(_max, Vector3.Max(start, target));
        }

        var dir = delta / (float)len;
        double accel = extrude ? _accel : _travelAccel;
        double entry = 0;
        if (_open >= 0)
        {
            double junction = JunctionSpeed(_openDir, dir, Math.Min(_openSpeed, speed), Math.Min(_openAccel, accel));
            CloseOpen(junction);
            entry = junction;
        }

        var segment = new ToolpathSegment
        {
            Start = start,
            End = target,
            Extrusion = (float)Math.Max(de, 0),
            Speed = (float)speed,
            Line = _line,
            Layer = Math.Max(0, _layers.Count - 1),
            Kind = extrude ? MoveKind.Extrude : MoveKind.Travel,
            Feature = extrude ? (_typeComments ? _feature : FeatureType.Extrude) : FeatureType.Travel,
        };
        _segments.Add(segment);
        _open = _segments.Count - 1;
        _openEntry = entry;
        _openDir = dir;
        _openSpeed = speed;
        _openAccel = accel;
        if (extrude)
            _firstAfterExtrusion = _segments.Count;
    }

    private static double LimitAxis(double speed, double len, float axisDelta, double max)
    {
        double a = Math.Abs(axisDelta);
        if (a < 1e-9 || max <= 0)
            return speed;
        return Math.Min(speed, max * len / a);
    }

    private double JunctionSpeed(Vector3 prev, Vector3 next, double vmax, double accel)
    {
        double cosTheta = -Vector3.Dot(prev, next);
        if (cosTheta < -0.9999)
            return vmax;                        // Straight on.
        if (cosTheta > 0.9999)
            return 0;                           // Reversal.
        double sinHalf = Math.Sqrt(0.5 * (1 - cosTheta));
        double v = Math.Sqrt(accel * _jd * sinHalf / (1 - sinHalf));
        return Math.Min(v, vmax);
    }

    /// <summary>Give the open segment its duration now that its exit speed is known.</summary>
    private void CloseOpen(double exit)
    {
        if (_open < 0)
            return;
        var s = _segments[_open];
        double len = s.Length;
        double exitSpeed = Math.Min(exit, Math.Sqrt(_openEntry * _openEntry + 2 * _openAccel * len));
        s.StartTime = (float)(_time + _pendingDelay);
        s.Duration = (float)TrapezoidTime(len, _openEntry, _openSpeed, exitSpeed, _openAccel);
        _segments[_open] = s;
        _time = s.StartTime + s.Duration;
        _pendingDelay = 0;
        _open = -1;
    }

    public static double TrapezoidTime(double len, double v0, double vmax, double v1, double accel)
    {
        if (len <= 0)
            return 0;
        if (vmax <= 0)
            return 0;
        if (accel <= 0)
            return len / vmax;
        v0 = Math.Min(v0, vmax);
        v1 = Math.Min(v1, vmax);
        double dAcc = (vmax * vmax - v0 * v0) / (2 * accel);
        double dDec = (vmax * vmax - v1 * v1) / (2 * accel);
        if (dAcc + dDec <= len)
            return (vmax - v0) / accel + (vmax - v1) / accel + (len - dAcc - dDec) / vmax;
        double vp = Math.Sqrt((2 * accel * len + v0 * v0 + v1 * v1) / 2);
        if (vp < Math.Max(v0, v1))
            return 2 * len / (v0 + v1);
        return (vp - v0) / accel + (vp - v1) / accel;
    }

    private void StartLayer(float z)
    {
        int first = Math.Min(_firstAfterExtrusion, _segments.Count);
        if (_layers.Count == 0)
            first = 0;
        // Close the previous layer.
        if (_layers.Count > 0)
        {
            var last = _layers[^1];
            _layers[^1] = last with { SegmentCount = first - last.FirstSegment };
        }
        float height = _heightHint > 0 ? _heightHint
            : float.IsNaN(_layerZ) ? z : z - _layerZ;
        height = Math.Clamp(height <= 0 ? 0.2f : height, 0.05f, 1.0f);
        // Moves since the last extrusion belong to the new layer, Finish()
        // assigns the layer indices from the ranges.
        _layers.Add(new ToolpathLayer(_layers.Count, z, height, first, 0));
        _layerZ = z;
    }

    public Toolpath Finish(int lineCount)
    {
        CloseOpen(0);
        // Close the last layer, or make one layer of a file without extrusion.
        if (_layers.Count == 0 && _segments.Count > 0)
            _layers.Add(new ToolpathLayer(0, _segments[0].End.Z, 0.2f, 0, _segments.Count));
        else if (_layers.Count > 0)
            _layers[^1] = _layers[^1] with { SegmentCount = _segments.Count - _layers[^1].FirstSegment };

        var arr = _segments.ToArray();
        foreach (var layer in _layers)
            for (int i = layer.FirstSegment; i < layer.EndSegment; i++)
                arr[i].Layer = layer.Index;

        if (_min.X > _max.X)
        {
            _min = Vector3.Zero;
            _max = Vector3.Zero;
        }
        return new Toolpath(arr, _layers.ToArray(), (float)(_time + _pendingDelay), (float)_filament,
            _min, _max, lineCount);
    }
}
