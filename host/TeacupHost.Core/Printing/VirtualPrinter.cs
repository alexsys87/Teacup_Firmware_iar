using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using TeacupHost.Core.GCode;

namespace TeacupHost.Core.Printing;

/// <summary>
/// A printer in software that answers like the Teacup firmware: line numbers
/// and checksums with Resend, "ok" per line, M105/M155 temperatures with
/// heating, M109/M190 waits with "busy", moves with a planner buffer, M114,
/// and SPI flash files (M20 … M30, M28/M29 upload, SD printing with M27).
/// Lets the whole program be tried without hardware.
/// </summary>
public sealed class VirtualPrinter : IPrinterTransport
{
    private static readonly CultureInfo Ci = CultureInfo.InvariantCulture;
    private const double Ambient = 22.0;
    private const double PlannerBuffer = 1.0;   // Seconds of moves the planner holds.

    private readonly BlockingCollection<string> _input = new();
    private readonly object _emitLock = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly Random _random = new(1234);
    private Thread? _worker;
    private volatile bool _running;

    // Flags set by the "interrupt" (WriteLine), handled by the worker.
    private volatile bool _killRequest;
    private volatile bool _cancelWait;
    private volatile bool _quickstop;

    // Protocol.
    private int _lastN;
    private bool _halted;

    // Motion.
    private double _x, _y, _z, _e;
    private bool _absXyz = true, _absE = true;
    private double _feed = 1500;
    private double _plannerEnd;
    private int _speedFactor = 100;

    // Heaters.
    private double _hotend = Ambient, _hotendTarget, _bed = Ambient, _bedTarget;
    private int _hotendPower, _bedPower;
    private double _lastHeaterUpdate;
    private int _autoReport;
    private double _lastAutoReport;
    private int _fan;

    // Files (SPI flash).
    private readonly List<(string Name, byte[] Data)> _files = new();
    private byte[]? _sdData;
    private string _sdName = "";
    private int _sdPos;
    private bool _sdPrinting;
    private string? _writeName;
    private StringBuilder? _writeData;

    public VirtualPrinter()
    {
        _files.Add(("CUBE100.GCO", Encoding.ASCII.GetBytes(DemoGCode.Generate())));
    }

    public string Name => "Virtual printer";
    public bool IsOpen => _running;

    /// <summary>Speed of time: 1 is real time, 10 makes moves and heating 10 times faster.</summary>
    public double TimeScale { get; set; } = 1.0;

    /// <summary>Probability of a simulated checksum error per numbered line (tests Resend).</summary>
    public double LineErrorRate { get; set; }

    public event Action<string>? LineReceived;
    public event Action<Exception>? Faulted;

    public IReadOnlyList<(string Name, byte[] Data)> Files => _files;

    /// <summary>Restart like after a reset or a power dip (for tests of the host).</summary>
    public void Reboot() => _input.Add("\u0001start");

    /// <summary>
    /// What the firmware's HOST_WATCH does when the host is gone: retract, lift,
    /// park, hotend off (for tests of the host).
    /// </summary>
    public void ParkAsHostLost() => _input.Add("\u0001hostlost");

    /// <summary>Hotend target, °C (for tests).</summary>
    public double HotendTarget => _hotendTarget;

    public void Open()
    {
        _running = true;
        _lastHeaterUpdate = Seconds;
        _worker = new Thread(WorkerLoop) { IsBackground = true, Name = "Virtual printer" };
        _worker.Start();
        _input.Add("\u0001start");      // Boot message from the worker thread.
    }

    public void Close()
    {
        _running = false;
        _worker?.Join(1000);
        _worker = null;
    }

    public void Dispose() => Close();

    /// <summary>Host → printer. Like the UART interrupt: emergency commands act at once.</summary>
    public void WriteLine(string line)
    {
        if (!_running)
            return;
        string cmd = StripNumber(line).Trim();
        if (cmd.StartsWith("M112", StringComparison.OrdinalIgnoreCase))
            _killRequest = true;
        else if (cmd.StartsWith("M108", StringComparison.OrdinalIgnoreCase))
            _cancelWait = true;
        else if (cmd.StartsWith("M410", StringComparison.OrdinalIgnoreCase))
            _quickstop = true;
        _input.Add(line);
    }

    private static string StripNumber(string line)
    {
        var s = line.TrimStart();
        if (s.StartsWith('N') || s.StartsWith('n'))
        {
            int i = 1;
            while (i < s.Length && (char.IsDigit(s[i]) || s[i] == '-'))
                i++;
            s = s[i..];
        }
        int star = s.IndexOf('*');
        return star >= 0 ? s[..star] : s;
    }

    private double Seconds => _clock.Elapsed.TotalSeconds;

    private void Emit(string line)
    {
        lock (_emitLock)
        {
            try
            {
                LineReceived?.Invoke(line);
            }
            catch (Exception ex)
            {
                Faulted?.Invoke(ex);
            }
        }
    }

    // ---------------------------------------------------------------- worker

    private void WorkerLoop()
    {
        while (_running)
        {
            Service();
            if (_input.TryTake(out var line, _sdPrinting ? 0 : 20))
            {
                if (line == "\u0001start")
                {
                    Boot();
                    continue;
                }
                if (line == "\u0001hostlost")
                {
                    DrainPlanner();
                    _z += 10;
                    _hotendTarget = 0;
                    Emit("echo:Host lost, parking");
                    Emit("echo:Parked, hotend temperature lowered");
                    continue;
                }
                HostLine(line);
                continue;
            }
            if (_sdPrinting && !_halted)
                SdStep();
        }
    }

    private void Boot()
    {
        _lastN = 0;
        _halted = false;
        _killRequest = false;
        _hotendTarget = _bedTarget = 0;
        _autoReport = 0;
        _sdPrinting = false;
        _writeName = null;
        Emit("start");
        Emit("echo:Teacup virtual printer");
        Emit("echo:SPI flash: virtual, 8192 kB");
    }

    /// <summary>Background duties: heaters, auto report, emergency flags.</summary>
    private void Service()
    {
        double now = Seconds;
        double dt = (now - _lastHeaterUpdate) * TimeScale;
        if (now - _lastHeaterUpdate >= 0.1)
        {
            _lastHeaterUpdate = now;
            Heat(ref _hotend, _hotendTarget, dt, 4.0, out _hotendPower);
            Heat(ref _bed, _bedTarget, dt, 0.8, out _bedPower);
        }
        if (_killRequest && !_halted)
            Kill("Emergency stop (M112)");
        if (_quickstop)
        {
            _quickstop = false;
            _plannerEnd = now;
        }
        if (_autoReport > 0 && now - _lastAutoReport >= _autoReport)
        {
            _lastAutoReport = now;
            Emit(TemperatureText());
        }
    }

    private void Heat(ref double t, double target, double dt, double maxRate, out int power)
    {
        double goal = target > 0 ? target : Ambient;
        double rate = Math.Clamp((goal - t) * 0.4, -maxRate * 0.5, maxRate);
        t += rate * dt + (_random.NextDouble() - 0.5) * 0.06;
        power = target > 0 ? (int)Math.Clamp(60 + (target - t) * 40, 0, 255) : 0;
    }

    private void Kill(string reason)
    {
        _halted = true;
        _hotendTarget = _bedTarget = 0;
        _sdPrinting = false;
        _plannerEnd = Seconds;
        Emit("Error:" + reason);
        Emit("Error:Printer halted. kill() called!");
    }

    private string TemperatureText() =>
        string.Format(Ci, "T:{0:0.0}/{1:0.0} B:{2:0.0}/{3:0.0} @:{4} B@:{5}",
            _hotend, _hotendTarget, _bed, _bedTarget, _hotendPower, _bedPower);

    /// <summary>Sleep in small steps, keeping the services running. False if cancelled.</summary>
    private bool Wait(double seconds, Func<bool>? cancel = null)
    {
        double end = Seconds + seconds;
        double lastBusy = Seconds;
        while (_running && Seconds < end)
        {
            Service();
            if (_halted || (cancel?.Invoke() ?? false))
                return false;
            if (Seconds - lastBusy >= 2)
            {
                lastBusy = Seconds;
                Emit("echo:busy: processing");
            }
            Thread.Sleep(10);
        }
        return true;
    }

    // ---------------------------------------------------------------- protocol

    private void HostLine(string raw)
    {
        string line = raw.Trim();
        int semi = line.IndexOf(';');
        if (semi >= 0)
            line = line[..semi].TrimEnd();
        if (line.Length == 0)
            return;

        if (_halted)
        {
            if (StripNumber(line).Trim().StartsWith("M999", StringComparison.OrdinalIgnoreCase))
            {
                Thread.Sleep(200);
                Boot();
            }
            return;
        }

        string command = line;
        int star = line.IndexOf('*');
        if (line[0] is 'N' or 'n')
        {
            if (star < 0)
            {
                LineError("No Checksum with line number");
                return;
            }
            int cs = ResponseParser.Checksum(line[..star]);
            if (!int.TryParse(line[(star + 1)..].Trim(), NumberStyles.Integer, Ci, out int given) || given != cs ||
                (LineErrorRate > 0 && _random.NextDouble() < LineErrorRate))
            {
                LineError("checksum mismatch");
                return;
            }
            string body = line[1..star];
            int i = 0;
            while (i < body.Length && (char.IsDigit(body[i]) || body[i] == '-'))
                i++;
            int n = int.Parse(body[..i], Ci);
            command = body[i..].Trim();
            var parsed = GCodeCommand.Parse(command);
            if (parsed.Is('M', 110))
            {
                _lastN = parsed.Has('N') ? (int)parsed.Get('N') : n;
            }
            else if (n != _lastN + 1)
            {
                LineError("Line Number is not Last Line Number+1");
                return;
            }
            else
            {
                _lastN = n;
            }
        }
        else if (star >= 0)
        {
            LineError("No Line Number with checksum");
            return;
        }

        bool okSent = false;
        if (command.Length > 0)
            okSent = Execute(command, fromSd: false);
        if (!okSent)
            Emit("ok");
    }

    private void LineError(string msg)
    {
        Emit($"Error:{msg}, Last Line: {_lastN}");
        // The firmware drops everything that follows in the receive buffer.
        var keep = new List<string>();
        while (_input.TryTake(out var dropped))
        {
            if (dropped.StartsWith('\u0001'))
                keep.Add(dropped);              // Simulator events, not host lines.
        }
        foreach (var k in keep)
            _input.Add(k);
        Emit($"Resend: {_lastN + 1}");
        Emit("ok");
    }

    // ---------------------------------------------------------------- commands

    /// <summary>Execute one command. Returns true if it already sent its "ok".</summary>
    private bool Execute(string text, bool fromSd)
    {
        var c = GCodeCommand.Parse(text);

        // Upload in progress: store everything up to M29.
        if (_writeName != null && !fromSd && !c.Is('M', 29))
        {
            _writeData!.Append(text).Append('\n');
            return false;
        }

        if (c.Letter == 'G')
        {
            switch (c.Code)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                    Move(c, arc: c.Code >= 2);
                    break;
                case 4:
                    DrainPlanner();
                    Wait((c.Get('P') / 1000.0 + c.Get('S')) / TimeScale);
                    break;
                case 28:
                    DrainPlanner();
                    bool any = c.Has('X') || c.Has('Y') || c.Has('Z');
                    if (!any || c.Has('X')) _x = 0;
                    if (!any || c.Has('Y')) _y = 0;
                    if (!any || c.Has('Z')) _z = 0;
                    Wait(1.5 / TimeScale);
                    break;
                case 90:
                    _absXyz = _absE = true;
                    break;
                case 91:
                    _absXyz = _absE = false;
                    break;
                case 92:
                    if (c.TryGet('X', out double x)) _x = x;
                    if (c.TryGet('Y', out double y)) _y = y;
                    if (c.TryGet('Z', out double z)) _z = z;
                    if (c.TryGet('E', out double e)) _e = e;
                    break;
                case 29:
                    DrainPlanner();
                    Emit("echo:Bed leveling: virtual probe, 3x3 points");
                    for (int i = 0; i < 9; i++)
                    {
                        Wait(0.3 / TimeScale);
                        Emit(string.Format(Ci, "Bed X:{0} Y:{1} Z:{2:0.000}", 20 + (i % 3) * 90, 20 + (i / 3) * 70,
                            (_random.NextDouble() - 0.5) * 0.2));
                    }
                    break;
            }
            return false;
        }

        if (c.Letter == 'T')
            return false;
        if (c.Letter != 'M')
        {
            Emit($"echo:Unknown command: \"{text}\"");
            return false;
        }

        switch (c.Code)
        {
            case 82: _absE = true; break;
            case 83: _absE = false; break;
            case 104:
                if (c.TryGet('S', out double s104)) _hotendTarget = s104;
                break;
            case 140:
                if (c.TryGet('S', out double s140)) _bedTarget = s140;
                break;
            case 109:
            case 190:
                return HeatAndWait(c, c.Code == 109, fromSd);
            case 105:
                if (!fromSd)
                {
                    Emit("ok " + TemperatureText());
                    return true;
                }
                Emit(TemperatureText());
                break;
            case 155:
                _autoReport = (int)c.Get('S');
                _lastAutoReport = Seconds;
                break;
            case 106:
                _fan = (int)c.Get('S', 255);
                break;
            case 107:
                _fan = 0;
                break;
            case 114:
                DrainPlanner();
                Emit(string.Format(Ci, "X:{0:0.000} Y:{1:0.000} Z:{2:0.000} E:{3:0.000} Count X:{4} Y:{5} Z:{6}",
                    _x, _y, _z, _e, (int)(_x * 160), (int)(_y * 160), (int)(_z * 8000)));
                break;
            case 115:
                Emit("FIRMWARE_NAME:Teacup FIRMWARE_URL:http://github.com/traumflug/Teacup_Firmware/ " +
                     "PROTOCOL_VERSION:1.0 MACHINE_TYPE:Virtual EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:2 HEATER_COUNT:2");
                Emit("Cap:AUTOREPORT_TEMP:1");
                Emit("Cap:EMERGENCY_PARSER:1");
                Emit("Cap:BUSY_PROTOCOL:1");
                break;
            case 119:
                Emit($"x_min:{(_x <= 0 ? "triggered" : "open")} y_min:{(_y <= 0 ? "triggered" : "open")} " +
                     $"z_min:{(_z <= 0 ? "triggered" : "open")} filament:open");
                break;
            case 220:
                if (c.TryGet('S', out double s220)) _speedFactor = (int)s220;
                else Emit($"FR:{_speedFactor}%");
                break;
            case 400:
                DrainPlanner();
                break;
            case 112:
                Kill("Emergency stop (M112)");
                return true;
            case 410:
                _plannerEnd = Seconds;
                break;
            case 503:
                Emit("echo:Steps per unit: M92 X160.00 Y160.00 Z8000.00 E1672.00");
                Emit("echo:Max feedrates (mm/s): M203 X150.00 Y150.00 Z4.00 E25.00");
                Emit("echo:Acceleration (mm/s2): M204 P1000.00 R5000.00 T1000.00");
                Emit("echo:Hotend PID: M301 P22.20 I1.08 D114.00");
                break;
            case 303:
                Emit("PID Autotune start");
                for (int i = 1; i <= 5; i++)
                {
                    if (!Wait(1.0 / TimeScale))
                        break;
                    Emit($"bias: 120 d: 120 min: 195.{i} max: 205.{i}");
                }
                Emit("PID Autotune finished! Put the last Kp, Ki and Kd constants from below into Configuration.h");
                Emit("#define DEFAULT_Kp 22.20");
                Emit("#define DEFAULT_Ki 1.08");
                Emit("#define DEFAULT_Kd 114.00");
                break;
            case 20:
                Emit("Begin file list");
                foreach (var f in _files)
                    Emit($"{f.Name} {f.Data.Length}");
                Emit("End file list");
                break;
            case 21:
                Emit("echo:SD card ok");
                break;
            case 22:
                _sdPrinting = false;
                _sdData = null;
                break;
            case 23:
            {
                string name = c.Argument.Trim().ToUpperInvariant();
                var f = _files.FirstOrDefault(x => x.Name == name);
                if (f.Data == null)
                {
                    Emit($"echo:open failed, File: {name}.");
                    break;
                }
                _sdData = f.Data;
                _sdName = f.Name;
                _sdPos = 0;
                _sdPrinting = false;
                Emit($"File opened: {f.Name} Size: {f.Data.Length}");
                Emit("File selected");
                break;
            }
            case 24:
                if (_sdData != null)
                    _sdPrinting = true;
                break;
            case 25:
                _sdPrinting = false;
                break;
            case 26:
                if (_sdData != null && c.TryGet('S', out double pos))
                    _sdPos = (int)Math.Clamp(pos, 0, _sdData.Length);
                break;
            case 27:
                Emit(_sdPrinting && _sdData != null
                    ? $"SD printing byte {_sdPos}/{_sdData.Length}"
                    : "Not SD printing");
                break;
            case 28:
            {
                string name = c.Argument.Trim().ToUpperInvariant();
                if (name.Length == 0)
                {
                    Emit("echo:M28 needs a file name");
                    break;
                }
                _files.RemoveAll(x => x.Name == name);
                _writeName = name;
                _writeData = new StringBuilder();
                Emit($"Writing to file: {name}");
                break;
            }
            case 29:
                if (_writeName != null)
                {
                    _files.Add((_writeName, Encoding.ASCII.GetBytes(_writeData!.ToString())));
                    _writeName = null;
                    _writeData = null;
                    Emit("Done saving file.");
                }
                break;
            case 30:
            {
                string name = c.Argument.Trim().ToUpperInvariant();
                if (_files.RemoveAll(x => x.Name == name) > 0)
                    Emit($"File deleted:{name}");
                else
                    Emit($"Deletion failed, File: {name}.");
                break;
            }
            case 9002:
                _files.Clear();
                Emit("echo:Flash files deleted");
                break;
            case 110:
            case 999:
                break;
            case 17: case 18: case 84: case 80: case 81: case 117: case 73: case 221: case 300:
            case 500: case 501: case 502: case 201: case 203: case 204: case 205: case 92:
            case 900: case 851: case 420: case 113: case 86: case 302: case 301: case 304:
                break;
            default:
                Emit($"echo:Unknown command: \"{text}\"");
                break;
        }
        return false;
    }

    private bool HeatAndWait(GCodeCommand c, bool hotend, bool fromSd)
    {
        bool cool = c.Has('R');
        double target = c.Has('R') ? c.Get('R') : c.Get('S');
        if (hotend) _hotendTarget = target; else _bedTarget = target;
        DrainPlanner();
        _cancelWait = false;
        double lastReport = Seconds;
        double inside = -1;
        double lastBusy = Seconds;
        while (_running && !_halted)
        {
            Service();
            if (_cancelWait)
            {
                _cancelWait = false;
                Emit("echo:Wait cancelled (M108)");
                break;
            }
            double t = hotend ? _hotend : _bed;
            bool ok = cool ? Math.Abs(t - target) < 1.5 : t >= target - 1.5;
            if (target <= 0)
                break;
            if (ok)
            {
                if (inside < 0)
                    inside = Seconds;
                if ((Seconds - inside) * TimeScale >= 3)
                    break;
            }
            else
            {
                inside = -1;
            }
            if (Seconds - lastReport >= 1)
            {
                lastReport = Seconds;
                Emit(TemperatureText());
            }
            if (Seconds - lastBusy >= 2)
            {
                lastBusy = Seconds;
                Emit("echo:busy: processing");
            }
            Thread.Sleep(20);
        }
        return false;
    }

    private void Move(GCodeCommand c, bool arc)
    {
        if (c.TryGet('F', out double f) && f > 0)
            _feed = f;
        double x = _x, y = _y, z = _z;
        if (c.TryGet('X', out double cx)) x = _absXyz ? cx : _x + cx;
        if (c.TryGet('Y', out double cy)) y = _absXyz ? cy : _y + cy;
        if (c.TryGet('Z', out double cz)) z = _absXyz ? cz : _z + cz;
        double de = 0;
        if (c.TryGet('E', out double ce))
        {
            double eNew = _absE ? ce : _e + ce;
            de = eNew - _e;
            _e = eNew;
        }
        double dist = Math.Sqrt((x - _x) * (x - _x) + (y - _y) * (y - _y) + (z - _z) * (z - _z));
        if (arc)
            dist *= 1.3;
        if (dist < 1e-9)
            dist = Math.Abs(de);
        double speed = _feed / 60.0 * _speedFactor / 100.0;
        if (z != _z)
            speed = Math.Min(speed, 4.0 * dist / Math.Max(Math.Abs(z - _z), 1e-9));
        double duration = dist / Math.Max(speed, 0.1) / TimeScale;

        // The planner holds about a second of moves; wait for room.
        double now = Seconds;
        if (_plannerEnd < now)
            _plannerEnd = now;
        while (_running && !_halted && _plannerEnd - Seconds > PlannerBuffer)
        {
            Service();
            if (_quickstop)
                break;
            Thread.Sleep(5);
        }
        if (_plannerEnd < Seconds)
            _plannerEnd = Seconds;
        _plannerEnd += duration;
        _x = x;
        _y = y;
        _z = z;
    }

    private void DrainPlanner()
    {
        double lastBusy = Seconds;
        while (_running && !_halted && Seconds < _plannerEnd)
        {
            Service();
            if (Seconds - lastBusy >= 2)
            {
                lastBusy = Seconds;
                Emit("echo:busy: processing");
            }
            Thread.Sleep(5);
        }
    }

    private void SdStep()
    {
        var data = _sdData;
        if (data == null || _sdPos >= data.Length)
        {
            _sdPrinting = false;
            return;
        }
        int start = _sdPos;
        int end = Array.IndexOf(data, (byte)'\n', start);
        if (end < 0)
            end = data.Length;
        _sdPos = Math.Min(data.Length, end + 1);
        string line = Encoding.ASCII.GetString(data, start, end - start);
        string cmd = GCodeCommand.StripComment(line);
        if (cmd.Length > 0)
            Execute(cmd, fromSd: true);
        if (_sdPos >= data.Length)
        {
            DrainPlanner();
            _sdPrinting = false;
        }
    }
}
