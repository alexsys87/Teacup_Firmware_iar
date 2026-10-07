using System.Windows.Media.Media3D;
using TeacupHost.Core.GCode;
using TeacupHost.Infrastructure;

namespace TeacupHost.ViewModels;

/// <summary>What the viewer shows: preview, simulation and the live print; the current G-code line.</summary>
public sealed partial class MainViewModel
{
    private ViewerMode _viewerMode = ViewerMode.Preview;
    public ViewerMode ViewerMode
    {
        get => _viewerMode;
        private set
        {
            if (!Set(ref _viewerMode, value))
                return;
            OnPropertyChanged(nameof(IsSimulation));
            OnPropertyChanged(nameof(IsLive));
            OnPropertyChanged(nameof(CanSimulate));
            OnPropertyChanged(nameof(ModeText));
            RelayCommand.Refresh();
        }
    }

    public bool IsSimulation => ViewerMode == ViewerMode.Simulation;
    public bool IsLive => ViewerMode == ViewerMode.Live;
    public bool CanSimulate => ViewerMode != ViewerMode.Live && HasToolpath;

    public string ModeText => ViewerMode switch
    {
        ViewerMode.Simulation => "СИМУЛЯЦИЯ",
        ViewerMode.Live => "ПЕЧАТЬ",
        _ => "ПРОСМОТР",
    };

    public int LayerCount => Toolpath?.Layers.Count ?? 0;
    public int LayerSliderMax => Math.Max(0, LayerCount - 1);

    private bool _internalUpdate;

    private int _maxLayer;
    /// <summary>Highest visible layer (zero based). Moving it in a simulation seeks to the end of that layer.</summary>
    public int MaxLayer
    {
        get => _maxLayer;
        set
        {
            if (!Set(ref _maxLayer, value))
                return;
            OnPropertyChanged(nameof(LayerText));
            if (_internalUpdate || Toolpath == null)
                return;
            if (ViewerMode == ViewerMode.Simulation)
            {
                var l = Toolpath.Layers[Math.Clamp(value, 0, Toolpath.Layers.Count - 1)];
                int last = Math.Max(l.FirstSegment, l.EndSegment - 1);
                SeekSimulation(Toolpath.Segments[last].EndTime - 1e-4);
            }
        }
    }

    public string LayerText
    {
        get
        {
            var tp = Toolpath;
            if (tp == null || tp.Layers.Count == 0)
                return "—";
            int l = Math.Clamp(MaxLayer, 0, tp.Layers.Count - 1);
            return $"{l + 1} / {tp.Layers.Count} · Z {tp.Layers[l].Z:0.##}";
        }
    }

    private int _segmentLimit = int.MaxValue;
    public int SegmentLimit { get => _segmentLimit; private set => Set(ref _segmentLimit, value); }

    private Point3D _nozzlePosition;
    public Point3D NozzlePosition { get => _nozzlePosition; private set => Set(ref _nozzlePosition, value); }

    private bool _showNozzle;
    public bool ShowNozzle { get => _showNozzle; private set => Set(ref _showNozzle, value); }

    private bool _showTravel;
    public bool ShowTravel { get => _showTravel; set => Set(ref _showTravel, value); }

    public double BedWidth => _settings.BedWidth;
    public double BedDepth => _settings.BedDepth;
    public double FilamentDiameter => _settings.FilamentDiameter;

    // ---------------------------------------------------------------- current line

    private int _currentLine = -1;
    /// <summary>Zero-based line of the document being executed (simulated or printed).</summary>
    public int CurrentLine
    {
        get => _currentLine;
        private set
        {
            if (!Set(ref _currentLine, value))
                return;
            var doc = Document;
            CurrentLineText = doc != null && value >= 0 && value < doc.Lines.Count ? doc.Lines[value] : "";
            OnPropertyChanged(nameof(CurrentLineNumberText));
        }
    }

    public string CurrentLineNumberText => CurrentLine >= 0 && Document != null
        ? $"Строка {CurrentLine + 1:N0} из {Document.Lines.Count:N0}"
        : "";

    private string _currentLineText = "";
    public string CurrentLineText { get => _currentLineText; private set => Set(ref _currentLineText, value); }

    private string _nozzleText = "";
    public string NozzleText { get => _nozzleText; private set => Set(ref _nozzleText, value); }

    /// <summary>Show everything up to the given segment, the nozzle at its end.</summary>
    private void ShowProgress(int segment, Point3D? nozzle = null)
    {
        var tp = Toolpath;
        if (tp == null || tp.Segments.Length == 0)
            return;
        _internalUpdate = true;
        try
        {
            if (segment < 0)
            {
                SegmentLimit = 0;
                MaxLayer = 0;
                var first = tp.Segments[0].Start;
                NozzlePosition = new Point3D(first.X, first.Y, first.Z);
            }
            else
            {
                segment = Math.Min(segment, tp.Segments.Length - 1);
                ref readonly var s = ref tp.Segments[segment];
                SegmentLimit = segment + 1;
                MaxLayer = s.Layer;
                NozzlePosition = nozzle ?? new Point3D(s.End.X, s.End.Y, s.End.Z);
                NozzleText = $"X {NozzlePosition.X:0.00}  Y {NozzlePosition.Y:0.00}  Z {NozzlePosition.Z:0.00}  " +
                             $"F {s.Speed * 60:0} мм/мин";
            }
            ShowNozzle = true;
        }
        finally
        {
            _internalUpdate = false;
        }
    }

    /// <summary>Back to the plain preview: all layers, no nozzle.</summary>
    private void ShowAll()
    {
        _internalUpdate = true;
        SegmentLimit = int.MaxValue;
        MaxLayer = LayerSliderMax;
        _internalUpdate = false;
        ShowNozzle = false;
        NozzleText = "";
        OnPropertyChanged(nameof(LayerText));
    }

    /// <summary>Live print: the printer acknowledged (host) or reached (SD) this document line.</summary>
    private void ShowLiveLine(int docLine)
    {
        CurrentLine = docLine;
        var tp = Toolpath;
        if (tp == null)
            return;
        int seg = tp.LastSegmentAtOrBeforeLine(docLine);
        ShowProgress(seg);
        if (seg >= 0)
        {
            _liveEstimateDone = tp.Segments[seg].EndTime;
            // The time slider shows where the print is, by the estimate.
            _internalUpdate = true;
            SimTime = _liveEstimateDone;
            _internalUpdate = false;
            OnPropertyChanged(nameof(SimTimeText));
        }
    }

    // ---------------------------------------------------------------- simulation

    public double[] SimSpeeds { get; } = { 1, 2, 5, 10, 25, 50, 100, 250, 1000 };

    private double _simSpeed = 10;
    public double SimSpeed { get => _simSpeed; set => Set(ref _simSpeed, value); }

    private bool _simPlaying;
    public bool SimPlaying
    {
        get => _simPlaying;
        private set
        {
            if (Set(ref _simPlaying, value))
                RelayCommand.Refresh();
        }
    }

    public double SimTotal => Toolpath?.TotalTime ?? 0;

    private double _simTime;
    /// <summary>Simulated time, s. Setting it (slider) seeks.</summary>
    public double SimTime
    {
        get => _simTime;
        set
        {
            if (_internalUpdate)
            {
                Set(ref _simTime, value);
                return;
            }
            if (ViewerMode == ViewerMode.Live || Toolpath == null)
                return;
            // The slider writes its value back when its range changes (new
            // file): that's not a seek and must not start a simulation.
            if (Math.Abs(value - _simTime) < 1e-6)
                return;
            SeekSimulation(value);
        }
    }

    public string SimTimeText => $"{FormatTime(_simTime)} / {FormatTime(SimTotal)}";

    private double _lastTick = -1;

    public RelayCommand SimPlayPauseCommand { get; private set; } = null!;
    public RelayCommand SimStopCommand { get; private set; } = null!;
    public RelayCommand SimStepForwardCommand { get; private set; } = null!;
    public RelayCommand SimStepBackCommand { get; private set; } = null!;
    public RelayCommand SimLayerForwardCommand { get; private set; } = null!;
    public RelayCommand SimLayerBackCommand { get; private set; } = null!;

    private void CreateViewerCommands()
    {
        SimPlayPauseCommand = new RelayCommand(() =>
        {
            if (SimPlaying)
            {
                SimPlaying = false;
                return;
            }
            if (ViewerMode != ViewerMode.Simulation || _simTime >= SimTotal)
                SeekSimulation(ViewerMode == ViewerMode.Simulation && _simTime < SimTotal ? _simTime : 0);
            _lastTick = -1;
            SimPlaying = true;
        }, () => CanSimulate);
        SimStopCommand = new RelayCommand(StopSimulation, () => IsSimulation);
        SimStepForwardCommand = new RelayCommand(() => StepSegment(+1), () => CanSimulate);
        SimStepBackCommand = new RelayCommand(() => StepSegment(-1), () => CanSimulate);
        SimLayerForwardCommand = new RelayCommand(() => StepLayer(+1), () => CanSimulate);
        SimLayerBackCommand = new RelayCommand(() => StepLayer(-1), () => CanSimulate);
    }

    private void SeekSimulation(double time)
    {
        var tp = Toolpath;
        if (tp == null || tp.Segments.Length == 0)
            return;
        ViewerMode = ViewerMode.Simulation;
        time = Math.Clamp(time, 0, tp.TotalTime);
        _internalUpdate = true;
        SimTime = time;
        _internalUpdate = false;
        OnPropertyChanged(nameof(SimTime));
        OnPropertyChanged(nameof(SimTimeText));

        int seg = tp.SegmentAtTime((float)time);
        if (seg < 0)
        {
            ShowProgress(-1);
            CurrentLine = -1;
            return;
        }
        var p = tp.Segments[seg].PositionAt((float)time);
        ShowProgress(seg, new Point3D(p.X, p.Y, p.Z));
        CurrentLine = tp.Segments[seg].Line;
    }

    private void StepSegment(int dir)
    {
        var tp = Toolpath;
        if (tp == null || tp.Segments.Length == 0)
            return;
        SimPlaying = false;
        int cur = ViewerMode == ViewerMode.Simulation ? Math.Max(-1, SegmentLimit - 1) : -1;
        int next = Math.Clamp(cur + dir, 0, tp.Segments.Length - 1);
        var s = tp.Segments[next];
        SeekSimulation(s.StartTime + s.Duration * 0.999);
    }

    private void StepLayer(int dir)
    {
        var tp = Toolpath;
        if (tp == null || tp.Layers.Count == 0)
            return;
        SimPlaying = false;
        int cur = ViewerMode == ViewerMode.Simulation ? MaxLayer : (dir > 0 ? -1 : tp.Layers.Count);
        int next = Math.Clamp(cur + dir, 0, tp.Layers.Count - 1);
        var l = tp.Layers[next];
        var s = tp.Segments[Math.Max(l.FirstSegment, l.EndSegment - 1)];
        SeekSimulation(s.EndTime - 1e-4);
    }

    private void StopSimulation()
    {
        SimPlaying = false;
        if (ViewerMode != ViewerMode.Simulation)
            return;
        ViewerMode = ViewerMode.Preview;
        ShowAll();
        CurrentLine = -1;
    }

    private void SimulationTick()
    {
        double now = _clock.Elapsed.TotalSeconds;
        if (!SimPlaying || ViewerMode != ViewerMode.Simulation)
        {
            _lastTick = -1;
            return;
        }
        double dt = _lastTick < 0 ? 0 : now - _lastTick;
        _lastTick = now;
        double t = _simTime + dt * SimSpeed;
        if (t >= SimTotal)
        {
            t = SimTotal;
            SimPlaying = false;
        }
        SeekSimulation(t);
    }

    /// <summary>The user clicked a line in the listing: simulate up to it.</summary>
    public void JumpToLine(int line)
    {
        var tp = Toolpath;
        if (tp == null || tp.Segments.Length == 0 || ViewerMode == ViewerMode.Live)
            return;
        SimPlaying = false;
        int seg = tp.FirstSegmentAtOrAfterLine(line);
        if (seg >= tp.Segments.Length || tp.Segments[seg].Line != line)
            seg = tp.LastSegmentAtOrBeforeLine(line);
        if (seg < 0)
        {
            SeekSimulation(0);
            CurrentLine = line;
            return;
        }
        // The last segment of the line (arcs have many).
        while (seg + 1 < tp.Segments.Length && tp.Segments[seg + 1].Line == tp.Segments[seg].Line)
            seg++;
        var s = tp.Segments[seg];
        SeekSimulation(s.StartTime + s.Duration * 0.999);
        CurrentLine = line;
    }
}
