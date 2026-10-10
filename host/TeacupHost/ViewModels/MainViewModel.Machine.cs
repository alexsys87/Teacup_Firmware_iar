using System.Collections.ObjectModel;
using System.Globalization;
using System.Windows;
using TeacupHost.Controls;
using TeacupHost.Core.Printing;
using TeacupHost.Infrastructure;
using TeacupHost.Services;

namespace TeacupHost.ViewModels;

/// <summary>Temperatures, moving the axes, the extruder, fan, tests and macros.</summary>
public sealed partial class MainViewModel
{
    private static readonly CultureInfo Ci = CultureInfo.InvariantCulture;

    // ---------------------------------------------------------------- temperatures

    private double _hotendTemp, _hotendTarget, _bedTemp, _bedTarget;
    private int _hotendPower, _bedPower;

    public double HotendTemp { get => _hotendTemp; private set => Set(ref _hotendTemp, value); }
    public double HotendTarget { get => _hotendTarget; private set => Set(ref _hotendTarget, value); }
    public double BedTemp { get => _bedTemp; private set => Set(ref _bedTemp, value); }
    public double BedTarget { get => _bedTarget; private set => Set(ref _bedTarget, value); }
    /// <summary>Heater output, percent.</summary>
    public int HotendPower { get => _hotendPower; private set => Set(ref _hotendPower, value); }
    public int BedPower { get => _bedPower; private set => Set(ref _bedPower, value); }

    private double _hotendSetpoint;
    public double HotendSetpoint { get => _hotendSetpoint; set => Set(ref _hotendSetpoint, Math.Clamp(value, 0, 300)); }

    private double _bedSetpoint;
    public double BedSetpoint { get => _bedSetpoint; set => Set(ref _bedSetpoint, Math.Clamp(value, 0, 150)); }

    public double[] HotendPresets => _settings.HotendPresets;
    public double[] BedPresets => _settings.BedPresets;

    private readonly List<TemperatureSample> _samples = new();
    public IReadOnlyList<TemperatureSample> TemperatureSamples => _samples;

    private int _temperatureRevision;
    public int TemperatureRevision { get => _temperatureRevision; private set => Set(ref _temperatureRevision, value); }

    private double _lastSample = -10;

    private void OnTemperature(TemperatureReading t)
    {
        HotendTemp = t.Hotend;
        HotendTarget = t.HotendTarget;
        BedTemp = t.Bed;
        BedTarget = t.BedTarget;
        HotendPower = (int)Math.Round(t.HotendPower * 100 / 255.0);
        BedPower = (int)Math.Round(t.BedPower * 100 / 255.0);

        double now = _clock.Elapsed.TotalSeconds;
        if (now - _lastSample < 0.5)
            return;
        _lastSample = now;
        _samples.Add(new TemperatureSample(now, t.Hotend, t.HotendTarget, t.Bed, t.BedTarget));
        // Keep 30 minutes.
        if (_samples.Count > 4000)
            _samples.RemoveRange(0, 500);
        TemperatureRevision++;
    }

    public RelayCommand SetHotendCommand { get; private set; } = null!;
    public RelayCommand HotendOffCommand { get; private set; } = null!;
    public RelayCommand SetBedCommand { get; private set; } = null!;
    public RelayCommand BedOffCommand { get; private set; } = null!;
    public RelayCommand HotendPresetCommand { get; private set; } = null!;
    public RelayCommand BedPresetCommand { get; private set; } = null!;

    // ---------------------------------------------------------------- position and movement

    private double _posX, _posY, _posZ, _posE;
    public double PosX { get => _posX; private set => Set(ref _posX, value); }
    public double PosY { get => _posY; private set => Set(ref _posY, value); }
    public double PosZ { get => _posZ; private set => Set(ref _posZ, value); }
    public double PosE { get => _posE; private set => Set(ref _posE, value); }

    private void OnPosition(PrinterPosition p)
    {
        PosX = p.X;
        PosY = p.Y;
        PosZ = p.Z;
        PosE = p.E;
    }

    public double[] JogSteps { get; } = { 0.1, 1, 10, 50 };

    private double _jogStep = 10;
    public double JogStep { get => _jogStep; set => Set(ref _jogStep, value); }

    public double JogFeedXY
    {
        get => _settings.JogFeedXY;
        set
        {
            _settings.JogFeedXY = Math.Clamp(value, 60, 12000);
            OnPropertyChanged();
        }
    }

    public double JogFeedZ
    {
        get => _settings.JogFeedZ;
        set
        {
            _settings.JogFeedZ = Math.Clamp(value, 10, 1200);
            OnPropertyChanged();
        }
    }

    private double _extrudeLength;
    public double ExtrudeLength { get => _extrudeLength; set => Set(ref _extrudeLength, Math.Clamp(value, 0.1, 200)); }

    public double ExtrudeFeed
    {
        get => _settings.ExtrudeFeed;
        set
        {
            _settings.ExtrudeFeed = Math.Clamp(value, 10, 3000);
            OnPropertyChanged();
        }
    }

    private int _fanPercent = 100;
    public int FanPercent { get => _fanPercent; set => Set(ref _fanPercent, Math.Clamp(value, 0, 100)); }

    private int _speedFactor = 100;
    public int SpeedFactor { get => _speedFactor; set => Set(ref _speedFactor, Math.Clamp(value, 10, 300)); }

    private int _flowFactor = 100;
    public int FlowFactor { get => _flowFactor; set => Set(ref _flowFactor, Math.Clamp(value, 10, 300)); }

    public RelayCommand JogCommand { get; private set; } = null!;
    public RelayCommand HomeCommand { get; private set; } = null!;
    public RelayCommand ExtrudeCommand { get; private set; } = null!;
    public RelayCommand RetractCommand { get; private set; } = null!;
    public RelayCommand MotorsOffCommand { get; private set; } = null!;
    public RelayCommand GetPositionCommand { get; private set; } = null!;
    public RelayCommand FanOnCommand { get; private set; } = null!;
    public RelayCommand FanOffCommand { get; private set; } = null!;
    public RelayCommand ApplySpeedCommand { get; private set; } = null!;
    public RelayCommand ApplyFlowCommand { get; private set; } = null!;

    // ---------------------------------------------------------------- tests and emergency

    public RelayCommand SendRawCommand { get; private set; } = null!;
    public RelayCommand PidTuneCommand { get; private set; } = null!;
    public RelayCommand EmergencyStopCommand { get; private set; } = null!;
    public RelayCommand ResetCommand { get; private set; } = null!;
    public RelayCommand QuickStopCommand { get; private set; } = null!;
    public RelayCommand CancelWaitCommand { get; private set; } = null!;

    // ---------------------------------------------------------------- macros

    public ObservableCollection<MacroItem> Macros { get; } = new();

    private MacroItem? _selectedMacro;
    public MacroItem? SelectedMacro { get => _selectedMacro; set => Set(ref _selectedMacro, value); }

    public RelayCommand RunMacroCommand { get; private set; } = null!;
    public RelayCommand AddMacroCommand { get; private set; } = null!;
    public RelayCommand DeleteMacroCommand { get; private set; } = null!;

    private static string N(double v) => v.ToString("0.###", Ci);

    private void CreateMachineCommands()
    {
        bool Online() => IsOnline;
        bool Control() => CanControl;

        SetHotendCommand = new RelayCommand(() => _conn.Send($"M104 S{N(HotendSetpoint)}"), Online);
        HotendOffCommand = new RelayCommand(() => _conn.Send("M104 S0"), Online);
        SetBedCommand = new RelayCommand(() => _conn.Send($"M140 S{N(BedSetpoint)}"), Online);
        BedOffCommand = new RelayCommand(() => _conn.Send("M140 S0"), Online);
        HotendPresetCommand = new RelayCommand(p =>
        {
            if (p is double v || double.TryParse(p?.ToString(), NumberStyles.Float, Ci, out v))
            {
                HotendSetpoint = v;
                _conn.Send($"M104 S{N(v)}");
            }
        }, _ => IsOnline);
        BedPresetCommand = new RelayCommand(p =>
        {
            if (p is double v || double.TryParse(p?.ToString(), NumberStyles.Float, Ci, out v))
            {
                BedSetpoint = v;
                _conn.Send($"M140 S{N(v)}");
            }
        }, _ => IsOnline);

        JogCommand = new RelayCommand(p => Jog(p as string ?? ""), _ => CanControl);
        HomeCommand = new RelayCommand(p =>
        {
            string axes = p as string ?? "";
            _conn.Send(axes.Length == 0 ? "G28" : "G28 " + string.Join(' ', axes.ToCharArray()));
            _conn.Send("M114", SendKind.Poll);
        }, _ => CanControl);
        ExtrudeCommand = new RelayCommand(() => Extrude(ExtrudeLength), Control);
        RetractCommand = new RelayCommand(() => Extrude(-ExtrudeLength), Control);
        MotorsOffCommand = new RelayCommand(() => _conn.Send("M84"), Control);
        GetPositionCommand = new RelayCommand(() => _conn.Send("M114"), Online);
        FanOnCommand = new RelayCommand(() => _conn.Send($"M106 S{(int)Math.Round(FanPercent * 2.55)}"), Online);
        FanOffCommand = new RelayCommand(() => _conn.Send("M107"), Online);
        ApplySpeedCommand = new RelayCommand(() => _conn.Send($"M220 S{SpeedFactor}"), Online);
        ApplyFlowCommand = new RelayCommand(() => _conn.Send($"M221 S{FlowFactor}"), Online);

        SendRawCommand = new RelayCommand(p =>
        {
            if (p is string s)
                _conn.SendScript(s);
        }, _ => IsOnline);
        PidTuneCommand = new RelayCommand(() =>
        {
            if (MessageBox.Show(Loc.F("S.Ask.PidTune", N(HotendSetpoint)),
                    Loc.T("S.Ask.PidTuneTitle"), MessageBoxButton.OKCancel, MessageBoxImage.Question) == MessageBoxResult.OK)
                _conn.Send($"M303 E0 S{N(HotendSetpoint)} C8");
        }, Control);
        EmergencyStopCommand = new RelayCommand(() =>
        {
            _conn.SendEmergency("M112");
            if (PrintSource != PrintSource.None)
                EndLivePrint(Loc.T("S.Log.EmergencyStop"));
            Log(LogKind.Error, Loc.T("S.Log.EmergencyStop"));
        }, () => IsConnected);
        ResetCommand = new RelayCommand(() => _conn.Send("M999"), () => IsConnected);
        QuickStopCommand = new RelayCommand(() => _conn.SendEmergency("M410"), () => IsConnected);
        CancelWaitCommand = new RelayCommand(() => _conn.SendEmergency("M108"), () => IsConnected);

        RunMacroCommand = new RelayCommand(p =>
        {
            if (p is MacroItem m)
            {
                Log(LogKind.Info, Loc.F("S.Log.Macro", m.Name));
                _conn.SendScript(m.Script);
            }
        }, _ => IsOnline);
        AddMacroCommand = new RelayCommand(() =>
        {
            var m = new MacroItem(Loc.T("S.NewMacro"), "M105");
            Macros.Add(m);
            SelectedMacro = m;
        });
        DeleteMacroCommand = new RelayCommand(() =>
        {
            if (SelectedMacro != null)
                Macros.Remove(SelectedMacro);
        }, () => SelectedMacro != null);
    }

    private void Jog(string axis)
    {
        if (axis.Length < 2)
            return;
        char a = char.ToUpperInvariant(axis[0]);
        double d = axis[1] == '-' ? -JogStep : JogStep;
        if (a == 'Z')
            d = Math.Sign(d) * Math.Min(Math.Abs(d), 10);       // Z is slow, 10 mm at most.
        double feed = a == 'Z' ? JogFeedZ : JogFeedXY;
        SendJog(a == 'X' ? d : 0, a == 'Y' ? d : 0, a == 'Z' ? d : 0, feed, poll: true);
    }

    /// <summary>
    /// A jog or an extrusion is four lines at most. This many lines waiting are two jogs: more
    /// clicks are ignored, otherwise every click is a move that waits in the queue and the machine
    /// runs on for seconds after the last one (40 fast clicks were 128 lines and 6.7 s).
    /// </summary>
    private const int MaxQueuedJogLines = 8;

    /// <summary>A relative move of one or more axes (G91, G1, G90), used by the buttons, the keyboard and the gamepad.</summary>
    private void SendJog(double x, double y, double z, double feed, bool poll)
    {
        if (_conn.QueuedCommands >= MaxQueuedJogLines)
            return;
        var axes = new List<string>(3);
        if (x != 0)
            axes.Add("X" + N(x));
        if (y != 0)
            axes.Add("Y" + N(y));
        if (z != 0)
            axes.Add("Z" + N(z));
        if (axes.Count == 0)
            return;
        _conn.Send("G91");
        _conn.Send($"G1 {string.Join(' ', axes)} F{N(feed)}");
        _conn.Send("G90");
        if (poll)
            _conn.Send("M114", SendKind.Poll);
    }

    /// <param name="interactive">Ask before extruding cold (buttons). Keys and the gamepad can't answer a dialog: they only warn.</param>
    private void Extrude(double length, bool interactive = true)
    {
        if (_conn.QueuedCommands >= MaxQueuedJogLines)
            return;
        if (HotendTemp < 170)
        {
            if (!interactive)
            {
                Notify(Loc.T("S.Notice.ColdExtrudeTitle"), Loc.F("S.Notice.ColdExtrudeText", HotendTemp), NotifySeverity.Info);
                return;
            }
            if (MessageBox.Show(Loc.F("S.Ask.ColdExtrude", HotendTemp),
                    Loc.T("S.Ask.ColdExtrudeTitle"), MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
                return;
        }
        _conn.Send("G91");
        _conn.Send($"G1 E{N(length)} F{N(ExtrudeFeed)}");
        _conn.Send("G90");
    }
}
