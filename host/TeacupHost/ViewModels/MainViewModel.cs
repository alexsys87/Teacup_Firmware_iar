using System.Collections.Concurrent;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Windows;
using System.Windows.Threading;
using TeacupHost.Core.Printing;
using TeacupHost.Infrastructure;
using TeacupHost.Services;

namespace TeacupHost.ViewModels;

/// <summary>
/// The whole application state. Split into partial files: connection and
/// console here, machine control, jobs (print / SD / upload) and the viewer
/// (simulation, G-code listing) in the others.
/// </summary>
public sealed partial class MainViewModel : ObservableObject, IDisposable
{
    /// <summary>Item of the port list that stands for the virtual printer (shown localized).</summary>
    public const string VirtualPortName = "\u0001virtual";
    private const int MaxLogEntries = 3000;

    private readonly AppSettings _settings;
    private readonly PrinterConnection _conn = new();
    private readonly Dispatcher _dispatcher;
    private readonly DispatcherTimer _pump;
    private readonly ConcurrentQueue<LogEntry> _pendingLog = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();

    public MainViewModel(AppSettings settings)
    {
        _settings = settings;
        _dispatcher = Application.Current.Dispatcher;

        _isDarkTheme = ThemeService.IsDark;
        foreach (var m in settings.Macros ?? DefaultMacros())
            Macros.Add(new MacroItem(m.Name, m.Script));
        _selectedBaud = settings.BaudRate;
        _hotendSetpoint = settings.HotendSetpoint;
        _bedSetpoint = settings.BedSetpoint;
        _showTravel = settings.ShowTravel;
        _hideServiceLines = settings.HideTemperatureLines;
        _showJobLines = settings.ShowJobLines;
        _extrudeLength = settings.ExtrudeLength;
        _virtualTimeScale = settings.VirtualTimeScale;

        HookConnection();
        CreateCommands();
        Loc.Changed += OnLanguageChanged;
        RefreshPorts();
        _selectedPort = settings.Port != null && Ports.Contains(settings.Port) ? settings.Port : Ports.FirstOrDefault();

        _pump = new DispatcherTimer(DispatcherPriority.Background, _dispatcher)
        {
            Interval = TimeSpan.FromMilliseconds(40),
        };
        _pump.Tick += (_, _) => OnPump();
        _pump.Start();
    }

    public AppSettings Settings => _settings;

    // ---------------------------------------------------------------- theme and language

    private bool _isDarkTheme;
    public bool IsDarkTheme
    {
        get => _isDarkTheme;
        set
        {
            if (!Set(ref _isDarkTheme, value))
                return;
            ThemeService.Apply(value);
            _settings.DarkTheme = value;
        }
    }

    /// <summary>Code of the current language for the toolbar button, "RU" or "EN".</summary>
    public string LanguageCode => Loc.Language.ToUpperInvariant();

    public RelayCommand ToggleThemeCommand { get; private set; } = null!;
    public RelayCommand ToggleLanguageCommand { get; private set; } = null!;

    private void ToggleLanguage()
    {
        Loc.Apply(Loc.Language == Loc.Russian ? Loc.English : Loc.Russian);
        _settings.Language = Loc.Language;
    }

    /// <summary>Texts built in code follow the new language.</summary>
    private void OnLanguageChanged()
    {
        RefreshPorts();
        RebuildFileInfo();
        // File sizes are formatted when the items are made.
        var selected = SelectedSdFile?.Name;
        var files = SdFiles.Select(f => new SdFileItem(f.Name, f.Size)).ToList();
        SdFiles.Clear();
        foreach (var f in files)
            SdFiles.Add(f);
        SelectedSdFile = SdFiles.FirstOrDefault(f => f.Name == selected);
        if (_sdFilesReceived)
            SdStatus = SdFiles.Count == 0 ? Loc.T("S.SdNoFiles") : Loc.F("S.SdFileCount", SdFiles.Count);
        else
            SdStatus = Loc.T("S.SdNotListed");
        // Refresh every computed text (connection, mode, legend, line numbers …).
        OnPropertyChanged(string.Empty);
    }

    private static IEnumerable<MacroSettings> DefaultMacros() => new MacroSettings[]
    {
        new() { Name = Loc.T("S.Macro.PreheatPla"), Script = "M104 S205\nM140 S60" },
        new() { Name = Loc.T("S.Macro.PreheatPetg"), Script = "M104 S240\nM140 S80" },
        new() { Name = Loc.T("S.Macro.Cooldown"), Script = "M104 S0\nM140 S0" },
        new() { Name = Loc.T("S.Macro.Park"), Script = "G91\nG1 Z10 F240\nG90\nG1 X10 Y170 F6000" },
        new() { Name = Loc.T("S.Macro.BedLevel"), Script = "G28\nG29" },
        new() { Name = Loc.T("S.Macro.FilamentChange"), Script = "M600" },
        new() { Name = Loc.T("S.Macro.Stats"), Script = "M78" },
    };

    // ---------------------------------------------------------------- connection

    public ObservableCollection<string> Ports { get; } = new();
    public int[] BaudRates { get; } = { 115200, 250000, 230400, 500000, 57600, 38400, 19200, 9600 };

    private string? _selectedPort;
    public string? SelectedPort
    {
        get => _selectedPort;
        set
        {
            if (Set(ref _selectedPort, value))
                OnPropertyChanged(nameof(IsVirtualSelected));
        }
    }

    public bool IsVirtualSelected => SelectedPort == VirtualPortName;

    private int _selectedBaud;
    public int SelectedBaud
    {
        get => _selectedBaud;
        set => Set(ref _selectedBaud, value);
    }

    private double _virtualTimeScale;
    /// <summary>Speed of time of the virtual printer.</summary>
    public double VirtualTimeScale
    {
        get => _virtualTimeScale;
        set => Set(ref _virtualTimeScale, Math.Clamp(value, 1, 1000));
    }

    private ConnectionState _connectionState = ConnectionState.Disconnected;
    public ConnectionState ConnectionState
    {
        get => _connectionState;
        private set
        {
            if (!Set(ref _connectionState, value))
                return;
            OnPropertyChanged(nameof(IsConnected));
            OnPropertyChanged(nameof(IsOnline));
            OnPropertyChanged(nameof(IsDisconnected));
            OnPropertyChanged(nameof(ConnectionText));
            OnPropertyChanged(nameof(CanControl));
            RelayCommand.Refresh();
        }
    }

    public bool IsConnected => ConnectionState != ConnectionState.Disconnected;
    public bool IsDisconnected => ConnectionState == ConnectionState.Disconnected;
    public bool IsOnline => ConnectionState == ConnectionState.Online;

    /// <summary>Manual control is allowed: online and not printing.</summary>
    public bool CanControl => IsOnline && PrintSource == PrintSource.None && !IsUploading;

    public string ConnectionText => ConnectionState switch
    {
        ConnectionState.Disconnected => Loc.T("S.State.Disconnected"),
        ConnectionState.Connecting => Loc.T("S.State.Connecting"),
        ConnectionState.Online => Loc.F("S.State.Online",
            _conn.PortName == "Virtual printer" ? Loc.T("S.VirtualPrinter") : _conn.PortName ?? ""),
        ConnectionState.Halted => Loc.T("S.State.Halted"),
        _ => "",
    };

    private string _firmwareName = "";
    public string FirmwareName
    {
        get => _firmwareName;
        private set => Set(ref _firmwareName, value);
    }

    public RelayCommand RefreshPortsCommand { get; private set; } = null!;
    public RelayCommand ConnectCommand { get; private set; } = null!;
    public RelayCommand DisconnectCommand { get; private set; } = null!;

    public void RefreshPorts()
    {
        var current = SelectedPort;
        Ports.Clear();
        foreach (var p in SerialPortTransport.GetPortNames())
            Ports.Add(p);
        Ports.Add(VirtualPortName);
        SelectedPort = current != null && Ports.Contains(current) ? current : Ports.FirstOrDefault();
    }

    private void Connect()
    {
        if (SelectedPort == null)
            return;
        try
        {
            IPrinterTransport transport = SelectedPort == VirtualPortName
                ? new VirtualPrinter { TimeScale = VirtualTimeScale }
                : new SerialPortTransport(SelectedPort, SelectedBaud);
            Log(LogKind.Info, transport is VirtualPrinter
                ? Loc.F("S.Log.ConnectingVirtual", VirtualTimeScale)
                : Loc.F("S.Log.Connecting", transport.Name, SelectedBaud));
            _conn.Connect(transport);
            _settings.Port = SelectedPort;
            _settings.BaudRate = SelectedBaud;
        }
        catch (Exception ex)
        {
            Log(LogKind.Error, Loc.F("S.Log.PortFailed", ex.Message));
            Notify(Loc.T("S.Notice.ConnectFailed"), ex.Message, NotifySeverity.Error);
        }
    }

    private void Disconnect()
    {
        if (PrintSource != PrintSource.None &&
            MessageBox.Show(Loc.T("S.Ask.DisconnectWhilePrinting"), "Teacup Host",
                MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
            return;
        _conn.Disconnect();
    }

    private void HookConnection()
    {
        _conn.StateChanged += s => Ui(() => OnStateChanged(s));
        _conn.LineReceived += OnLineReceived;
        _conn.LineSent += OnLineSent;
        _conn.Info += (msg, detail) =>
            _pendingLog.Enqueue(new LogEntry(LogKind.Info, Loc.F("S.Conn." + msg, detail)));
        _conn.TemperatureUpdated += t => Ui(() => OnTemperature(t));
        _conn.PositionUpdated += p => Ui(() => OnPosition(p));
        _conn.PrinterError += e => Ui(() => OnPrinterError(e));
        _conn.JobProgress += i => Interlocked.Exchange(ref _pendingJobAck, i);
        _conn.JobCompleted += (kind, cancelled) => Ui(() => OnJobCompleted(kind, cancelled));
        _conn.SdFilesListed += files => Ui(() => OnSdFiles(files));
        _conn.SdProgress += (p, s) =>
        {
            Interlocked.Exchange(ref _pendingSdSize, s);
            Interlocked.Exchange(ref _pendingSdPos, p);
        };
        _conn.SdFileSelected += (name, size) => Ui(() => SdStatus = Loc.F("S.SdSelected", name, size));
        _conn.SdMessage += msg => Ui(() => OnSdMessage(msg));
        _conn.SdPrintFinished += () => Ui(OnSdPrintFinished);
    }

    private void Ui(Action a) => _dispatcher.BeginInvoke(a, DispatcherPriority.Normal);

    private void OnStateChanged(ConnectionState s)
    {
        var old = ConnectionState;
        ConnectionState = s;
        if (s == ConnectionState.Disconnected)
        {
            if (PrintSource == PrintSource.Sd)
                EndLivePrint(Loc.T("S.Log.SdLostOnDisconnect"));
            HotendTarget = BedTarget = 0;
            FirmwareName = "";
            Log(LogKind.Info, Loc.T("S.Log.Disconnected"));
        }
        else if (s == ConnectionState.Online && old != ConnectionState.Online)
        {
            _conn.SdRefresh();
        }
        else if (s == ConnectionState.Halted)
        {
            if (PrintSource == PrintSource.Sd)
                EndLivePrint(Loc.T("S.Log.PrinterHalted"));
            Notify(Loc.T("S.Notice.Halted"), Loc.T("S.Notice.HaltedText"), NotifySeverity.Error);
        }
        OnPropertyChanged(nameof(CanControl));
    }

    private void OnPrinterError(string e)
    {
        if (ResponseParser.IsFatalError(e))
            Notify(Loc.T("S.Notice.PrinterError"), e, NotifySeverity.Error);
    }

    // ---------------------------------------------------------------- notifications

    public enum NotifySeverity
    {
        Info,
        Success,
        Warning,
        Error,
    }

    private string _noticeTitle = "";
    public string NoticeTitle
    {
        get => _noticeTitle;
        private set => Set(ref _noticeTitle, value);
    }

    private string _noticeMessage = "";
    public string NoticeMessage
    {
        get => _noticeMessage;
        private set => Set(ref _noticeMessage, value);
    }

    private NotifySeverity _noticeSeverity;
    public NotifySeverity NoticeSeverity
    {
        get => _noticeSeverity;
        private set => Set(ref _noticeSeverity, value);
    }

    private bool _noticeOpen;
    public bool NoticeOpen
    {
        get => _noticeOpen;
        set => Set(ref _noticeOpen, value);
    }

    private DispatcherTimer? _noticeTimer;

    public void Notify(string title, string message, NotifySeverity severity = NotifySeverity.Info)
    {
        NoticeTitle = title;
        NoticeMessage = message;
        NoticeSeverity = severity;
        NoticeOpen = true;
        // Errors and warnings stay until closed, the rest go away by themselves.
        _noticeTimer?.Stop();
        if (severity is NotifySeverity.Info or NotifySeverity.Success)
        {
            _noticeTimer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(8) };
            _noticeTimer.Tick += (_, _) =>
            {
                _noticeTimer?.Stop();
                NoticeOpen = false;
            };
            _noticeTimer.Start();
        }
    }

    // ---------------------------------------------------------------- console

    public ObservableCollection<LogEntry> LogEntries { get; } = new();

    private string _consoleInput = "";
    public string ConsoleInput
    {
        get => _consoleInput;
        set => Set(ref _consoleInput, value);
    }

    private bool _hideServiceLines;
    /// <summary>Hide "ok", temperature reports, busy and M27/M105 polls in the console.</summary>
    public bool HideServiceLines
    {
        get => _hideServiceLines;
        set
        {
            if (Set(ref _hideServiceLines, value))
                _settings.HideTemperatureLines = value;
        }
    }

    private bool _showJobLines;
    /// <summary>Show the lines of a running print or upload in the console.</summary>
    public bool ShowJobLines
    {
        get => _showJobLines;
        set
        {
            if (Set(ref _showJobLines, value))
                _settings.ShowJobLines = value;
        }
    }

    private bool _autoScrollConsole = true;
    public bool AutoScrollConsole
    {
        get => _autoScrollConsole;
        set => Set(ref _autoScrollConsole, value);
    }

    private readonly List<string> _history = new();
    private int _historyPos;

    public RelayCommand SendConsoleCommand { get; private set; } = null!;
    public RelayCommand ClearConsoleCommand { get; private set; } = null!;

    /// <summary>Raised after new console lines were added (the view scrolls down).</summary>
    public event Action? LogAppended;

    private void SendConsole()
    {
        string text = ConsoleInput.Trim();
        if (text.Length == 0)
            return;
        if (_history.Count == 0 || _history[^1] != text)
            _history.Add(text);
        _historyPos = _history.Count;
        ConsoleInput = "";
        foreach (var part in text.Split(new[] { '\n', '|' }, StringSplitOptions.RemoveEmptyEntries))
            _conn.Send(part.Trim().ToUpperInvariant().StartsWith("M117") ? part.Trim() : part.Trim().ToUpperInvariant());
    }

    public void HistoryUp()
    {
        if (_history.Count == 0)
            return;
        _historyPos = Math.Max(0, _historyPos - 1);
        ConsoleInput = _history[_historyPos];
    }

    public void HistoryDown()
    {
        if (_history.Count == 0)
            return;
        _historyPos = Math.Min(_history.Count, _historyPos + 1);
        ConsoleInput = _historyPos < _history.Count ? _history[_historyPos] : "";
    }

    private static bool IsServiceLine(string line) =>
        line == "ok" || line.StartsWith("ok ", StringComparison.Ordinal) && line.Contains("T:") ||
        line.StartsWith("T:", StringComparison.Ordinal) ||
        line.StartsWith("echo:busy", StringComparison.Ordinal) ||
        line.StartsWith("SD printing byte", StringComparison.Ordinal) ||
        line.StartsWith("Not SD printing", StringComparison.Ordinal);

    private void OnLineReceived(string line)
    {
        if (line.StartsWith("FIRMWARE_NAME:", StringComparison.Ordinal))
        {
            string name = line[14..];
            int sp = name.IndexOf(" FIRMWARE_URL", StringComparison.Ordinal);
            if (sp < 0)
                sp = name.IndexOf(' ');
            string fw = sp > 0 ? name[..sp] : name;
            int mt = line.IndexOf("MACHINE_TYPE:", StringComparison.Ordinal);
            if (mt >= 0)
                fw += " · " + line[(mt + 13)..].Split(' ')[0];
            Ui(() => FirmwareName = fw);
        }
        if (_hideServiceLines && IsServiceLine(line))
            return;
        bool error = line.StartsWith("Error", StringComparison.OrdinalIgnoreCase) ||
                     line.StartsWith("!!", StringComparison.Ordinal);
        bool warn = line.StartsWith("Resend", StringComparison.OrdinalIgnoreCase) ||
                    line.Contains("Unknown command", StringComparison.OrdinalIgnoreCase);
        _pendingLog.Enqueue(new LogEntry(error ? LogKind.Error : warn ? LogKind.Warning : LogKind.Received, line));
    }

    private void OnLineSent(string text, SendKind kind)
    {
        if ((kind is SendKind.Job or SendKind.Resend) && !_showJobLines)
            return;
        if (kind == SendKind.Poll && _hideServiceLines)
            return;
        _pendingLog.Enqueue(new LogEntry(LogKind.Sent, text));
    }

    public void Log(LogKind kind, string text) => _pendingLog.Enqueue(new LogEntry(kind, text));

    private void FlushLog()
    {
        if (_pendingLog.IsEmpty)
            return;
        int added = 0;
        while (_pendingLog.TryDequeue(out var e) && added < 500)
        {
            LogEntries.Add(e);
            added++;
        }
        if (LogEntries.Count > MaxLogEntries)
        {
            int remove = LogEntries.Count - MaxLogEntries + 300;
            for (int i = 0; i < remove; i++)
                LogEntries.RemoveAt(0);
        }
        LogAppended?.Invoke();
    }

    // ---------------------------------------------------------------- pump

    private int _pendingJobAck = -1;
    private long _pendingSdPos = -1;
    private long _pendingSdSize;

    private void OnPump()
    {
        FlushLog();

        int ack = Interlocked.Exchange(ref _pendingJobAck, -1);
        if (ack >= 0)
            OnJobAck(ack);

        long sdPos = Interlocked.Exchange(ref _pendingSdPos, -1);
        if (sdPos >= 0)
            OnSdProgress(sdPos, Interlocked.Read(ref _pendingSdSize));

        SimulationTick();
        UpdateElapsed();
    }

    // ---------------------------------------------------------------- commands

    private void CreateCommands()
    {
        RefreshPortsCommand = new RelayCommand(RefreshPorts, () => IsDisconnected);
        ConnectCommand = new RelayCommand(Connect, () => IsDisconnected && SelectedPort != null);
        DisconnectCommand = new RelayCommand(Disconnect, () => IsConnected);
        ToggleThemeCommand = new RelayCommand(() => IsDarkTheme = !IsDarkTheme);
        ToggleLanguageCommand = new RelayCommand(ToggleLanguage);
        SendConsoleCommand = new RelayCommand(SendConsole, () => IsConnected);
        ClearConsoleCommand = new RelayCommand(() => LogEntries.Clear());
        CreateMachineCommands();
        CreateJobCommands();
        CreateViewerCommands();
    }

    public void SaveSettings()
    {
        _settings.HotendSetpoint = HotendSetpoint;
        _settings.BedSetpoint = BedSetpoint;
        _settings.ShowTravel = ShowTravel;
        _settings.ExtrudeLength = ExtrudeLength;
        _settings.VirtualTimeScale = VirtualTimeScale;
        _settings.BaudRate = SelectedBaud;
        _settings.Macros = Macros.Select(m => new MacroSettings { Name = m.Name, Script = m.Script }).ToList();
        _settings.Save();
    }

    public void Dispose()
    {
        Loc.Changed -= OnLanguageChanged;
        _pump.Stop();
        _conn.Dispose();
    }
}
