using System.Diagnostics;

namespace TeacupHost.Core.Printing;

public enum ConnectionState
{
    Disconnected,
    Connecting,
    Online,
    /// <summary>The firmware called printer_kill(); only M999 or reset helps.</summary>
    Halted,
}

/// <summary>
/// Messages of the connection itself. The UI turns them into text in its
/// language; <c>Detail</c> carries the variable part (line number, error).
/// </summary>
public enum ConnectionMessage
{
    /// <summary>The first "ok" arrived.</summary>
    Online,
    /// <summary>The port failed (cable pulled); detail: the error.</summary>
    LinkLost,
    /// <summary>"Resend" for a line no longer in the history; detail: line number.</summary>
    ResendNotInHistory,
    /// <summary>"Resend" for a line never sent, numbering restarted; detail: line number.</summary>
    ResendUnknownLine,
    /// <summary>"start" from the printer.</summary>
    PrinterReset,
    /// <summary>"start" while printing: the print is lost.</summary>
    PrinterResetDuringPrint,
    /// <summary>No answer to M110, sending it again.</summary>
    HandshakeRetry,
    /// <summary>20 s of silence while waiting for "ok", resynchronising with M105.</summary>
    OkTimeout,
}

public enum JobKind
{
    /// <summary>Printing a file from this computer, line by line.</summary>
    Print,
    /// <summary>Uploading a file to SD / SPI flash with M28 … M29.</summary>
    Upload,
}

/// <summary>Why a line went out, the console uses it for filtering.</summary>
public enum SendKind
{
    Manual,
    Job,
    Poll,
    Emergency,
    Resend,
    Handshake,
}

/// <summary>A line of a job: the command without comment and its line in the document.</summary>
public readonly record struct JobLine(string Command, int SourceLine);

/// <summary>
/// Host side of the Marlin/Teacup serial protocol, the same model as
/// printcore (Pronterface): every line gets a number and a checksum, the
/// next line goes out after the "ok" of the previous one, "Resend: n" sends
/// the lines again from n. Manual commands go between the lines of a job.
/// M112, M108 and M410 go out at once, the firmware's emergency parser
/// handles them even while the command queue is full.
/// </summary>
/// <remarks>
/// Events are raised on the reader thread of the transport or on the timer
/// thread, inside the internal lock. Handlers must be short and must not
/// block; a UI should marshal them to its dispatcher.
/// </remarks>
public sealed class PrinterConnection : IDisposable
{
    private const int HistorySize = 4096;

    private readonly struct InFlight
    {
        public InFlight(int lineNumber, int jobIndex)
        {
            LineNumber = lineNumber;
            JobIndex = jobIndex;
        }

        /// <summary>Line number, -1 for an unnumbered emergency line.</summary>
        public int LineNumber { get; }
        public int JobIndex { get; }
    }

    private readonly object _lock = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private IPrinterTransport? _transport;
    private Timer? _timer;

    private readonly Queue<(string Command, SendKind Kind)> _priority = new();
    private readonly LinkedList<InFlight> _inFlight = new();
    private readonly Dictionary<int, string> _history = new();
    private readonly Dictionary<int, int> _lineToJob = new();
    private readonly Dictionary<int, SendKind> _lineKind = new();
    private int _nextLine;
    private int? _resendFrom;

    private double _handshakeSentAt;
    private double _lastRx;
    private double _lastTemperature;
    private double _lastSdPoll;
    private double _waitingSince = -1;

    private IReadOnlyList<JobLine>? _job;
    private JobKind _jobKind;
    private int _jobNext;
    private int _jobAcked;
    private bool _jobPaused;

    private bool _listingFiles;
    private readonly List<SdFileInfo> _files = new();
    private bool _sdPrinting;
    private bool _sdPaused;
    private double _sdStartedAt;

    public ConnectionState State { get; private set; } = ConnectionState.Disconnected;
    public string? PortName => _transport?.Name;

    /// <summary>Seconds between M27 polls while the printer prints from SD.</summary>
    public double SdPollInterval { get; set; } = 2.0;

    /// <summary>M155 interval for temperature auto reports, 0 to poll with M105.</summary>
    public int TemperatureInterval { get; set; } = 2;

    public bool IsJobActive { get { lock (_lock) return _job != null; } }
    public bool IsJobPaused { get { lock (_lock) return _job != null && _jobPaused; } }
    public JobKind ActiveJobKind { get { lock (_lock) return _jobKind; } }
    public int JobAcknowledged { get { lock (_lock) return _jobAcked; } }
    public int JobLength { get { lock (_lock) return _job?.Count ?? 0; } }
    public bool IsSdPrinting { get { lock (_lock) return _sdPrinting; } }
    public bool IsSdPaused { get { lock (_lock) return _sdPrinting && _sdPaused; } }

    public event Action<ConnectionState>? StateChanged;
    public event Action<string>? LineReceived;
    public event Action<string, SendKind>? LineSent;
    /// <summary>Messages of the connection itself (handshake, timeouts, errors).</summary>
    public event Action<ConnectionMessage, string>? Info;
    public event Action<TemperatureReading>? TemperatureUpdated;
    public event Action<PrinterPosition>? PositionUpdated;
    public event Action? Busy;
    public event Action<string>? PrinterError;
    /// <summary>Index of the last job line the printer acknowledged (executed).</summary>
    public event Action<int>? JobProgress;
    /// <summary>Job finished: kind, true if cancelled or aborted.</summary>
    public event Action<JobKind, bool>? JobCompleted;
    public event Action<IReadOnlyList<SdFileInfo>>? SdFilesListed;
    public event Action<long, long>? SdProgress;
    public event Action<string, long>? SdFileSelected;
    /// <summary>Other SD/flash messages: "Done saving file.", "open failed", …</summary>
    public event Action<string>? SdMessage;
    public event Action? SdPrintFinished;

    private double Now => _clock.Elapsed.TotalSeconds;

    // ---------------------------------------------------------------- connect

    public void Connect(IPrinterTransport transport)
    {
        Disconnect();
        // Open first: a busy or missing port throws here and leaves nothing behind.
        try
        {
            transport.Open();
        }
        catch
        {
            transport.Dispose();
            throw;
        }
        lock (_lock)
        {
            _transport = transport;
            transport.LineReceived += OnLine;
            transport.Faulted += OnFaulted;
            ResetProtocol();
            _lastRx = Now;
            SetState(ConnectionState.Connecting);
            SendHandshake();
            _timer = new Timer(_ => OnTimer(), null, 250, 250);
        }
    }

    public void Disconnect() => Disconnect(null);

    private void Disconnect((ConnectionMessage Message, string Detail)? reason)
    {
        IPrinterTransport? t;
        lock (_lock)
        {
            t = _transport;
            if (t == null)
                return;
            _transport = null;
            _timer?.Dispose();
            _timer = null;
            t.LineReceived -= OnLine;
            t.Faulted -= OnFaulted;
            AbortJob();
            _sdPrinting = false;
            ResetProtocol();
            if (reason is { } r)
                Info?.Invoke(r.Message, r.Detail);
            SetState(ConnectionState.Disconnected);
        }
        try
        {
            t.Close();
            t.Dispose();
        }
        catch
        {
            // Nothing to do, the link is gone anyway.
        }
    }

    private void OnFaulted(Exception ex) =>
        ThreadPool.QueueUserWorkItem(_ => Disconnect((ConnectionMessage.LinkLost, ex.Message)));

    private void ResetProtocol()
    {
        _priority.Clear();
        _inFlight.Clear();
        _history.Clear();
        _lineToJob.Clear();
        _lineKind.Clear();
        _resendFrom = null;
        _nextLine = 1;
        _listingFiles = false;
        _waitingSince = -1;
    }

    private void SetState(ConnectionState s)
    {
        if (State == s)
            return;
        State = s;
        StateChanged?.Invoke(s);
    }

    private void SendHandshake()
    {
        // "N0 M110 N0": next line number is 1. Repeated until the first "ok".
        _inFlight.Clear();
        _nextLine = 1;
        string text = ResponseParser.FormatNumbered(0, "M110 N0");
        _inFlight.AddLast(new InFlight(0, -1));
        _handshakeSentAt = Now;
        Write(text, SendKind.Handshake);
    }

    private void OnOnline()
    {
        SetState(ConnectionState.Online);
        Info?.Invoke(ConnectionMessage.Online, "");
        _priority.Enqueue(("M115", SendKind.Poll));
        if (TemperatureInterval > 0)
            _priority.Enqueue(($"M155 S{TemperatureInterval}", SendKind.Poll));
        _priority.Enqueue(("M105", SendKind.Poll));
        _priority.Enqueue(("M114", SendKind.Poll));
        _lastTemperature = Now;
    }

    // ---------------------------------------------------------------- sending

    /// <summary>Queue a command; it goes out between job lines, in order.</summary>
    public void Send(string command, SendKind kind = SendKind.Manual)
    {
        command = command.Trim();
        if (command.Length == 0)
            return;
        if (IsEmergency(command))
        {
            SendEmergency(command);
            return;
        }
        lock (_lock)
        {
            if (_transport == null)
                return;
            TrackSdCommand(command);
            _priority.Enqueue((command, kind));
            Pump();
        }
    }

    /// <summary>Queue several lines (a macro), comments and empty lines dropped.</summary>
    public void SendScript(string script)
    {
        foreach (var raw in script.Split('\n'))
        {
            string cmd = GCode.GCodeCommand.StripComment(raw);
            if (cmd.Length > 0)
                Send(cmd);
        }
    }

    /// <summary>
    /// The link is 8-bit ASCII; other characters (e.g. Cyrillic in M117)
    /// become '?', so the checksum matches what the printer receives.
    /// </summary>
    public static string ToAscii(string s)
    {
        foreach (char c in s)
        {
            if (c > 126 || c < 32)
                return string.Create(s.Length, s, (span, src) =>
                {
                    for (int i = 0; i < src.Length; i++)
                        span[i] = src[i] is >= ' ' and <= '~' ? src[i] : '?';
                });
        }
        return s;
    }

    public static bool IsEmergency(string command)
    {
        var cmd = GCode.GCodeCommand.Parse(command);
        return cmd.Letter == 'M' && cmd.Code is 112 or 108 or 410;
    }

    /// <summary>Send M112 / M108 / M410 right now, past the queues.</summary>
    public void SendEmergency(string command)
    {
        lock (_lock)
        {
            if (_transport == null)
                return;
            // The line still runs through the normal parser and gets an "ok".
            _inFlight.AddLast(new InFlight(-1, -1));
            Write(ToAscii(command), SendKind.Emergency);
            if (command.StartsWith("M112", StringComparison.OrdinalIgnoreCase))
            {
                AbortJob();
                _sdPrinting = false;
            }
        }
    }

    private void Write(string text, SendKind kind)
    {
        var t = _transport;
        if (t == null)
            return;
        t.WriteLine(text);
        LineSent?.Invoke(text, kind);
    }

    private bool NumberedInFlight()
    {
        foreach (var f in _inFlight)
            if (f.LineNumber >= 0)
                return true;
        return false;
    }

    /// <summary>Send the next line if the previous one is acknowledged.</summary>
    private void Pump()
    {
        if (_transport == null || State is ConnectionState.Disconnected or ConnectionState.Connecting)
            return;
        if (NumberedInFlight())
            return;

        if (_resendFrom is int r)
        {
            if (r < _nextLine && _history.TryGetValue(r, out var old))
            {
                _resendFrom = r + 1 < _nextLine ? r + 1 : null;
                _inFlight.AddLast(new InFlight(r, _lineToJob.GetValueOrDefault(r, -1)));
                _waitingSince = Now;
                Write(old, SendKind.Resend);
                return;
            }
            if (r < _nextLine)
                Info?.Invoke(ConnectionMessage.ResendNotInHistory, r.ToString(System.Globalization.CultureInfo.InvariantCulture));
            _resendFrom = null;
        }

        string? cmd = null;
        SendKind kind = SendKind.Manual;
        int jobIndex = -1;
        if (_priority.Count > 0)
        {
            (cmd, kind) = _priority.Dequeue();
        }
        else if (_job != null && !_jobPaused && _jobNext < _job.Count && State == ConnectionState.Online)
        {
            jobIndex = _jobNext++;
            cmd = _job[jobIndex].Command;
            kind = SendKind.Job;
        }
        if (cmd == null)
        {
            CheckJobDone();
            return;
        }

        int n = _nextLine++;
        string text = ResponseParser.FormatNumbered(n, ToAscii(cmd));
        _history[n] = text;
        _history.Remove(n - HistorySize);
        _lineToJob.Remove(n - HistorySize);
        _lineKind.Remove(n - HistorySize);
        if (jobIndex >= 0)
            _lineToJob[n] = jobIndex;
        _lineKind[n] = kind;
        _inFlight.AddLast(new InFlight(n, jobIndex));
        _waitingSince = Now;
        Write(text, kind);
    }

    // ---------------------------------------------------------------- receiving

    private void OnLine(string line)
    {
        lock (_lock)
        {
            if (_transport == null)
                return;
            _lastRx = Now;
            LineReceived?.Invoke(line);

            if (line == "start" || line.StartsWith("start ", StringComparison.Ordinal))
            {
                OnPrinterReset();
                return;
            }

            if (ResponseParser.IsOk(line))
            {
                if (line.Length > 3 && ResponseParser.TryParseTemperature(line, out var okTemp))
                {
                    _lastTemperature = Now;
                    TemperatureUpdated?.Invoke(okTemp);
                }
                OnOk();
                return;
            }

            if (ResponseParser.TryParseResend(line, out int resend))
            {
                OnResend(resend);
                return;
            }

            if (line.StartsWith("Error:", StringComparison.OrdinalIgnoreCase))
            {
                if (ResponseParser.IsFatalError(line))
                {
                    AbortJob();
                    _sdPrinting = false;
                    _priority.Clear();
                    _inFlight.Clear();
                    SetState(ConnectionState.Halted);
                }
                PrinterError?.Invoke(line);
                return;
            }

            if (line.StartsWith("echo:busy", StringComparison.OrdinalIgnoreCase))
            {
                Busy?.Invoke();
                return;
            }

            if (ResponseParser.TryParseTemperature(line, out var temp))
            {
                _lastTemperature = Now;
                TemperatureUpdated?.Invoke(temp);
                return;
            }

            if (ResponseParser.TryParsePosition(line, out var pos))
            {
                PositionUpdated?.Invoke(pos);
                return;
            }

            ParseSdLine(line);
        }
    }

    private void OnOk()
    {
        if (State == ConnectionState.Connecting)
        {
            _inFlight.Clear();
            OnOnline();
            Pump();
            return;
        }
        if (_inFlight.Count > 0)
        {
            var f = _inFlight.First!.Value;
            _inFlight.RemoveFirst();
            bool failed = _resendFrom is int r && f.LineNumber >= r;
            if (!failed && f.JobIndex >= 0 && f.JobIndex + 1 > _jobAcked && _job != null)
            {
                _jobAcked = f.JobIndex + 1;
                JobProgress?.Invoke(f.JobIndex);
            }
        }
        _waitingSince = NumberedInFlight() ? Now : -1;
        Pump();
    }

    private void OnResend(int n)
    {
        if (n >= _nextLine || n < 0)
        {
            Info?.Invoke(ConnectionMessage.ResendUnknownLine, n.ToString(System.Globalization.CultureInfo.InvariantCulture));
            // The printer lost track (reset?), start numbering again.
            _priority.Clear();
            SendHandshake();
            SetState(ConnectionState.Connecting);
            return;
        }
        _resendFrom = n;
        // The firmware drops everything after the bad line, emergency lines
        // sent after it won't get an "ok".
        bool afterBad = false;
        var node = _inFlight.First;
        while (node != null)
        {
            var next = node.Next;
            if (node.Value.LineNumber >= n)
                afterBad = true;
            else if (afterBad && node.Value.LineNumber < 0)
                _inFlight.Remove(node);
            node = next;
        }
    }

    private void OnPrinterReset()
    {
        bool hadJob = _job != null || _sdPrinting;
        Info?.Invoke(hadJob ? ConnectionMessage.PrinterResetDuringPrint : ConnectionMessage.PrinterReset, "");
        AbortJob();
        _sdPrinting = false;
        ResetProtocol();
        SetState(ConnectionState.Connecting);
        SendHandshake();
    }

    private void ParseSdLine(string line)
    {
        if (line.StartsWith("Begin file list", StringComparison.OrdinalIgnoreCase))
        {
            _listingFiles = true;
            _files.Clear();
            return;
        }
        if (line.StartsWith("End file list", StringComparison.OrdinalIgnoreCase))
        {
            _listingFiles = false;
            SdFilesListed?.Invoke(_files.ToArray());
            return;
        }
        if (_listingFiles && ResponseParser.TryParseFileEntry(line, out var file))
        {
            _files.Add(file);
            return;
        }
        if (ResponseParser.TryParseSdProgress(line, out long p, out long size))
        {
            SdProgress?.Invoke(p, size);
            return;
        }
        if (line.StartsWith("Not SD printing", StringComparison.OrdinalIgnoreCase))
        {
            if (_sdPrinting && !_sdPaused && Now - _sdStartedAt > 1.0)
            {
                _sdPrinting = false;
                SdPrintFinished?.Invoke();
            }
            return;
        }
        if (ResponseParser.TryParseFileOpened(line, out string name, out long fsize))
        {
            SdFileSelected?.Invoke(name, fsize);
            return;
        }
        if (line.StartsWith("File selected", StringComparison.OrdinalIgnoreCase))
            return;
        if (line.Contains("open failed", StringComparison.OrdinalIgnoreCase))
            _sdPrinting = false;
        if (line.Contains("open failed", StringComparison.OrdinalIgnoreCase) ||
            line.StartsWith("Writing to file", StringComparison.OrdinalIgnoreCase) ||
            line.StartsWith("Done saving file", StringComparison.OrdinalIgnoreCase) ||
            line.StartsWith("File deleted", StringComparison.OrdinalIgnoreCase) ||
            line.StartsWith("Deletion failed", StringComparison.OrdinalIgnoreCase) ||
            line.Contains("SD card", StringComparison.OrdinalIgnoreCase) ||
            line.Contains("SD init", StringComparison.OrdinalIgnoreCase) ||
            line.Contains("Flash", StringComparison.OrdinalIgnoreCase))
            SdMessage?.Invoke(line);
    }

    // ---------------------------------------------------------------- timer

    private void OnTimer()
    {
        lock (_lock)
        {
            if (_transport == null)
                return;
            double now = Now;

            if (State == ConnectionState.Connecting)
            {
                if (now - _handshakeSentAt > 2.0)
                {
                    Info?.Invoke(ConnectionMessage.HandshakeRetry, "");
                    SendHandshake();
                }
                return;
            }
            if (State != ConnectionState.Online)
                return;

            // No auto report (other firmware, or M155 was switched off): poll.
            if (now - _lastTemperature > 3.0 * Math.Max(TemperatureInterval, 1) + 2)
            {
                if (!_priority.Any(p => p.Command == "M105"))
                    _priority.Enqueue(("M105", SendKind.Poll));
                _lastTemperature = now;
            }

            if (_sdPrinting && !_sdPaused && now - _lastSdPoll > SdPollInterval)
            {
                if (!_priority.Any(p => p.Command == "M27"))
                    _priority.Enqueue(("M27", SendKind.Poll));
                _lastSdPoll = now;
            }

            // A lost "ok": nothing heard for 20 s while waiting. The firmware
            // sends "busy" every 2 s during long commands, so it's really gone.
            if (_waitingSince >= 0 && now - _lastRx > 20 && now - _waitingSince > 20)
            {
                Info?.Invoke(ConnectionMessage.OkTimeout, "");
                _inFlight.Clear();
                _inFlight.AddLast(new InFlight(-1, -1));
                _waitingSince = -1;
                Write("M105", SendKind.Poll);
            }

            Pump();
        }
    }

    // ---------------------------------------------------------------- jobs

    /// <summary>Start printing (or uploading) lines from this computer.</summary>
    public void StartJob(IReadOnlyList<JobLine> lines, JobKind kind = JobKind.Print)
    {
        lock (_lock)
        {
            if (_transport == null || State != ConnectionState.Online)
                throw new InvalidOperationException("The printer is not connected");
            if (_job != null)
                throw new InvalidOperationException("A job is already running");
            _job = lines;
            _jobKind = kind;
            _jobNext = 0;
            _jobAcked = 0;
            _jobPaused = false;
            Pump();
        }
    }

    /// <summary>Upload job: M28 name, the lines, M29.</summary>
    public static List<JobLine> CreateUploadJob(string fileName, IReadOnlyList<JobLine> lines)
    {
        var job = new List<JobLine>(lines.Count + 2) { new("M28 " + fileName, -1) };
        foreach (var l in lines)
        {
            // M28/M29 inside the file would end the upload early.
            var c = GCode.GCodeCommand.Parse(l.Command);
            if (c.Letter == 'M' && c.Code is 28 or 29)
                continue;
            job.Add(l);
        }
        job.Add(new JobLine("M29", -1));
        return job;
    }

    public void PauseJob()
    {
        lock (_lock)
        {
            if (_job != null)
                _jobPaused = true;
        }
    }

    public void ResumeJob()
    {
        lock (_lock)
        {
            if (_job == null)
                return;
            _jobPaused = false;
            Pump();
        }
    }

    public void CancelJob()
    {
        lock (_lock)
        {
            if (_job == null)
                return;
            var kind = _jobKind;
            bool upload = kind == JobKind.Upload && _jobNext > 0;
            _job = null;
            JobCompleted?.Invoke(kind, true);
            // An interrupted upload must still be closed, or the printer keeps
            // storing every following line.
            if (upload)
                _priority.Enqueue(("M29", SendKind.Manual));
            Pump();
        }
    }

    private void AbortJob()
    {
        if (_job == null)
            return;
        var kind = _jobKind;
        _job = null;
        JobCompleted?.Invoke(kind, true);
    }

    private void CheckJobDone()
    {
        if (_job == null || _jobNext < _job.Count)
            return;
        foreach (var f in _inFlight)
            if (f.JobIndex >= 0)
                return;
        var kind = _jobKind;
        _job = null;
        JobCompleted?.Invoke(kind, false);
    }

    // ---------------------------------------------------------------- SD / flash

    private void TrackSdCommand(string command)
    {
        var c = GCode.GCodeCommand.Parse(command);
        if (c.Letter != 'M')
            return;
        switch (c.Code)
        {
            case 24:
                _sdPrinting = true;
                _sdPaused = false;
                _sdStartedAt = Now;
                _lastSdPoll = Now;
                break;
            case 25:
                if (_sdPrinting)
                    _sdPaused = true;
                break;
            case 22:
                _sdPrinting = false;
                break;
        }
    }

    public void SdRefresh() => Send("M20");
    public void SdInit() => Send("M21");

    public void SdStartPrint(string fileName)
    {
        Send("M23 " + fileName);
        Send("M24");
    }

    public void SdPause() => Send("M25");
    public void SdResume() => Send("M24");

    /// <summary>Stop the SD print: pause it and forget the job.</summary>
    public void SdStop()
    {
        Send("M25");
        lock (_lock)
        {
            _sdPrinting = false;
            _sdPaused = false;
        }
    }

    public void SdDelete(string fileName) => Send("M30 " + fileName);

    public void Dispose() => Disconnect();
}
