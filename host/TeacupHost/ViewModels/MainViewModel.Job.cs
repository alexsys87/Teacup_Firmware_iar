using System.Collections.ObjectModel;
using System.IO;
using System.Text.RegularExpressions;
using System.Windows;
using Microsoft.Win32;
using TeacupHost.Core.GCode;
using TeacupHost.Core.Printing;
using TeacupHost.Infrastructure;
using TeacupHost.Services;

namespace TeacupHost.ViewModels;

/// <summary>G-code files, printing from this computer, the SD card / SPI flash and uploads.</summary>
public sealed partial class MainViewModel
{
    // ---------------------------------------------------------------- file

    private GCodeDocument? _document;
    public GCodeDocument? Document { get => _document; private set => Set(ref _document, value); }

    private Toolpath? _toolpath;
    public Toolpath? Toolpath
    {
        get => _toolpath;
        private set
        {
            if (!Set(ref _toolpath, value))
                return;
            OnPropertyChanged(nameof(LayerCount));
            OnPropertyChanged(nameof(LayerSliderMax));
            OnPropertyChanged(nameof(HasToolpath));
            OnPropertyChanged(nameof(CanSimulate));
            OnPropertyChanged(nameof(SimTotal));
            OnPropertyChanged(nameof(SimTimeText));
            OnPropertyChanged(nameof(Legend));
        }
    }

    public bool HasToolpath => Toolpath is { Segments.Length: > 0 };

    private IReadOnlyList<GCodeLineItem> _gcodeLines = [];
    public IReadOnlyList<GCodeLineItem> GCodeLines { get => _gcodeLines; private set => Set(ref _gcodeLines, value); }

    private string _fileInfo = Loc.T("S.NoFile");
    public string FileInfo { get => _fileInfo; private set => Set(ref _fileInfo, value); }

    private string _fileName = "";
    public string FileName { get => _fileName; private set => Set(ref _fileName, value); }

    private bool _isLoading;
    public bool IsLoading { get => _isLoading; private set => Set(ref _isLoading, value); }

    private double _loadProgress;
    public double LoadProgress { get => _loadProgress; private set => Set(ref _loadProgress, value); }

    /// <summary>Types of extrusion present in the file, for the legend.</summary>
    public IReadOnlyList<LegendItem> Legend
    {
        get
        {
            var tp = Toolpath;
            if (tp == null)
                return [];
            var seen = new bool[Controls.FeaturePalette.Count];
            foreach (ref readonly var s in tp.Segments.AsSpan())
                seen[(int)s.Feature] = true;
            var list = new List<LegendItem>();
            for (int i = 0; i < seen.Length; i++)
                if (seen[i])
                    list.Add(new LegendItem((FeatureType)i));
            return list;
        }
    }

    public RelayCommand OpenFileCommand { get; private set; } = null!;
    public RelayCommand OpenDemoCommand { get; private set; } = null!;

    private void OpenFile()
    {
        var dlg = new OpenFileDialog
        {
            Title = Loc.T("S.OpenDialogTitle"),
            Filter = Loc.T("S.OpenDialogFilter"),
            InitialDirectory = _settings.LastFolder ?? "",
        };
        if (dlg.ShowDialog() != true)
            return;
        _settings.LastFolder = Path.GetDirectoryName(dlg.FileName);
        _ = LoadFileAsync(dlg.FileName);
    }

    public bool CanLoadFile => !IsLoading && PrintSource != PrintSource.Host && !IsUploading;

    public async Task LoadFileAsync(string path)
    {
        if (!CanLoadFile)
            return;
        await LoadAsync(Path.GetFileName(path), () => GCodeDocument.Load(path));
    }

    // ---------------------------------------------------------------- test cube

    // Leave room on the bed for the skirt and above the cube for the final lift.
    private double MaxCubeWidth => Math.Max(DemoGCode.MinSide, BedWidth - 2 * DemoGCode.SkirtMargin);
    private double MaxCubeDepth => Math.Max(DemoGCode.MinSide, BedDepth - 2 * DemoGCode.SkirtMargin);
    private double MaxCubeHeight => Math.Max(DemoGCode.MaxLayerHeight, _settings.BedHeight - DemoGCode.EndLift);

    public double CubeWidth
    {
        get => _settings.CubeWidth;
        set => SetCubeSize(() => _settings.CubeWidth = Math.Clamp(value, DemoGCode.MinSide, MaxCubeWidth));
    }

    public double CubeDepth
    {
        get => _settings.CubeDepth;
        set => SetCubeSize(() => _settings.CubeDepth = Math.Clamp(value, DemoGCode.MinSide, MaxCubeDepth));
    }

    public double CubeHeight
    {
        get => _settings.CubeHeight;
        set => SetCubeSize(() => _settings.CubeHeight = Math.Clamp(value, DemoGCode.MinLayerHeight, MaxCubeHeight));
    }

    public double CubeLayerHeight
    {
        get => _settings.CubeLayerHeight;
        set => SetCubeSize(() => _settings.CubeLayerHeight =
            Math.Clamp(value, DemoGCode.MinLayerHeight, DemoGCode.MaxLayerHeight));
    }

    /// <summary>"100 × 100 × 100" in millimetres.</summary>
    public string CubeSizeText => $"{N(CubeWidth)} × {N(CubeDepth)} × {N(CubeHeight)}";

    public string CubeTip => Loc.F("S.CubeTip", CubeSizeText);

    private void SetCubeSize(Action apply)
    {
        apply();
        // NaN from an emptied box: back to the default size.
        if (double.IsNaN(_settings.CubeWidth)) _settings.CubeWidth = 100;
        if (double.IsNaN(_settings.CubeDepth)) _settings.CubeDepth = 100;
        if (double.IsNaN(_settings.CubeHeight)) _settings.CubeHeight = 100;
        if (double.IsNaN(_settings.CubeLayerHeight)) _settings.CubeLayerHeight = 0.2;
        OnPropertyChanged(nameof(CubeWidth));
        OnPropertyChanged(nameof(CubeDepth));
        OnPropertyChanged(nameof(CubeHeight));
        OnPropertyChanged(nameof(CubeLayerHeight));
        OnPropertyChanged(nameof(CubeSizeText));
        OnPropertyChanged(nameof(CubeTip));
    }

    /// <summary>Generates the test cube of the size set on the Print tab, centred on the bed.</summary>
    private Task LoadDemoAsync()
    {
        // Settings from an older version or edited by hand may be out of range.
        CubeWidth = CubeWidth;
        CubeDepth = CubeDepth;
        CubeHeight = CubeHeight;
        CubeLayerHeight = CubeLayerHeight;
        double w = CubeWidth, d = CubeDepth, layer = CubeLayerHeight;
        double h = Math.Max(CubeHeight, layer);
        string name = w == d && d == h ? $"cube{N(w)}.gcode" : $"cube{N(w)}x{N(d)}x{N(h)}.gcode";
        double cx = BedWidth / 2, cy = BedDepth / 2;
        return LoadAsync(name, () => GCodeDocument.FromText(name, DemoGCode.Generate(cx, cy, w, d, h, layer)));
    }

    private async Task LoadAsync(string name, Func<GCodeDocument> load)
    {
        StopSimulation();
        IsLoading = true;
        LoadProgress = 0;
        FileInfo = Loc.F("S.Loading", name);
        try
        {
            var progress = new Progress<double>(p => LoadProgress = p * 100);
            var options = new ToolpathOptions();
            var (doc, tp, items) = await Task.Run(() =>
            {
                var d = load();
                var t = ToolpathBuilder.Build(d.Lines, options, default, progress);
                var lines = new GCodeLineItem[d.Lines.Count];
                for (int i = 0; i < lines.Length; i++)
                    lines[i] = new GCodeLineItem(i, d.Lines[i]);
                return (d, t, lines);
            });
            SetDocument(doc, tp, items);
            Log(LogKind.Info, Loc.F("S.Log.Opened", doc.Name, doc.Lines.Count, tp.Layers.Count));
        }
        catch (Exception ex)
        {
            FileInfo = Loc.T("S.LoadError");
            Notify(Loc.T("S.Notice.OpenFailed"), ex.Message, NotifySeverity.Error);
        }
        finally
        {
            IsLoading = false;
            RelayCommand.Refresh();
        }
    }

    private void SetDocument(GCodeDocument doc, Toolpath tp, IReadOnlyList<GCodeLineItem> items)
    {
        Document = doc;
        GCodeLines = items;
        Toolpath = tp;
        FileName = doc.Name;
        UploadName = MakeShortName(doc.Name);
        RebuildFileInfo();
        ViewerMode = ViewerMode.Preview;
        ShowAll();
        SimTime = 0;
        CurrentLine = -1;
    }

    private void RebuildFileInfo()
    {
        var doc = Document;
        var tp = Toolpath;
        if (doc == null || tp == null)
        {
            FileInfo = Loc.T("S.NoFile");
            return;
        }
        var size = tp.Max - tp.Min;
        double grams = tp.FilamentLength * Math.PI * _settings.FilamentDiameter * _settings.FilamentDiameter / 4 * 1.24 / 1000;
        FileInfo = Loc.F("S.FileInfo", doc.Lines.Count, tp.Layers.Count, size.X, size.Y, tp.Max.Z,
            FormatTime(tp.TotalTime), tp.FilamentLength / 1000, grams);
    }

    public static string FormatTime(double seconds)
    {
        if (double.IsNaN(seconds) || seconds < 0)
            seconds = 0;
        var t = TimeSpan.FromSeconds(seconds);
        return t.TotalHours >= 1 ? $"{(int)t.TotalHours}:{t.Minutes:00}:{t.Seconds:00}" : $"{t.Minutes}:{t.Seconds:00}";
    }

    /// <summary>Lines to send: comments and empty lines removed, source lines kept.</summary>
    private static List<JobLine> MakeJobLines(GCodeDocument doc)
    {
        var job = new List<JobLine>(doc.Lines.Count);
        for (int i = 0; i < doc.Lines.Count; i++)
        {
            string cmd = GCodeCommand.StripComment(doc.Lines[i]);
            if (cmd.Length > 0)
                job.Add(new JobLine(cmd, i));
        }
        return job;
    }

    // ---------------------------------------------------------------- printing

    private PrintSource _printSource;
    public PrintSource PrintSource
    {
        get => _printSource;
        private set
        {
            if (!Set(ref _printSource, value))
                return;
            OnPropertyChanged(nameof(IsPrinting));
            OnPropertyChanged(nameof(CanControl));
            OnPropertyChanged(nameof(CanLoadFile));
            OnPropertyChanged(nameof(PrintSourceText));
            RelayCommand.Refresh();
        }
    }

    public bool IsPrinting => PrintSource != PrintSource.None;

    public string PrintSourceText => PrintSource switch
    {
        PrintSource.Host => Loc.T("S.Source.Host"),
        PrintSource.Sd => Loc.T("S.Source.Sd"),
        _ => IsUploading ? Loc.T("S.Source.Upload") : Loc.T("S.Source.None"),
    };

    private bool _isPaused;
    public bool IsPaused { get => _isPaused; private set => Set(ref _isPaused, value); }

    private double _printProgress;
    /// <summary>0…100.</summary>
    public double PrintProgress { get => _printProgress; private set => Set(ref _printProgress, value); }

    private string _elapsedText = "";
    public string ElapsedText { get => _elapsedText; private set => Set(ref _elapsedText, value); }

    private string _remainingText = "";
    public string RemainingText { get => _remainingText; private set => Set(ref _remainingText, value); }

    private List<JobLine>? _jobLines;
    private double _printStart;
    private double _pauseStart;
    private double _pausedTotal;
    private float _liveEstimateDone;     // Estimated print time of the moves done, s.

    public RelayCommand StartPrintCommand { get; private set; } = null!;
    public RelayCommand PauseCommand { get; private set; } = null!;
    public RelayCommand ResumeCommand { get; private set; } = null!;
    public RelayCommand StopCommand { get; private set; } = null!;

    private void StartHostPrint()
    {
        var doc = Document;
        if (doc == null)
            return;
        var lines = MakeJobLines(doc);
        var longLine = lines.FirstOrDefault(l => l.Command.Length > 80);
        if (longLine.Command != null)
            Log(LogKind.Warning, Loc.F("S.Log.LongLine", longLine.SourceLine + 1));
        try
        {
            StopSimulation();
            _jobLines = lines;
            _conn.StartJob(lines);
            BeginLivePrint(PrintSource.Host);
            Log(LogKind.Info, Loc.F("S.Log.HostPrintStarted", doc.Name, lines.Count));
        }
        catch (Exception ex)
        {
            Notify(Loc.T("S.Notice.PrintNotStarted"), ex.Message, NotifySeverity.Error);
        }
    }

    private void BeginLivePrint(PrintSource source)
    {
        PrintSource = source;
        IsPaused = false;
        PrintProgress = 0;
        _printStart = _clock.Elapsed.TotalSeconds;
        _pausedTotal = 0;
        _liveEstimateDone = 0;
        ViewerMode = ViewerMode.Live;
        if (Toolpath != null && (_liveMap != null || source == PrintSource.Host))
        {
            SegmentLimit = 0;
            MaxLayer = 0;
            ShowNozzle = true;
        }
    }

    private void EndLivePrint(string message)
    {
        if (PrintSource == PrintSource.None)
            return;
        PrintSource = PrintSource.None;
        IsPaused = false;
        _jobLines = null;
        _liveMap = null;
        RemainingText = "";
        ViewerMode = ViewerMode.Preview;
        ShowAll();
        CurrentLine = -1;
        Log(LogKind.Info, message);
    }

    private void Pause()
    {
        if (PrintSource == PrintSource.Host)
            _conn.PauseJob();
        else if (PrintSource == PrintSource.Sd)
            _conn.SdPause();
        else
            return;
        IsPaused = true;
        _pauseStart = _clock.Elapsed.TotalSeconds;
        Log(LogKind.Info, Loc.T("S.Log.Pause"));
    }

    private void Resume()
    {
        if (PrintSource == PrintSource.Host)
            _conn.ResumeJob();
        else if (PrintSource == PrintSource.Sd)
            _conn.SdResume();
        else
            return;
        IsPaused = false;
        _pausedTotal += _clock.Elapsed.TotalSeconds - _pauseStart;
        Log(LogKind.Info, Loc.T("S.Log.Resume"));
    }

    private void Stop()
    {
        if (IsUploading)
        {
            if (MessageBox.Show(Loc.T("S.Ask.CancelUpload"), "Teacup Host", MessageBoxButton.YesNo,
                    MessageBoxImage.Question) == MessageBoxResult.Yes)
                _conn.CancelJob();
            return;
        }
        if (MessageBox.Show(Loc.T("S.Ask.StopPrint"),
                "Teacup Host", MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
            return;
        var source = PrintSource;
        if (source == PrintSource.Host)
            _conn.CancelJob();
        else if (source == PrintSource.Sd)
            _conn.SdStop();
        _conn.SendEmergency("M410");
        _conn.SendScript(_settings.CancelScript);
        EndLivePrint(Loc.T("S.Log.PrintStopped"));
    }

    private void OnJobCompleted(JobKind kind, bool cancelled)
    {
        if (kind == JobKind.Upload)
        {
            OnUploadCompleted(cancelled);
            return;
        }
        if (PrintSource != PrintSource.Host)
            return;
        double elapsed = _clock.Elapsed.TotalSeconds - _printStart - _pausedTotal;
        if (!cancelled)
        {
            PrintProgress = 100;
            Notify(Loc.T("S.Notice.PrintDone"), Loc.F("S.Notice.PrintDoneText", FileName, FormatTime(elapsed)),
                NotifySeverity.Success);
        }
        EndLivePrint(cancelled ? Loc.T("S.Log.PrintCancelled") : Loc.F("S.Log.PrintDone", FormatTime(elapsed)));
    }

    private void OnJobAck(int index)
    {
        if (IsUploading)
        {
            int total = Math.Max(1, _conn.JobLength);
            UploadProgress = (index + 1) * 100.0 / total;
            return;
        }
        if (PrintSource != PrintSource.Host || _jobLines == null || index >= _jobLines.Count)
            return;
        PrintProgress = (index + 1) * 100.0 / _jobLines.Count;
        ShowLiveLine(_jobLines[index].SourceLine);
    }

    private void UpdateElapsed()
    {
        if (PrintSource == PrintSource.None)
            return;
        double now = _clock.Elapsed.TotalSeconds;
        double paused = _pausedTotal + (IsPaused ? now - _pauseStart : 0);
        double elapsed = now - _printStart - paused;
        ElapsedText = FormatTime(elapsed);
        var tp = Toolpath;
        if (tp == null || tp.TotalTime <= 0 || _liveEstimateDone <= 0)
        {
            RemainingText = "";
            return;
        }
        double remaining = tp.TotalTime - _liveEstimateDone;
        // After a few minutes correct the estimate by how fast the printer really is.
        if (elapsed > 120 && _liveEstimateDone > 60)
            remaining *= Math.Clamp(elapsed / _liveEstimateDone, 0.5, 3);
        RemainingText = "≈ " + FormatTime(remaining);
    }

    // ---------------------------------------------------------------- SD card / SPI flash

    public ObservableCollection<SdFileItem> SdFiles { get; } = new();

    private SdFileItem? _selectedSdFile;
    public SdFileItem? SelectedSdFile { get => _selectedSdFile; set => Set(ref _selectedSdFile, value); }

    private string _sdStatus = Loc.T("S.SdNotListed");
    private bool _sdFilesReceived;
    public string SdStatus { get => _sdStatus; private set => Set(ref _sdStatus, value); }

    private string _sdPrintingFile = "";
    public string SdPrintingFile { get => _sdPrintingFile; private set => Set(ref _sdPrintingFile, value); }

    public RelayCommand SdRefreshCommand { get; private set; } = null!;
    public RelayCommand SdInitCommand { get; private set; } = null!;
    public RelayCommand SdPrintCommand { get; private set; } = null!;
    public RelayCommand SdDeleteCommand { get; private set; } = null!;
    public RelayCommand SdFormatCommand { get; private set; } = null!;

    /// <summary>Byte offsets of the uploaded files, to show the line printed from SD.</summary>
    private sealed record SdMap(GCodeDocument Doc, Toolpath Tp, IReadOnlyList<GCodeLineItem> Items,
        long[] Offsets, int[] DocLines);

    private readonly Dictionary<string, SdMap> _uploadMaps = new(StringComparer.OrdinalIgnoreCase);
    private SdMap? _liveMap;

    private void OnSdFiles(IReadOnlyList<SdFileInfo> files)
    {
        var selected = SelectedSdFile?.Name;
        SdFiles.Clear();
        foreach (var f in files)
            SdFiles.Add(new SdFileItem(f.Name, f.Size));
        SelectedSdFile = SdFiles.FirstOrDefault(f => f.Name == selected) ?? SdFiles.FirstOrDefault();
        _sdFilesReceived = true;
        SdStatus = files.Count == 0 ? Loc.T("S.SdNoFiles") : Loc.F("S.SdFileCount", files.Count);
    }

    private void OnSdMessage(string msg)
    {
        SdStatus = msg.StartsWith("echo:", StringComparison.Ordinal) ? msg[5..] : msg;
        if (msg.StartsWith("File deleted", StringComparison.OrdinalIgnoreCase) ||
            msg.Contains("files deleted", StringComparison.OrdinalIgnoreCase) ||
            msg.Contains("SD card ok", StringComparison.OrdinalIgnoreCase))
            _conn.SdRefresh();
        if (msg.Contains("open failed", StringComparison.OrdinalIgnoreCase) && PrintSource == PrintSource.Sd)
        {
            EndLivePrint(Loc.F("S.Log.SdOpenFailed", msg));
            Notify(Loc.T("S.Notice.SdPrint"), msg, NotifySeverity.Error);
        }
    }

    private void StartSdPrint()
    {
        var file = SelectedSdFile;
        if (file == null)
            return;
        StopSimulation();

        // What to show: the uploaded file, or the open file if it has the same size.
        _liveMap = null;
        if (_uploadMaps.TryGetValue(file.Name, out var map))
        {
            _liveMap = map;
            if (Document != map.Doc)
                SetDocument(map.Doc, map.Tp, map.Items);
        }
        else if (Document != null && Toolpath != null)
        {
            var doc = Document;
            if (Math.Abs(doc.SizeBytes - file.Size) <= 2)
            {
                _liveMap = new SdMap(doc, Toolpath, GCodeLines, doc.LineOffsets,
                    Enumerable.Range(0, doc.Lines.Count).ToArray());
                Log(LogKind.Info, Loc.F("S.Log.SdMatchedByOpenFile", file.Name, doc.Name));
            }
            else
            {
                Log(LogKind.Info, Loc.F("S.Log.SdNoMatch", file.Name));
            }
        }

        SdPrintingFile = file.Name;
        _conn.SdStartPrint(file.Name);
        BeginLivePrint(PrintSource.Sd);
        if (_liveMap == null)
            ShowNozzle = false;
        Log(LogKind.Info, Loc.F("S.Log.SdPrintStarted", file.Name));
    }

    private void OnSdProgress(long pos, long size)
    {
        if (PrintSource != PrintSource.Sd || size <= 0)
            return;
        PrintProgress = pos * 100.0 / size;
        SdStatus = Loc.F("S.SdProgress", SdPrintingFile, pos, size);
        var map = _liveMap;
        if (map == null || Toolpath != map.Tp)
            return;
        // The firmware reports the read position, which is the line about to execute.
        int idx = GCodeDocument.LineAtOffset(map.Offsets, Math.Max(0, pos - 1));
        if (idx >= 0 && idx < map.DocLines.Length)
            ShowLiveLine(map.DocLines[idx]);
    }

    private void OnSdPrintFinished()
    {
        if (PrintSource != PrintSource.Sd)
            return;
        double elapsed = _clock.Elapsed.TotalSeconds - _printStart - _pausedTotal;
        PrintProgress = 100;
        if (_liveMap != null && Toolpath != null)
            ShowAll();
        Notify(Loc.T("S.Notice.SdDone"), Loc.F("S.Notice.PrintDoneText", SdPrintingFile, FormatTime(elapsed)),
            NotifySeverity.Success);
        EndLivePrint(Loc.F("S.Log.SdDone", FormatTime(elapsed)));
    }

    // ---------------------------------------------------------------- upload

    private string _uploadName = "";
    public string UploadName
    {
        get => _uploadName;
        set => Set(ref _uploadName, value.Trim().ToUpperInvariant());
    }

    private bool _isUploading;
    public bool IsUploading
    {
        get => _isUploading;
        private set
        {
            if (!Set(ref _isUploading, value))
                return;
            OnPropertyChanged(nameof(CanControl));
            OnPropertyChanged(nameof(CanLoadFile));
            OnPropertyChanged(nameof(PrintSourceText));
            RelayCommand.Refresh();
        }
    }

    private double _uploadProgress;
    public double UploadProgress { get => _uploadProgress; private set => Set(ref _uploadProgress, value); }

    public RelayCommand UploadCommand { get; private set; } = null!;

    private static readonly Regex ShortName = new(@"^[A-Z0-9_\-~!#$%&'()@^`{}]{1,8}\.[A-Z0-9_]{1,3}$");

    /// <summary>8.3 name for the printer: "My part v2.gcode" → "MYPARTV2.GCO".</summary>
    public static string MakeShortName(string fileName)
    {
        string stem = Path.GetFileNameWithoutExtension(fileName).ToUpperInvariant();
        var chars = stem.Where(c => c is >= 'A' and <= 'Z' or >= '0' and <= '9' or '_' or '-').Take(8).ToArray();
        string s = chars.Length > 0 ? new string(chars) : "PRINT";
        return s + ".GCO";
    }

    private List<JobLine>? _uploadLines;
    private string _uploadingName = "";

    private void StartUpload()
    {
        var doc = Document;
        if (doc == null)
            return;
        string name = UploadName;
        if (!ShortName.IsMatch(name))
        {
            Notify(Loc.T("S.Notice.FileName"), Loc.T("S.Notice.FileNameText"), NotifySeverity.Warning);
            return;
        }
        if (SdFiles.Any(f => f.Name.Equals(name, StringComparison.OrdinalIgnoreCase)) &&
            MessageBox.Show(Loc.F("S.Ask.ReplaceFile", name), Loc.T("S.Ask.UploadTitle"),
                MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes)
            return;
        var lines = MakeJobLines(doc);
        try
        {
            var job = PrinterConnection.CreateUploadJob(name, lines);
            _uploadLines = job.Skip(1).Take(job.Count - 2).ToList();
            _uploadingName = name;
            UploadProgress = 0;
            IsUploading = true;
            _conn.StartJob(job, JobKind.Upload);
            Log(LogKind.Info, Loc.F("S.Log.UploadStarted", doc.Name, name, lines.Count));
        }
        catch (Exception ex)
        {
            IsUploading = false;
            Notify(Loc.T("S.Notice.UploadNotStarted"), ex.Message, NotifySeverity.Error);
        }
    }

    private void OnUploadCompleted(bool cancelled)
    {
        IsUploading = false;
        if (cancelled || _uploadLines == null || Document == null || Toolpath == null)
        {
            Log(LogKind.Warning, Loc.T("S.Log.UploadCancelled"));
            _uploadLines = null;
            return;
        }
        // The firmware stores each line as received plus "\n".
        var offsets = new long[_uploadLines.Count];
        var docLines = new int[_uploadLines.Count];
        long pos = 0;
        for (int i = 0; i < _uploadLines.Count; i++)
        {
            offsets[i] = pos;
            docLines[i] = _uploadLines[i].SourceLine;
            pos += PrinterConnection.ToAscii(_uploadLines[i].Command).Length + 1;
        }
        _uploadMaps[_uploadingName] = new SdMap(Document, Toolpath, GCodeLines, offsets, docLines);
        _uploadLines = null;
        UploadProgress = 100;
        Notify(Loc.T("S.Notice.Uploaded"), Loc.F("S.Notice.UploadedText", _uploadingName, pos), NotifySeverity.Success);
        Log(LogKind.Info, Loc.F("S.Log.Uploaded", _uploadingName, pos));
        _conn.SdRefresh();
    }

    private void CreateJobCommands()
    {
        OpenFileCommand = new RelayCommand(OpenFile, () => CanLoadFile);
        OpenDemoCommand = new RelayCommand(() => _ = LoadDemoAsync(), () => CanLoadFile);

        StartPrintCommand = new RelayCommand(StartHostPrint,
            () => IsOnline && Document != null && PrintSource == PrintSource.None && !IsUploading && !IsLoading);
        PauseCommand = new RelayCommand(Pause, () => IsPrinting && !IsPaused && IsOnline);
        ResumeCommand = new RelayCommand(Resume, () => IsPrinting && IsPaused && IsOnline);
        StopCommand = new RelayCommand(Stop, () => (IsPrinting || IsUploading) && IsConnected);

        SdRefreshCommand = new RelayCommand(() => _conn.SdRefresh(), () => IsOnline);
        SdInitCommand = new RelayCommand(() => _conn.SdInit(), () => IsOnline && !IsPrinting);
        SdPrintCommand = new RelayCommand(StartSdPrint,
            () => IsOnline && SelectedSdFile != null && PrintSource == PrintSource.None && !IsUploading);
        SdDeleteCommand = new RelayCommand(() =>
        {
            var f = SelectedSdFile;
            if (f != null && MessageBox.Show(Loc.F("S.Ask.DeleteFile", f.Name), Loc.T("S.Ask.SdTitle"),
                    MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
            {
                _conn.SdDelete(f.Name);
                _uploadMaps.Remove(f.Name);
            }
        }, () => IsOnline && SelectedSdFile != null && !IsPrinting && !IsUploading);
        SdFormatCommand = new RelayCommand(() =>
        {
            if (MessageBox.Show(Loc.T("S.Ask.FormatFlash"),
                    "SPI flash", MessageBoxButton.YesNo, MessageBoxImage.Warning) == MessageBoxResult.Yes)
            {
                _conn.Send("M9002");
                _uploadMaps.Clear();
            }
        }, () => IsOnline && !IsPrinting && !IsUploading);
        UploadCommand = new RelayCommand(StartUpload,
            () => IsOnline && Document != null && PrintSource == PrintSource.None && !IsUploading && !IsLoading);
    }
}

public sealed class LegendItem
{
    public LegendItem(FeatureType type)
    {
        Type = type;
    }

    public FeatureType Type { get; }
    public string Name => Loc.T("S.Feature." + Type);
    public System.Windows.Media.Brush Brush => Controls.FeaturePalette.BrushOf(Type);
}
