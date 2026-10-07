using System.Collections.ObjectModel;
using System.IO;
using System.Text.RegularExpressions;
using System.Windows;
using Microsoft.Win32;
using TeacupHost.Core.GCode;
using TeacupHost.Core.Printing;
using TeacupHost.Infrastructure;

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

    private string _fileInfo = "Файл не открыт";
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
            Title = "Открыть G-код",
            Filter = "G-код (*.gcode;*.gco;*.g;*.gc;*.nc)|*.gcode;*.gco;*.g;*.gc;*.nc|Все файлы (*.*)|*.*",
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

    private Task LoadDemoAsync() =>
        LoadAsync("demo.gcode", () => GCodeDocument.FromText("demo.gcode", DemoGCode.Generate()));

    private async Task LoadAsync(string name, Func<GCodeDocument> load)
    {
        StopSimulation();
        IsLoading = true;
        LoadProgress = 0;
        FileInfo = $"Загрузка {name}…";
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
            Log(LogKind.Info, $"Открыт {doc.Name}: {doc.Lines.Count} строк, {tp.Layers.Count} слоёв");
        }
        catch (Exception ex)
        {
            FileInfo = "Ошибка загрузки";
            Notify("Не удалось открыть файл", ex.Message, NotifySeverity.Error);
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
        var size = tp.Max - tp.Min;
        double grams = tp.FilamentLength * Math.PI * _settings.FilamentDiameter * _settings.FilamentDiameter / 4 * 1.24 / 1000;
        FileInfo = $"{doc.Lines.Count:N0} строк · {tp.Layers.Count} слоёв · " +
                   $"{size.X:0.#}×{size.Y:0.#}×{tp.Max.Z:0.##} мм\n" +
                   $"Время ≈ {FormatTime(tp.TotalTime)} · филамент {tp.FilamentLength / 1000:0.##} м (≈{grams:0} г PLA)";
        ViewerMode = ViewerMode.Preview;
        ShowAll();
        SimTime = 0;
        CurrentLine = -1;
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
        PrintSource.Host => "Печать с компьютера",
        PrintSource.Sd => "Печать с SD / флеш",
        _ => IsUploading ? "Загрузка на SD / флеш" : "Нет задания",
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
            Log(LogKind.Warning, $"Строка {longLine.SourceLine + 1} длиннее 80 символов, прошивка может её отбросить");
        try
        {
            StopSimulation();
            _jobLines = lines;
            _conn.StartJob(lines);
            BeginLivePrint(PrintSource.Host);
            Log(LogKind.Info, $"Печать {doc.Name} с компьютера: {lines.Count} команд");
        }
        catch (Exception ex)
        {
            Notify("Печать не запущена", ex.Message, NotifySeverity.Error);
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
        Log(LogKind.Info, "Пауза");
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
        Log(LogKind.Info, "Продолжение");
    }

    private void Stop()
    {
        if (IsUploading)
        {
            if (MessageBox.Show("Прервать загрузку файла?", "Teacup Host", MessageBoxButton.YesNo,
                    MessageBoxImage.Question) == MessageBoxResult.Yes)
                _conn.CancelJob();
            return;
        }
        if (MessageBox.Show("Остановить печать?\nДвижение прервётся сразу (M410), нагреватели выключатся.",
                "Teacup Host", MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
            return;
        var source = PrintSource;
        if (source == PrintSource.Host)
            _conn.CancelJob();
        else if (source == PrintSource.Sd)
            _conn.SdStop();
        _conn.SendEmergency("M410");
        _conn.SendScript(_settings.CancelScript);
        EndLivePrint("Печать остановлена");
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
            Notify("Печать завершена", $"{FileName} за {FormatTime(elapsed)}", NotifySeverity.Success);
        }
        EndLivePrint(cancelled ? "Печать прервана" : $"Печать завершена за {FormatTime(elapsed)}");
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

    private string _sdStatus = "Список не получен";
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
        SdStatus = files.Count == 0 ? "Файлов нет" : $"Файлов: {files.Count}";
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
            EndLivePrint("Файл не открылся: " + msg);
            Notify("Печать с SD", msg, NotifySeverity.Error);
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
                Log(LogKind.Info, $"{file.Name}: показываю по открытому файлу {doc.Name} (размер совпадает)");
            }
            else
            {
                Log(LogKind.Info, $"{file.Name}: открытый файл другого размера, прогресс без отображения строк");
            }
        }

        SdPrintingFile = file.Name;
        _conn.SdStartPrint(file.Name);
        BeginLivePrint(PrintSource.Sd);
        if (_liveMap == null)
            ShowNozzle = false;
        Log(LogKind.Info, $"Печать с SD / флеш: {file.Name}");
    }

    private void OnSdProgress(long pos, long size)
    {
        if (PrintSource != PrintSource.Sd || size <= 0)
            return;
        PrintProgress = pos * 100.0 / size;
        SdStatus = $"{SdPrintingFile}: {pos:N0} / {size:N0} байт";
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
        Notify("Печать с SD завершена", $"{SdPrintingFile} за {FormatTime(elapsed)}", NotifySeverity.Success);
        EndLivePrint($"Печать с SD завершена за {FormatTime(elapsed)}");
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
            Notify("Имя файла", "Нужно имя в формате 8.3 латиницей, например PART.GCO", NotifySeverity.Warning);
            return;
        }
        if (SdFiles.Any(f => f.Name.Equals(name, StringComparison.OrdinalIgnoreCase)) &&
            MessageBox.Show($"Файл {name} уже есть на принтере. Заменить?", "Загрузка",
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
            Log(LogKind.Info, $"Загрузка {doc.Name} → {name}: {lines.Count} строк");
        }
        catch (Exception ex)
        {
            IsUploading = false;
            Notify("Загрузка не запущена", ex.Message, NotifySeverity.Error);
        }
    }

    private void OnUploadCompleted(bool cancelled)
    {
        IsUploading = false;
        if (cancelled || _uploadLines == null || Document == null || Toolpath == null)
        {
            Log(LogKind.Warning, "Загрузка прервана");
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
        Notify("Файл загружен", $"{_uploadingName}: {pos:N0} байт", NotifySeverity.Success);
        Log(LogKind.Info, $"Файл {_uploadingName} загружен, {pos:N0} байт");
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
            if (f != null && MessageBox.Show($"Удалить {f.Name} с принтера?", "SD / флеш",
                    MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
            {
                _conn.SdDelete(f.Name);
                _uploadMaps.Remove(f.Name);
            }
        }, () => IsOnline && SelectedSdFile != null && !IsPrinting && !IsUploading);
        SdFormatCommand = new RelayCommand(() =>
        {
            if (MessageBox.Show("Удалить ВСЕ файлы из SPI flash (M9002)?\nНа SD-карте команда не работает.",
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
    public string Name => Controls.FeaturePalette.NameOf(Type);
    public System.Windows.Media.Brush Brush => Controls.FeaturePalette.BrushOf(Type);
}
