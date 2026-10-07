using TeacupHost.Infrastructure;

namespace TeacupHost.ViewModels;

public enum LogKind
{
    Sent,
    Received,
    Info,
    Warning,
    Error,
}

public sealed class LogEntry
{
    public LogEntry(LogKind kind, string text)
    {
        Kind = kind;
        Text = text;
        Time = DateTime.Now.ToString("HH:mm:ss");
    }

    public LogKind Kind { get; }
    public string Text { get; }
    public string Time { get; }
    public string Prefix => Kind switch
    {
        LogKind.Sent => ">>",
        LogKind.Received => "<<",
        LogKind.Error => "!!",
        LogKind.Warning => "!",
        _ => "--",
    };
}

/// <summary>A line of the G-code listing.</summary>
public sealed class GCodeLineItem
{
    public GCodeLineItem(int index, string text)
    {
        Index = index;
        Text = text;
    }

    public int Index { get; }
    public int Number => Index + 1;
    public string Text { get; }
    public bool IsComment => Text.TrimStart().StartsWith(';');
}

public sealed class SdFileItem
{
    public SdFileItem(string name, long size)
    {
        Name = name;
        Size = size;
    }

    public string Name { get; }
    public long Size { get; }
    public string SizeText => Size switch
    {
        < 1024 => Services.Loc.F("S.SizeB", Size),
        < 1024 * 1024 => Services.Loc.F("S.SizeKB", Size / 1024.0),
        _ => Services.Loc.F("S.SizeMB", Size / 1024.0 / 1024.0),
    };
}

public sealed class MacroItem : ObservableObject
{
    private string _name;
    private string _script;

    public MacroItem(string name, string script)
    {
        _name = name;
        _script = script;
    }

    public string Name
    {
        get => _name;
        set => Set(ref _name, value);
    }

    public string Script
    {
        get => _script;
        set => Set(ref _script, value);
    }
}

public enum ViewerMode
{
    /// <summary>The whole file, the layer slider picks the top layer.</summary>
    Preview,
    /// <summary>Simulated print, time runs at the chosen speed.</summary>
    Simulation,
    /// <summary>Follows the real print.</summary>
    Live,
}

public enum PrintSource
{
    None,
    Host,
    Sd,
}
