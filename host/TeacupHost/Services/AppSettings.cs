using System.IO;
using System.Text.Json;

namespace TeacupHost.Services;

public sealed class MacroSettings
{
    public string Name { get; set; } = "";
    public string Script { get; set; } = "";
}

/// <summary>User settings, stored as JSON in %APPDATA%\TeacupHost\settings.json.</summary>
public sealed class AppSettings
{
    public string? Port { get; set; }
    public int BaudRate { get; set; } = 115200;
    public double VirtualTimeScale { get; set; } = 10;

    // Printer geometry, printer.p3steel.h: X_MAX, Y_MAX, Z_MAX.
    public double BedWidth { get; set; } = 220;
    public double BedDepth { get; set; } = 180;
    public double BedHeight { get; set; } = 200;
    public double FilamentDiameter { get; set; } = 1.75;

    public double JogFeedXY { get; set; } = 3000;
    public double JogFeedZ { get; set; } = 240;
    public double ExtrudeLength { get; set; } = 5;
    public double ExtrudeFeed { get; set; } = 120;

    public double HotendSetpoint { get; set; } = 205;
    public double BedSetpoint { get; set; } = 60;
    public double[] HotendPresets { get; set; } = { 180, 205, 215, 230, 240, 250 };
    public double[] BedPresets { get; set; } = { 50, 60, 70, 80, 90, 100 };

    public string CancelScript { get; set; } = "M104 S0\nM140 S0\nM107\nM84";
    public bool ShowTravel { get; set; }
    public bool HideTemperatureLines { get; set; } = true;
    public bool ShowJobLines { get; set; }
    public string? LastFolder { get; set; }

    public List<MacroSettings> Macros { get; set; } = new()
    {
        new() { Name = "Прогрев PLA", Script = "M104 S205\nM140 S60" },
        new() { Name = "Прогрев PETG", Script = "M104 S240\nM140 S80" },
        new() { Name = "Остудить", Script = "M104 S0\nM140 S0" },
        new() { Name = "Парковка", Script = "G91\nG1 Z10 F240\nG90\nG1 X10 Y170 F6000" },
        new() { Name = "Сетка стола", Script = "G28\nG29" },
        new() { Name = "Смена филамента", Script = "M600" },
        new() { Name = "Статистика", Script = "M78" },
    };

    private static string FilePath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "TeacupHost", "settings.json");

    private static readonly JsonSerializerOptions Options = new() { WriteIndented = true };

    public static AppSettings Load()
    {
        try
        {
            if (File.Exists(FilePath))
                return JsonSerializer.Deserialize<AppSettings>(File.ReadAllText(FilePath), Options) ?? new AppSettings();
        }
        catch
        {
            // Broken file: start with the defaults.
        }
        return new AppSettings();
    }

    public void Save()
    {
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);
            File.WriteAllText(FilePath, JsonSerializer.Serialize(this, Options));
        }
        catch
        {
            // Settings are a convenience, never fail because of them.
        }
    }
}
