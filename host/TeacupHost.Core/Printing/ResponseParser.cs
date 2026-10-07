using System.Globalization;
using System.Text.RegularExpressions;

namespace TeacupHost.Core.Printing;

public readonly record struct TemperatureReading(
    double Hotend, double HotendTarget, double Bed, double BedTarget,
    int HotendPower, int BedPower, bool HasBed);

public readonly record struct PrinterPosition(double X, double Y, double Z, double E);

public sealed record SdFileInfo(string Name, long Size);

/// <summary>Parsers for the Marlin-style messages Teacup sends.</summary>
public static partial class ResponseParser
{
    private static readonly CultureInfo Ci = CultureInfo.InvariantCulture;

    [GeneratedRegex(@"(?<![\w@\]])T:\s*(-?[\d.]+)(?:\s*/\s*(-?[\d.]+))?")]
    private static partial Regex HotendRegex();

    [GeneratedRegex(@"(?<![\w@])B:\s*(-?[\d.]+)(?:\s*/\s*(-?[\d.]+))?")]
    private static partial Regex BedRegex();

    [GeneratedRegex(@"(?<![\w])@:\s*(\d+)")]
    private static partial Regex HotendPowerRegex();

    [GeneratedRegex(@"B@:\s*(\d+)")]
    private static partial Regex BedPowerRegex();

    [GeneratedRegex(@"^X:\s*(-?[\d.]+)\s+Y:\s*(-?[\d.]+)\s+Z:\s*(-?[\d.]+)(?:\s+E:\s*(-?[\d.]+))?")]
    private static partial Regex PositionRegex();

    [GeneratedRegex(@"^SD printing byte (\d+)\s*/\s*(\d+)")]
    private static partial Regex SdProgressRegex();

    [GeneratedRegex(@"^(?:Resend|rs)[:\s]\s*N?(\d+)", RegexOptions.IgnoreCase)]
    private static partial Regex ResendRegex();

    [GeneratedRegex(@"^(\S+)\s+(\d+)$")]
    private static partial Regex FileEntryRegex();

    [GeneratedRegex(@"^File opened:\s*(\S+)\s+Size:\s*(\d+)")]
    private static partial Regex FileOpenedRegex();

    public static bool IsOk(string line) =>
        line.StartsWith("ok", StringComparison.Ordinal) &&
        (line.Length == 2 || line[2] == ' ' || line[2] == '\t');

    public static bool TryParseTemperature(string line, out TemperatureReading reading)
    {
        reading = default;
        var t = HotendRegex().Match(line);
        if (!t.Success)
            return false;
        var b = BedRegex().Match(line);
        var tp = HotendPowerRegex().Match(line);
        var bp = BedPowerRegex().Match(line);
        reading = new TemperatureReading(
            D(t.Groups[1].Value), t.Groups[2].Success ? D(t.Groups[2].Value) : 0,
            b.Success ? D(b.Groups[1].Value) : 0, b.Success && b.Groups[2].Success ? D(b.Groups[2].Value) : 0,
            tp.Success ? int.Parse(tp.Groups[1].Value, Ci) : 0,
            bp.Success ? int.Parse(bp.Groups[1].Value, Ci) : 0,
            b.Success);
        return true;
    }

    public static bool TryParsePosition(string line, out PrinterPosition pos)
    {
        pos = default;
        var m = PositionRegex().Match(line);
        if (!m.Success)
            return false;
        pos = new PrinterPosition(D(m.Groups[1].Value), D(m.Groups[2].Value), D(m.Groups[3].Value),
            m.Groups[4].Success ? D(m.Groups[4].Value) : 0);
        return true;
    }

    public static bool TryParseSdProgress(string line, out long position, out long size)
    {
        position = size = 0;
        var m = SdProgressRegex().Match(line);
        if (!m.Success)
            return false;
        position = long.Parse(m.Groups[1].Value, Ci);
        size = long.Parse(m.Groups[2].Value, Ci);
        return true;
    }

    public static bool TryParseResend(string line, out int lineNumber)
    {
        lineNumber = 0;
        var m = ResendRegex().Match(line);
        if (!m.Success)
            return false;
        lineNumber = int.Parse(m.Groups[1].Value, Ci);
        return true;
    }

    public static bool TryParseFileEntry(string line, out SdFileInfo file)
    {
        file = null!;
        var m = FileEntryRegex().Match(line.Trim());
        if (!m.Success)
            return false;
        file = new SdFileInfo(m.Groups[1].Value, long.Parse(m.Groups[2].Value, Ci));
        return true;
    }

    public static bool TryParseFileOpened(string line, out string name, out long size)
    {
        name = "";
        size = 0;
        var m = FileOpenedRegex().Match(line);
        if (!m.Success)
            return false;
        name = m.Groups[1].Value;
        size = long.Parse(m.Groups[2].Value, Ci);
        return true;
    }

    /// <summary>Messages after which the printer is stopped (printer_kill() and friends).</summary>
    public static bool IsFatalError(string line) =>
        line.StartsWith("Error:", StringComparison.OrdinalIgnoreCase) &&
        (line.Contains("halted", StringComparison.OrdinalIgnoreCase) ||
         line.Contains("kill", StringComparison.OrdinalIgnoreCase) ||
         line.Contains("stopped", StringComparison.OrdinalIgnoreCase));

    /// <summary>XOR checksum of a line, like printcore and the firmware compute it.</summary>
    public static int Checksum(string s)
    {
        int cs = 0;
        foreach (char c in s)
            cs ^= c & 0xFF;
        return cs;
    }

    /// <summary>"N12 G1 X10*85" for line 12.</summary>
    public static string FormatNumbered(int lineNumber, string command)
    {
        string body = "N" + lineNumber.ToString(Ci) + " " + command;
        return body + "*" + Checksum(body).ToString(Ci);
    }

    private static double D(string s) =>
        double.TryParse(s, NumberStyles.Float, Ci, out double v) ? v : 0;
}
