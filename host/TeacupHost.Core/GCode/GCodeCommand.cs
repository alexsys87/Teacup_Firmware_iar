using System.Globalization;

namespace TeacupHost.Core.GCode;

/// <summary>
/// One parsed G-code line: command letter and number (G1, M104, T0),
/// parameters and the comment. Parsing is tolerant, like the firmware:
/// unknown letters are kept as parameters, text in quotes is skipped.
/// </summary>
public sealed class GCodeCommand
{
    private readonly double[] _values = new double[26];
    private uint _seen;

    /// <summary>Command letter ('G', 'M', 'T') or '\0' if the line has none.</summary>
    public char Letter { get; private set; }

    /// <summary>Command number, e.g. 1 for G1, 862.3 stays 862 with <see cref="SubCode"/> 3.</summary>
    public int Code { get; private set; }

    /// <summary>Sub-code after a dot (M862.3 → 3), -1 if none.</summary>
    public int SubCode { get; private set; } = -1;

    /// <summary>Comment text after ';' without the ';', or null.</summary>
    public string? Comment { get; private set; }

    /// <summary>Raw text after the command for M117/M28/M23 style commands.</summary>
    public string Argument { get; private set; } = "";

    public bool IsEmpty => Letter == '\0';

    public bool Has(char letter) => (_seen & Bit(letter)) != 0;

    public double Get(char letter, double fallback = 0) =>
        Has(letter) ? _values[char.ToUpperInvariant(letter) - 'A'] : fallback;

    public bool TryGet(char letter, out double value)
    {
        if (Has(letter))
        {
            value = _values[char.ToUpperInvariant(letter) - 'A'];
            return true;
        }
        value = 0;
        return false;
    }

    public bool Is(char letter, int code) => Letter == letter && Code == code;

    private static uint Bit(char letter)
    {
        int i = char.ToUpperInvariant(letter) - 'A';
        return i is >= 0 and < 26 ? 1u << i : 0;
    }

    /// <summary>Remove the comment and surrounding spaces from a line.</summary>
    public static string StripComment(string line)
    {
        int semi = line.IndexOf(';');
        var s = semi >= 0 ? line.AsSpan(0, semi) : line.AsSpan();
        return s.Trim().ToString();
    }

    /// <summary>Parse a line. Line number and checksum (N12 ... *34) are dropped.</summary>
    public static GCodeCommand Parse(string line)
    {
        var cmd = new GCodeCommand();
        int semi = line.IndexOf(';');
        if (semi >= 0)
        {
            cmd.Comment = line[(semi + 1)..].Trim();
            line = line[..semi];
        }
        int star = line.IndexOf('*');
        if (star >= 0)
            line = line[..star];

        int i = 0;
        int n = line.Length;
        bool first = true;
        while (i < n)
        {
            char c = line[i];
            if (char.IsWhiteSpace(c)) { i++; continue; }
            if (c == '"')
            {
                // Quoted text (M862.3 P "MK3S") isn't G-code.
                int end = line.IndexOf('"', i + 1);
                i = end < 0 ? n : end + 1;
                continue;
            }
            if (c == '(')
            {
                int end = line.IndexOf(')', i + 1);
                i = end < 0 ? n : end + 1;
                continue;
            }
            char letter = char.ToUpperInvariant(c);
            if (letter < 'A' || letter > 'Z') { i++; continue; }
            i++;
            int start = i;
            while (i < n && (char.IsDigit(line[i]) || line[i] is '.' or '-' or '+'))
                i++;
            var number = line.AsSpan(start, i - start);

            if (first && letter == 'N' && cmd.Letter == '\0')
            {
                // Line number, not a parameter.
                continue;
            }
            if (cmd.Letter == '\0' && letter is 'G' or 'M' or 'T')
            {
                int dot = number.IndexOf('.');
                var main = dot >= 0 ? number[..dot] : number;
                int.TryParse(main, NumberStyles.Integer, CultureInfo.InvariantCulture, out int code);
                cmd.Letter = letter;
                cmd.Code = code;
                if (dot >= 0 && int.TryParse(number[(dot + 1)..], NumberStyles.Integer,
                        CultureInfo.InvariantCulture, out int sub))
                    cmd.SubCode = sub;
                first = false;
                cmd.Argument = line[i..].Trim();
                // Commands with free text: nothing else to parse.
                if (letter == 'M' && code is 117 or 118 or 23 or 28 or 30 or 32)
                    break;
                continue;
            }
            first = false;
            if (double.TryParse(number, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
            {
                cmd._values[letter - 'A'] = v;
                cmd._seen |= Bit(letter);
            }
            else if (number.Length == 0)
            {
                // Bare letter (G28 X): seen, value 0.
                cmd._values[letter - 'A'] = 0;
                cmd._seen |= Bit(letter);
            }
        }
        return cmd;
    }

    public override string ToString() =>
        Letter == '\0' ? "" : SubCode >= 0 ? $"{Letter}{Code}.{SubCode}" : $"{Letter}{Code}";
}
