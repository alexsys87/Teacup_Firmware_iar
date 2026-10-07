using System.Text;

namespace TeacupHost.Core.GCode;

/// <summary>
/// A G-code file in memory: its lines and the byte offset of each line in
/// the file. The offsets map "SD printing byte n/m" back to a line.
/// </summary>
public sealed class GCodeDocument
{
    public string Name { get; }
    public string? Path { get; }
    public IReadOnlyList<string> Lines { get; }

    /// <summary>Byte offset of the start of each line in the file.</summary>
    public long[] LineOffsets { get; }

    public long SizeBytes { get; }

    private GCodeDocument(string name, string? path, string[] lines, long[] offsets, long size)
    {
        Name = name;
        Path = path;
        Lines = lines;
        LineOffsets = offsets;
        SizeBytes = size;
    }

    public static GCodeDocument Load(string path)
    {
        var bytes = File.ReadAllBytes(path);
        return FromBytes(System.IO.Path.GetFileName(path), path, bytes);
    }

    public static GCodeDocument FromText(string name, string text) =>
        FromBytes(name, null, Encoding.UTF8.GetBytes(text));

    public static GCodeDocument FromBytes(string name, string? path, byte[] bytes)
    {
        var lines = new List<string>(Math.Max(16, bytes.Length / 24));
        var offsets = new List<long>(lines.Capacity);
        int start = 0;
        for (int i = 0; i <= bytes.Length; i++)
        {
            if (i == bytes.Length || bytes[i] == (byte)'\n')
            {
                if (i == bytes.Length && start == i)
                    break;                       // No empty line after the last EOL.
                int end = i;
                if (end > start && bytes[end - 1] == (byte)'\r')
                    end--;
                offsets.Add(start);
                lines.Add(Encoding.UTF8.GetString(bytes, start, end - start));
                start = i + 1;
            }
        }
        return new GCodeDocument(name, path, lines.ToArray(), offsets.ToArray(), bytes.Length);
    }

    /// <summary>Line index that contains the given byte offset.</summary>
    public int LineAtOffset(long offset) => LineAtOffset(LineOffsets, offset);

    public static int LineAtOffset(long[] offsets, long offset)
    {
        if (offsets.Length == 0)
            return -1;
        int idx = Array.BinarySearch(offsets, offset);
        if (idx < 0)
            idx = ~idx - 1;
        return Math.Clamp(idx, 0, offsets.Length - 1);
    }
}
