using System.Numerics;

namespace TeacupHost.Core.GCode;

public enum MoveKind : byte
{
    Travel,
    Extrude,
}

/// <summary>What an extrusion prints, from slicer ";TYPE:" comments.</summary>
public enum FeatureType : byte
{
    Extrude,        // No type information.
    OuterWall,
    InnerWall,
    Infill,
    SolidInfill,
    TopSolid,
    Bridge,
    GapFill,
    Support,
    Skirt,
    Custom,
    Travel,         // Pseudo type for travel moves (palette index).
}

/// <summary>One straight move. Arcs are split into several segments.</summary>
public struct ToolpathSegment
{
    public Vector3 Start;
    public Vector3 End;
    /// <summary>Time from the start of the file when this move begins, s.</summary>
    public float StartTime;
    public float Duration;
    /// <summary>Filament length of this move, mm (0 for travel).</summary>
    public float Extrusion;
    /// <summary>Feedrate, mm/s.</summary>
    public float Speed;
    /// <summary>Zero-based line of the document.</summary>
    public int Line;
    public int Layer;
    public MoveKind Kind;
    public FeatureType Feature;

    public readonly float EndTime => StartTime + Duration;
    public readonly float Length => Vector3.Distance(Start, End);

    public readonly Vector3 PositionAt(float time)
    {
        if (Duration <= 0)
            return End;
        float f = Math.Clamp((time - StartTime) / Duration, 0f, 1f);
        return Vector3.Lerp(Start, End, f);
    }
}

public readonly record struct ToolpathLayer(int Index, float Z, float Height, int FirstSegment, int SegmentCount)
{
    public int EndSegment => FirstSegment + SegmentCount;
}

/// <summary>Toolpath of a whole file: segments in file order, grouped into layers.</summary>
public sealed class Toolpath
{
    public ToolpathSegment[] Segments { get; }
    public IReadOnlyList<ToolpathLayer> Layers { get; }
    public float TotalTime { get; }
    public float FilamentLength { get; }
    public Vector3 Min { get; }
    public Vector3 Max { get; }
    public int LineCount { get; }

    public static Toolpath Empty { get; } = new([], [], 0, 0, Vector3.Zero, Vector3.Zero, 0);

    public Toolpath(ToolpathSegment[] segments, IReadOnlyList<ToolpathLayer> layers, float totalTime,
        float filament, Vector3 min, Vector3 max, int lineCount)
    {
        Segments = segments;
        Layers = layers;
        TotalTime = totalTime;
        FilamentLength = filament;
        Min = min;
        Max = max;
        LineCount = lineCount;
    }

    /// <summary>Index of the segment running at the given time (-1 before the first).</summary>
    public int SegmentAtTime(float time)
    {
        var s = Segments;
        if (s.Length == 0 || time < s[0].StartTime)
            return -1;
        int lo = 0, hi = s.Length - 1;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) >> 1;
            if (s[mid].StartTime <= time)
                lo = mid;
            else
                hi = mid - 1;
        }
        return lo;
    }

    /// <summary>
    /// Index of the last segment that belongs to a line at or before the given
    /// line (-1 if none). Lines without moves map to the move before them.
    /// </summary>
    public int LastSegmentAtOrBeforeLine(int line)
    {
        var s = Segments;
        if (s.Length == 0 || s[0].Line > line)
            return -1;
        int lo = 0, hi = s.Length - 1;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) >> 1;
            if (s[mid].Line <= line)
                lo = mid;
            else
                hi = mid - 1;
        }
        return lo;
    }

    /// <summary>First segment of a line or after it, Segments.Length if none.</summary>
    public int FirstSegmentAtOrAfterLine(int line)
    {
        int i = LastSegmentAtOrBeforeLine(line - 1);
        return i + 1;
    }

    public int LayerOfSegment(int segment)
    {
        if (segment < 0 || Segments.Length == 0)
            return 0;
        return Segments[Math.Min(segment, Segments.Length - 1)].Layer;
    }
}
