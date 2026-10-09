namespace TeacupHost.Core.Input;

/// <summary>Axis directions held down by the keyboard, a stick or a D-pad.</summary>
[Flags]
public enum JogDirections
{
    None = 0,
    XPlus = 1,
    XMinus = 2,
    YPlus = 4,
    YMinus = 8,
    ZPlus = 16,
    ZMinus = 32,
}

public static class JogDirectionsExtensions
{
    private const JogDirections XAxis = JogDirections.XPlus | JogDirections.XMinus;
    private const JogDirections YAxis = JogDirections.YPlus | JogDirections.YMinus;
    private const JogDirections ZAxis = JogDirections.ZPlus | JogDirections.ZMinus;

    /// <summary>
    /// Opposite directions of one axis cancel each other. Z wins over X/Y:
    /// the two have different feed rates, so they are never mixed in one move.
    /// </summary>
    public static JogDirections Normalize(this JogDirections d)
    {
        if ((d & XAxis) == XAxis)
            d &= ~XAxis;
        if ((d & YAxis) == YAxis)
            d &= ~YAxis;
        if ((d & ZAxis) == ZAxis)
            d &= ~ZAxis;
        if ((d & ZAxis) != 0)
            d &= ZAxis;
        return d;
    }

    /// <summary>-1, 0 or +1 for the given axis letter (X, Y or Z) of a normalized value.</summary>
    public static int Sign(this JogDirections d, char axis) => char.ToUpperInvariant(axis) switch
    {
        'X' => (d.HasFlag(JogDirections.XPlus) ? 1 : 0) - (d.HasFlag(JogDirections.XMinus) ? 1 : 0),
        'Y' => (d.HasFlag(JogDirections.YPlus) ? 1 : 0) - (d.HasFlag(JogDirections.YMinus) ? 1 : 0),
        'Z' => (d.HasFlag(JogDirections.ZPlus) ? 1 : 0) - (d.HasFlag(JogDirections.ZMinus) ? 1 : 0),
        _ => 0,
    };
}

/// <summary>Step and feed rates of manual moves, as chosen in the Axes tab.</summary>
public readonly record struct JogSettings(double Step, double FeedXY, double FeedZ);

/// <summary>One relative move to send: distances in mm per axis and the feed rate in mm/min.</summary>
public readonly record struct JogMove(double X, double Y, double Z, double Feed, bool IsChunk)
{
    /// <summary>Length of the path of the tool.</summary>
    public double Distance => Math.Sqrt(X * X + Y * Y + Z * Z);

    /// <summary>How long the move takes at the full feed rate, ignoring acceleration.</summary>
    public double DurationMs => Feed > 0 ? Distance / Feed * 60000.0 : 0;
}

/// <summary>Result of one <see cref="JogRepeater.Update"/> call.</summary>
/// <param name="Move">A move to send now, if any.</param>
/// <param name="Released">A held key or stick that was moving in chunks was let go.</param>
public readonly record struct JogTick(JogMove? Move, bool Released);

/// <summary>Picks the next or previous value of the step list.</summary>
public static class JogStepPicker
{
    /// <summary>The step after (+1) or before (-1) the current one; the nearest list item is used if it is not on the list.</summary>
    public static double Next(IReadOnlyList<double> steps, double current, int delta)
    {
        if (steps.Count == 0)
            return current;
        int best = 0;
        for (int i = 1; i < steps.Count; i++)
        {
            if (Math.Abs(steps[i] - current) < Math.Abs(steps[best] - current))
                best = i;
        }
        return steps[Math.Clamp(best + Math.Sign(delta), 0, steps.Count - 1)];
    }
}
