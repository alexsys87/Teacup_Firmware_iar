namespace TeacupHost.Core.Input;

/// <summary>
/// Turns "which directions are held now" into moves to send.
/// <list type="bullet">
/// <item>A tap (short press) gives one move of the selected step.</item>
/// <item>Holding longer than <see cref="InitialDelayMs"/> keeps the tool moving: short chunks are
/// sent one after another, each only when the previous ones are about to run out. The queue of
/// the printer therefore never holds much more than <see cref="LeadMs"/> of motion, and the tool
/// stops soon after the key is released (within <see cref="LeadMs"/> + <see cref="ChunkMs"/> at the
/// full feed rate; the firmware's M410 would be quicker, but it stops abruptly and may skip steps).</item>
/// </list>
/// Time is passed in by the caller (milliseconds), so the class has no timers and is easy to test.
/// </summary>
public sealed class JogRepeater
{
    /// <summary>Z moves are slow and dangerous for the nozzle: one command moves 10 mm at most.</summary>
    public const double MaxZStep = 10;

    /// <summary>
    /// Keys of a diagonal rarely go down in the same millisecond: the first move waits this long
    /// so that "right, then up" is one diagonal move and not two.
    /// </summary>
    public double ChordWindowMs { get; set; } = 30;

    /// <summary>A press shorter than this is a tap; a longer one starts the continuous motion.</summary>
    public double InitialDelayMs { get; set; } = 350;

    /// <summary>Duration of the longest chunk sent while a key is held.</summary>
    public double ChunkMs { get; set; } = 150;

    /// <summary>The next chunk is sent when no more than this much of the previous motion is left.</summary>
    public double LeadMs { get; set; } = 80;

    /// <summary>Chunks are never sent more often than this (tiny steps would flood the link).</summary>
    public double MinIntervalMs { get; set; } = 50;

    private bool _active;
    private bool _tapSent;
    private bool _chunkSent;
    private JogDirections _current;
    private double _pressedAt;
    private double _lastFire;
    private double _busyUntil;

    /// <summary>A key, stick or button is down right now.</summary>
    public bool IsActive => _active;

    public void Reset()
    {
        _active = false;
        _tapSent = false;
        _chunkSent = false;
        _current = JogDirections.None;
    }

    /// <param name="held">Directions held now (any source, they are combined by the caller).</param>
    /// <param name="nowMs">Monotonic time in milliseconds.</param>
    /// <param name="queuedCommands">Commands waiting to be written to the printer; no chunks are added while there are any.</param>
    public JogTick Update(JogDirections held, double nowMs, JogSettings settings, int queuedCommands)
    {
        held = held.Normalize();

        if (held == JogDirections.None)
        {
            if (!_active)
                return default;

            JogMove? tap = null;
            // Released before the chord window ended: the tap would be lost, send it now.
            if (!_tapSent)
                tap = Fire(_current, settings, settings.Step, isChunk: false, nowMs);
            bool released = _chunkSent;
            Reset();
            return new JogTick(tap, released);
        }

        if (!_active)
        {
            _active = true;
            _tapSent = false;
            _chunkSent = false;
            _pressedAt = nowMs;
        }
        _current = held;
        double heldMs = nowMs - _pressedAt;

        if (!_tapSent)
        {
            if (heldMs < ChordWindowMs)
                return default;
            return new JogTick(Fire(held, settings, settings.Step, isChunk: false, nowMs), false);
        }

        if (heldMs < InitialDelayMs || queuedCommands > 0 ||
            _busyUntil - nowMs > LeadMs || nowMs - _lastFire < MinIntervalMs)
            return default;

        double feed = held.Sign('Z') != 0 ? settings.FeedZ : settings.FeedXY;
        double chunk = Math.Min(settings.Step, feed / 60.0 * ChunkMs / 1000.0);
        _chunkSent = true;
        return new JogTick(Fire(held, settings, chunk, isChunk: true, nowMs), false);
    }

    private JogMove? Fire(JogDirections d, JogSettings s, double distance, bool isChunk, double nowMs)
    {
        d = d.Normalize();
        bool z = d.Sign('Z') != 0;
        double feed = z ? s.FeedZ : s.FeedXY;
        if (d == JogDirections.None || distance <= 0 || feed <= 0)
            return null;

        double zDistance = Math.Min(distance, MaxZStep);
        var move = new JogMove(d.Sign('X') * distance, d.Sign('Y') * distance, d.Sign('Z') * zDistance, feed, isChunk);

        if (!isChunk)
            _tapSent = true;
        _lastFire = nowMs;
        _busyUntil = Math.Max(_busyUntil, nowMs) + move.DurationMs;
        return move;
    }
}
