namespace TeacupHost.Core.Input;

/// <summary>
/// Maps a gamepad to the manual control:
/// left stick or D-pad: X/Y, right stick (up/down): Z, LB/RB: step down/up,
/// A: home all, X: home X Y, Y: home Z, Back: motors off, LT/RT: retract/extrude.
/// </summary>
public sealed class GamepadMapper
{
    // tan(22.5°): a stick within 22.5° of an axis moves along that axis only.
    private const float SnapTan = 0.41421356f;

    /// <summary>Sticks below this deflection (0…1) are at rest.</summary>
    public float Deadzone { get; set; } = 0.35f;

    /// <summary>Triggers above this (0…1) count as pressed.</summary>
    public float TriggerThreshold { get; set; } = 0.5f;

    private GamepadState _previous;
    private bool _hasPrevious;

    public void Reset() => _hasPrevious = false;

    /// <summary>Directions held on the left stick, the D-pad and the right stick.</summary>
    public JogDirections Directions(in GamepadState s)
    {
        if (!s.Connected)
            return JogDirections.None;

        var d = JogDirections.None;

        // Left stick: eight directions with the axes snapped.
        float x = s.LeftX, y = s.LeftY;
        if (Math.Sqrt(x * x + y * y) >= Deadzone)
        {
            float ax = Math.Abs(x), ay = Math.Abs(y);
            if (ax > ay * SnapTan)
                d |= x > 0 ? JogDirections.XPlus : JogDirections.XMinus;
            if (ay > ax * SnapTan)
                d |= y > 0 ? JogDirections.YPlus : JogDirections.YMinus;
        }

        var b = s.Buttons;
        if (b.HasFlag(GamepadButtons.DPadRight))
            d |= JogDirections.XPlus;
        if (b.HasFlag(GamepadButtons.DPadLeft))
            d |= JogDirections.XMinus;
        if (b.HasFlag(GamepadButtons.DPadUp))
            d |= JogDirections.YPlus;
        if (b.HasFlag(GamepadButtons.DPadDown))
            d |= JogDirections.YMinus;

        // Right stick: only its vertical deflection, up raises the nozzle.
        if (s.RightY >= Deadzone && Math.Abs(s.RightY) > Math.Abs(s.RightX) * SnapTan)
            d |= JogDirections.ZPlus;
        else if (s.RightY <= -Deadzone && Math.Abs(s.RightY) > Math.Abs(s.RightX) * SnapTan)
            d |= JogDirections.ZMinus;

        return d.Normalize();
    }

    /// <summary>
    /// Buttons that went down since the previous call. The first reading after a connect
    /// only remembers the state, so a button held while plugging in does nothing.
    /// </summary>
    public GamepadActions Pressed(in GamepadState s)
    {
        if (!s.Connected)
        {
            _hasPrevious = false;
            return GamepadActions.None;
        }
        var prev = _previous;
        bool had = _hasPrevious;
        _previous = s;
        _hasPrevious = true;
        if (!had)
            return GamepadActions.None;

        var down = s.Buttons & ~prev.Buttons;
        var a = GamepadActions.None;
        if (down.HasFlag(GamepadButtons.A))
            a |= GamepadActions.HomeAll;
        if (down.HasFlag(GamepadButtons.X))
            a |= GamepadActions.HomeXY;
        if (down.HasFlag(GamepadButtons.Y))
            a |= GamepadActions.HomeZ;
        if (down.HasFlag(GamepadButtons.Back))
            a |= GamepadActions.MotorsOff;
        if (down.HasFlag(GamepadButtons.RightShoulder))
            a |= GamepadActions.StepUp;
        if (down.HasFlag(GamepadButtons.LeftShoulder))
            a |= GamepadActions.StepDown;
        if (s.RightTrigger >= TriggerThreshold && prev.RightTrigger < TriggerThreshold)
            a |= GamepadActions.Extrude;
        if (s.LeftTrigger >= TriggerThreshold && prev.LeftTrigger < TriggerThreshold)
            a |= GamepadActions.Retract;
        return a;
    }
}
