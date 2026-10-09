namespace TeacupHost.Core.Input;

/// <summary>Reads the first connected gamepad: XInput first, then generic joysticks.</summary>
public sealed class GamepadPoller
{
    private readonly IGamepadSource[] _sources;
    private IGamepadSource? _active;

    public GamepadPoller()
        : this(new XInputPad(), new WinMmJoystick())
    {
    }

    public GamepadPoller(params IGamepadSource[] sources) => _sources = sources;

    /// <summary>Name of the device that is read now, null if there is none.</summary>
    public string? DeviceName => _active?.DeviceName;

    public GamepadState Poll(double nowMs)
    {
        if (_active != null)
        {
            var s = _active.Poll(nowMs);
            if (s.Connected)
                return s;
            _active = null;
        }
        foreach (var src in _sources)
        {
            var s = src.Poll(nowMs);
            if (s.Connected)
            {
                _active = src;
                return s;
            }
        }
        return default;
    }
}
