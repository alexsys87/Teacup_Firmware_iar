using System.Runtime.InteropServices;

namespace TeacupHost.Core.Input;

/// <summary>
/// Generic USB joysticks and gamepads without XInput, through the old winmm API.
/// The first stick (X/Y axes) and the hat switch move X/Y, the R axis moves Z;
/// buttons 1…8 are A, B, X, Y, LB, RB, Back, Start.
/// </summary>
public sealed class WinMmJoystick : IGamepadSource
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct JoyInfoEx
    {
        public uint dwSize;
        public uint dwFlags;
        public uint dwXpos;
        public uint dwYpos;
        public uint dwZpos;
        public uint dwRpos;
        public uint dwUpos;
        public uint dwVpos;
        public uint dwButtons;
        public uint dwButtonNumber;
        public uint dwPOV;
        public uint dwReserved1;
        public uint dwReserved2;
    }

    [DllImport("winmm.dll")]
    private static extern uint joyGetPosEx(uint joyId, ref JoyInfoEx info);

    private const uint JoyReturnAll = 0x000000FF;
    private const uint NoError = 0;
    private const int MaxDevices = 16;
    private const double RescanMs = 1000;

    private readonly Func<uint, (uint Result, JoyInfoEx Info)>? _testRead;
    private bool _unavailable;
    private int _id = -1;
    private double _lastScan = double.NegativeInfinity;

    public WinMmJoystick()
    {
    }

    /// <summary>For tests: reads through the given function instead of the DLL.</summary>
    internal WinMmJoystick(Func<uint, (uint Result, JoyInfoEx Info)> read) => _testRead = read;

    public string? DeviceName => _id >= 0 ? $"Joystick #{_id + 1}" : null;

    public GamepadState Poll(double nowMs)
    {
        if (_unavailable)
            return default;

        if (_id >= 0)
        {
            if (TryRead((uint)_id, out var info))
                return Convert(info);
            _id = -1;
            _lastScan = nowMs;
            return default;
        }

        if (nowMs - _lastScan < RescanMs)
            return default;
        _lastScan = nowMs;
        for (int i = 0; i < MaxDevices; i++)
        {
            if (_unavailable)
                return default;
            if (TryRead((uint)i, out var info))
            {
                _id = i;
                return Convert(info);
            }
        }
        return default;
    }

    private bool TryRead(uint id, out JoyInfoEx info)
    {
        info = new JoyInfoEx { dwSize = (uint)Marshal.SizeOf<JoyInfoEx>(), dwFlags = JoyReturnAll };
        try
        {
            if (_testRead != null)
            {
                var r = _testRead(id);
                info = r.Info;
                return r.Result == NoError;
            }
            return joyGetPosEx(id, ref info) == NoError;
        }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException or
                                       BadImageFormatException or PlatformNotSupportedException)
        {
            _unavailable = true;
            return false;
        }
    }

    internal static GamepadState Convert(in JoyInfoEx j)
    {
        var buttons = GamepadButtons.None;
        uint raw = j.dwButtons;
        if ((raw & 0x01) != 0)
            buttons |= GamepadButtons.A;
        if ((raw & 0x02) != 0)
            buttons |= GamepadButtons.B;
        if ((raw & 0x04) != 0)
            buttons |= GamepadButtons.X;
        if ((raw & 0x08) != 0)
            buttons |= GamepadButtons.Y;
        if ((raw & 0x10) != 0)
            buttons |= GamepadButtons.LeftShoulder;
        if ((raw & 0x20) != 0)
            buttons |= GamepadButtons.RightShoulder;
        if ((raw & 0x40) != 0)
            buttons |= GamepadButtons.Back;
        if ((raw & 0x80) != 0)
            buttons |= GamepadButtons.Start;

        // Hat switch: hundredths of a degree clockwise from up, 0xFFFF (or more) when centered.
        uint pov = j.dwPOV;
        if (pov <= 35999)
        {
            if (pov >= 31500 || pov <= 4500)
                buttons |= GamepadButtons.DPadUp;
            if (pov >= 4500 && pov <= 13500)
                buttons |= GamepadButtons.DPadRight;
            if (pov >= 13500 && pov <= 22500)
                buttons |= GamepadButtons.DPadDown;
            if (pov >= 22500 && pov <= 31500)
                buttons |= GamepadButtons.DPadLeft;
        }

        // Axes are 0…65535 with the center in the middle and "up" at the small end.
        return new GamepadState(true, Axis(j.dwXpos), -Axis(j.dwYpos), 0, -Axis(j.dwRpos), 0, 0, buttons);
    }

    private static float Axis(uint v) => Math.Clamp((v - 32767.5f) / 32767.5f, -1f, 1f);
}
