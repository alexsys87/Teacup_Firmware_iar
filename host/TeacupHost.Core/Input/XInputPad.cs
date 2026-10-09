using System.Runtime.InteropServices;

namespace TeacupHost.Core.Input;

/// <summary>Xbox-compatible gamepads through XInput (xinput1_4.dll, xinput9_1_0.dll on old Windows).</summary>
public sealed class XInputPad : IGamepadSource
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct XInputGamepad
    {
        public ushort wButtons;
        public byte bLeftTrigger;
        public byte bRightTrigger;
        public short sThumbLX;
        public short sThumbLY;
        public short sThumbRX;
        public short sThumbRY;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct XInputState
    {
        public uint dwPacketNumber;
        public XInputGamepad Gamepad;
    }

    [DllImport("xinput1_4.dll", EntryPoint = "XInputGetState")]
    private static extern uint GetState14(uint userIndex, out XInputState state);

    [DllImport("xinput9_1_0.dll", EntryPoint = "XInputGetState")]
    private static extern uint GetState910(uint userIndex, out XInputState state);

    private const uint Success = 0;
    private const int Slots = 4;
    private const double RescanMs = 1000;

    private delegate uint GetStateFn(uint index, out XInputState state);

    private readonly GetStateFn? _testRead;
    private bool _useOld;
    private bool _unavailable;
    private int _slot = -1;
    private double _lastScan = double.NegativeInfinity;

    public XInputPad()
    {
    }

    /// <summary>For tests: reads through the given function instead of the DLL.</summary>
    internal XInputPad(Func<uint, (uint Result, XInputState State)> read)
    {
        _testRead = (uint i, out XInputState s) =>
        {
            var r = read(i);
            s = r.State;
            return r.Result;
        };
    }

    public string? DeviceName => _slot >= 0 ? $"Xbox / XInput #{_slot + 1}" : null;

    public GamepadState Poll(double nowMs)
    {
        if (_unavailable)
            return default;

        if (_slot >= 0)
        {
            if (TryRead((uint)_slot, out var st))
                return Convert(st.Gamepad);
            _slot = -1;
            _lastScan = nowMs;
            return default;
        }

        // Asking an empty slot is slow, so empty slots are only checked once a second.
        if (nowMs - _lastScan < RescanMs)
            return default;
        _lastScan = nowMs;
        for (int i = 0; i < Slots; i++)
        {
            if (_unavailable)
                return default;
            if (TryRead((uint)i, out var st))
            {
                _slot = i;
                return Convert(st.Gamepad);
            }
        }
        return default;
    }

    private bool TryRead(uint index, out XInputState state)
    {
        state = default;
        try
        {
            if (_testRead != null)
                return _testRead(index, out state) == Success;
            if (!_useOld)
            {
                try
                {
                    return GetState14(index, out state) == Success;
                }
                catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException)
                {
                    _useOld = true;
                }
            }
            return GetState910(index, out state) == Success;
        }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException or
                                       BadImageFormatException or PlatformNotSupportedException)
        {
            // No XInput on this system (not Windows, N edition): this source stays silent.
            _unavailable = true;
            return false;
        }
    }

    internal static GamepadState Convert(in XInputGamepad g) => new(
        true,
        Axis(g.sThumbLX), Axis(g.sThumbLY), Axis(g.sThumbRX), Axis(g.sThumbRY),
        g.bLeftTrigger / 255f, g.bRightTrigger / 255f,
        (GamepadButtons)g.wButtons);

    private static float Axis(short v) => Math.Clamp(v / 32767f, -1f, 1f);
}
