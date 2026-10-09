namespace TeacupHost.Core.Input;

/// <summary>Buttons of a gamepad. The values are those of XINPUT_GAMEPAD.wButtons.</summary>
[Flags]
public enum GamepadButtons
{
    None = 0,
    DPadUp = 0x0001,
    DPadDown = 0x0002,
    DPadLeft = 0x0004,
    DPadRight = 0x0008,
    Start = 0x0010,
    Back = 0x0020,
    LeftThumb = 0x0040,
    RightThumb = 0x0080,
    LeftShoulder = 0x0100,
    RightShoulder = 0x0200,
    A = 0x1000,
    B = 0x2000,
    X = 0x4000,
    Y = 0x8000,
}

/// <summary>
/// One reading of a gamepad. Sticks are -1…+1 with +Y up (like XInput), triggers 0…1.
/// </summary>
public readonly record struct GamepadState(
    bool Connected,
    float LeftX,
    float LeftY,
    float RightX,
    float RightY,
    float LeftTrigger,
    float RightTrigger,
    GamepadButtons Buttons);

/// <summary>Something a gamepad button asks the program to do once per press.</summary>
[Flags]
public enum GamepadActions
{
    None = 0,
    HomeAll = 1,
    HomeXY = 2,
    HomeZ = 4,
    MotorsOff = 8,
    StepUp = 16,
    StepDown = 32,
    Extrude = 64,
    Retract = 128,
}

/// <summary>A source of gamepad readings (XInput, generic joystick).</summary>
public interface IGamepadSource
{
    /// <summary>Name of the device in use, null while none is connected.</summary>
    string? DeviceName { get; }

    /// <summary>Read the device. Rescans for devices now and then while none is connected.</summary>
    GamepadState Poll(double nowMs);
}
