using System.Runtime.InteropServices;
using TeacupHost.Core.Input;

namespace TeacupHost.Core.Tests;

/// <summary>Manual control by keyboard and gamepad: directions, hold-to-move, gamepad mapping and readers.</summary>
public class InputTests
{
    private static readonly JogSettings Default = new(Step: 10, FeedXY: 3000, FeedZ: 240);

    // ---------------------------------------------------------------- directions and steps

    [Fact]
    public void OppositeDirectionsCancel()
    {
        Assert.Equal(JogDirections.None, (JogDirections.XPlus | JogDirections.XMinus).Normalize());
        Assert.Equal(JogDirections.YPlus, (JogDirections.YPlus | JogDirections.XPlus | JogDirections.XMinus).Normalize());
    }

    [Fact]
    public void ZWinsOverXY()
    {
        var d = (JogDirections.XPlus | JogDirections.YMinus | JogDirections.ZPlus).Normalize();
        Assert.Equal(JogDirections.ZPlus, d);
        Assert.Equal(1, d.Sign('Z'));
        Assert.Equal(0, d.Sign('X'));
    }

    [Fact]
    public void StepListIsWalkedAndClamped()
    {
        double[] steps = { 0.1, 1, 10, 50 };
        Assert.Equal(50, JogStepPicker.Next(steps, 10, +1));
        Assert.Equal(1, JogStepPicker.Next(steps, 10, -1));
        Assert.Equal(50, JogStepPicker.Next(steps, 50, +1));
        Assert.Equal(0.1, JogStepPicker.Next(steps, 0.1, -1));
        // A value that is not on the list starts from the nearest item.
        Assert.Equal(50, JogStepPicker.Next(steps, 12, +1));
    }

    // ---------------------------------------------------------------- hold to move

    [Fact]
    public void TapSendsOneStep()
    {
        var r = new JogRepeater();
        var moves = new List<JogMove>();
        Run(r, JogDirections.XPlus, 0, 200, moves);          // key held 200 ms: still a tap
        Run(r, JogDirections.None, 200, 400, moves);
        Assert.Equal(1, moves.Count);
        Assert.Equal(10, moves[0].X);
        Assert.Equal(0, moves[0].Y);
        Assert.False(moves[0].IsChunk);
        Assert.Equal(3000, moves[0].Feed);
    }

    [Fact]
    public void VeryShortTapIsNotLost()
    {
        var r = new JogRepeater();
        Assert.Null(r.Update(JogDirections.YMinus, 0, Default, 0).Move);
        // Released 10 ms later, before the chord window ended.
        var t = r.Update(JogDirections.None, 10, Default, 0);
        Assert.NotNull(t.Move);
        Assert.Equal(-10, t.Move!.Value.Y);
        Assert.False(t.Released);
    }

    [Fact]
    public void KeysOfADiagonalMakeOneMove()
    {
        var r = new JogRepeater();
        var moves = new List<JogMove>();
        r.Update(JogDirections.XPlus, 0, Default, 0);
        var t = r.Update(JogDirections.XPlus | JogDirections.YPlus, 15, Default, 0);   // second key 15 ms later
        Assert.Null(t.Move);
        t = r.Update(JogDirections.XPlus | JogDirections.YPlus, 35, Default, 0);
        Assert.NotNull(t.Move);
        Assert.Equal(10, t.Move!.Value.X);
        Assert.Equal(10, t.Move!.Value.Y);
    }

    [Fact]
    public void ZStepIsLimitedAndUsesTheZFeed()
    {
        var r = new JogRepeater();
        var s = new JogSettings(50, 3000, 240);
        r.Update(JogDirections.ZPlus, 0, s, 0);
        var m = r.Update(JogDirections.ZPlus, 40, s, 0).Move!.Value;
        Assert.Equal(10, m.Z);
        Assert.Equal(0, m.X);
        Assert.Equal(240, m.Feed);
    }

    [Fact]
    public void HoldKeepsMovingInChunksAndStopsOnRelease()
    {
        var r = new JogRepeater();
        var moves = new List<JogMove>();
        var s = new JogSettings(50, 3000, 240);           // 50 mm step at 50 mm/s
        Run(r, JogDirections.XPlus, 0, 3000, moves, s);
        Assert.False(moves[0].IsChunk);
        Assert.Equal(50, moves[0].X);
        // While held the moves are chunks of ChunkMs at the full feed rate: 50 mm/s * 0.15 s = 7.5 mm.
        double chunkLength = 3000 / 60.0 * r.ChunkMs / 1000;
        var chunks = moves.Skip(1).ToList();
        Assert.True(chunks.Count > 5);
        Assert.True(chunks.All(c => c.IsChunk && Math.Abs(c.X - chunkLength) < 1e-9));

        var release = r.Update(JogDirections.None, 3000, s, 0);
        Assert.True(release.Released);
        Assert.Null(release.Move);

        // Nothing more after the release.
        var after = new List<JogMove>();
        Run(r, JogDirections.None, 3020, 4000, after, s);
        Assert.Empty(after);
    }

    [Fact]
    public void QueuedMotionNeverGrowsBeyondTheLead()
    {
        // Model: the printer executes moves back to back at the full feed rate.
        var r = new JogRepeater();
        var s = new JogSettings(50, 3000, 240);
        double busyUntil = 0, worst = 0;
        double total = 0;
        for (double t = 0; t <= 5000; t += 20)
        {
            var m = r.Update(JogDirections.XPlus, t, s, 0).Move;
            if (m != null)
            {
                busyUntil = Math.Max(busyUntil, t) + m.Value.DurationMs;
                total += m.Value.X;
            }
            worst = Math.Max(worst, busyUntil - t);
        }
        // At most the lead plus one chunk (the first move of 50 mm takes a second, so allow that for it).
        Assert.True(worst <= 1000 + 1, $"queue of {worst} ms");
        // The motion is continuous: close to 50 mm/s for the 5 s.
        Assert.True(total > 200, $"moved {total} mm");
    }

    [Fact]
    public void ReleaseLeavesAtMostTheLeadPlusOneChunk()
    {
        // The printer runs the moves back to back at the full feed rate. Whenever the key is
        // let go, what is still queued is bounded, so the tool stops soon (and never far away).
        foreach (double release in new[] { 2000.0, 2013.0, 2047.0, 2101.0, 2160.0, 2222.0, 2999.0 })
        {
            var r = new JogRepeater();
            var s = new JogSettings(50, 3000, 240);
            double busyUntil = 0;
            double t = 0;
            for (; t < release; t += 20)
            {
                var m = r.Update(JogDirections.XPlus, t, s, 0).Move;
                if (m != null)
                    busyUntil = Math.Max(busyUntil, t) + m.Value.DurationMs;
            }
            double left = Math.Max(0, busyUntil - release);
            Assert.True(left <= r.LeadMs + r.ChunkMs + 1e-6, $"released at {release}: {left} ms still queued");
            var last = r.Update(JogDirections.None, release, s, 0);
            Assert.Null(last.Move);
        }
    }

    [Fact]
    public void NoChunksWhileCommandsAreWaiting()
    {
        var r = new JogRepeater();
        var s = new JogSettings(1, 3000, 240);
        r.Update(JogDirections.XPlus, 0, s, 0);
        r.Update(JogDirections.XPlus, 40, s, 0);              // the tap
        for (double t = 400; t < 1000; t += 20)
            Assert.Null(r.Update(JogDirections.XPlus, t, s, queuedCommands: 3).Move);
        Assert.NotNull(r.Update(JogDirections.XPlus, 1000, s, 0).Move);
    }

    [Fact]
    public void TinyStepsAreRateLimited()
    {
        var r = new JogRepeater();
        var s = new JogSettings(0.1, 3000, 240);
        var moves = new List<JogMove>();
        Run(r, JogDirections.XPlus, 0, 2000, moves, s);
        // After the 350 ms delay at most one chunk per 50 ms: 0.1 mm each.
        Assert.True(moves.Count <= 1 + 1650 / 50 + 1, $"{moves.Count} moves");
        Assert.True(moves.Skip(1).All(m => Math.Abs(m.X - 0.1) < 1e-9));
    }

    [Fact]
    public void ChangingDirectionWhileHoldingKeepsMoving()
    {
        var r = new JogRepeater();
        var moves = new List<JogMove>();
        var s = new JogSettings(50, 3000, 240);
        Run(r, JogDirections.XPlus, 0, 1000, moves, s);
        int before = moves.Count;
        Run(r, JogDirections.XPlus | JogDirections.YPlus, 1000, 2000, moves, s);
        var diagonal = moves.Skip(before).Where(m => m.Y > 0).ToList();
        Assert.True(diagonal.Count > 0);
        Assert.True(diagonal.All(m => m.IsChunk && m.X > 0));
    }

    private static void Run(JogRepeater r, JogDirections held, double from, double to, List<JogMove> moves, JogSettings? s = null)
    {
        for (double t = from; t < to; t += 20)
        {
            var tick = r.Update(held, t, s ?? Default, 0);
            if (tick.Move != null)
                moves.Add(tick.Move.Value);
        }
    }

    // ---------------------------------------------------------------- gamepad mapping

    private static GamepadState Pad(float lx = 0, float ly = 0, float rx = 0, float ry = 0,
        float lt = 0, float rt = 0, GamepadButtons b = GamepadButtons.None) =>
        new(true, lx, ly, rx, ry, lt, rt, b);

    [Fact]
    public void StickInsideTheDeadzoneDoesNothing()
    {
        var m = new GamepadMapper();
        Assert.Equal(JogDirections.None, m.Directions(Pad(lx: 0.2f, ly: -0.2f)));
        Assert.Equal(JogDirections.None, m.Directions(default));           // not connected
    }

    [Fact]
    public void LeftStickGivesEightDirectionsWithSnappedAxes()
    {
        var m = new GamepadMapper();
        Assert.Equal(JogDirections.XPlus, m.Directions(Pad(lx: 1)));
        Assert.Equal(JogDirections.YMinus, m.Directions(Pad(ly: -0.9f)));
        // Almost straight right with a little up: still only X.
        Assert.Equal(JogDirections.XPlus, m.Directions(Pad(lx: 0.9f, ly: 0.2f)));
        Assert.Equal(JogDirections.XMinus | JogDirections.YPlus, m.Directions(Pad(lx: -0.7f, ly: 0.7f)));
    }

    [Fact]
    public void DPadAndRightStick()
    {
        var m = new GamepadMapper();
        Assert.Equal(JogDirections.XMinus, m.Directions(Pad(b: GamepadButtons.DPadLeft)));
        Assert.Equal(JogDirections.YPlus | JogDirections.XPlus,
            m.Directions(Pad(b: GamepadButtons.DPadUp | GamepadButtons.DPadRight)));
        Assert.Equal(JogDirections.ZPlus, m.Directions(Pad(ry: 0.8f)));
        Assert.Equal(JogDirections.ZMinus, m.Directions(Pad(ry: -0.8f, rx: 0.1f)));
        Assert.Equal(JogDirections.None, m.Directions(Pad(rx: 1)));           // right stick sideways: nothing
        // Z wins over the left stick.
        Assert.Equal(JogDirections.ZPlus, m.Directions(Pad(lx: 1, ry: 1)));
        // Stick and D-pad pushing opposite ways cancel.
        Assert.Equal(JogDirections.None, m.Directions(Pad(lx: 1, b: GamepadButtons.DPadLeft)));
    }

    [Fact]
    public void ButtonsActOncePerPress()
    {
        var m = new GamepadMapper();
        // A button held while the pad is connected does nothing.
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(b: GamepadButtons.A)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(b: GamepadButtons.A)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));

        Assert.Equal(GamepadActions.HomeAll, m.Pressed(Pad(b: GamepadButtons.A)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(b: GamepadButtons.A)));      // still held
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.HomeXY, m.Pressed(Pad(b: GamepadButtons.X)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.HomeZ, m.Pressed(Pad(b: GamepadButtons.Y)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.MotorsOff, m.Pressed(Pad(b: GamepadButtons.Back)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.StepUp, m.Pressed(Pad(b: GamepadButtons.RightShoulder)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.StepDown, m.Pressed(Pad(b: GamepadButtons.LeftShoulder)));
    }

    [Fact]
    public void TriggersExtrudeAndRetractOnCrossingTheThreshold()
    {
        var m = new GamepadMapper();
        m.Pressed(Pad());
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(rt: 0.3f)));
        Assert.Equal(GamepadActions.Extrude, m.Pressed(Pad(rt: 0.7f)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(rt: 1)));
        Assert.Equal(GamepadActions.None, m.Pressed(Pad()));
        Assert.Equal(GamepadActions.Retract, m.Pressed(Pad(lt: 1)));
    }

    [Fact]
    public void ReconnectingForgetsTheOldButtons()
    {
        var m = new GamepadMapper();
        m.Pressed(Pad());
        Assert.Equal(GamepadActions.HomeAll, m.Pressed(Pad(b: GamepadButtons.A)));
        Assert.Equal(GamepadActions.None, m.Pressed(default));                       // unplugged
        // Plugged in again with A held: not a press.
        Assert.Equal(GamepadActions.None, m.Pressed(Pad(b: GamepadButtons.A)));
    }

    // ---------------------------------------------------------------- XInput

    [Fact]
    public void XInputStructsHaveTheNativeSize()
    {
        Assert.Equal(12, Marshal.SizeOf<XInputPad.XInputGamepad>());
        Assert.Equal(16, Marshal.SizeOf<XInputPad.XInputState>());
    }

    [Fact]
    public void XInputValuesAreConverted()
    {
        var g = new XInputPad.XInputGamepad
        {
            wButtons = 0x1000 | 0x0004,                 // A + D-pad left
            bLeftTrigger = 255,
            bRightTrigger = 0,
            sThumbLX = short.MaxValue,
            sThumbLY = short.MinValue,
            sThumbRX = 0,
            sThumbRY = 16384,
        };
        var s = XInputPad.Convert(g);
        Assert.True(s.Connected);
        Assert.Equal(GamepadButtons.A | GamepadButtons.DPadLeft, s.Buttons);
        Assert.Equal(1f, s.LeftX);
        Assert.Equal(-1f, s.LeftY);
        Assert.Equal(0.5f, s.RightY, 2);
        Assert.Equal(1f, s.LeftTrigger);
        Assert.Equal(0f, s.RightTrigger);
    }

    [Fact]
    public void XInputFindsTheSlotAndScansRarely()
    {
        int calls = 0;
        bool plugged = false;
        var pad = new XInputPad(i =>
        {
            calls++;
            return (plugged && i == 2 ? 0u : 1167u, default);
        });

        Assert.False(pad.Poll(0).Connected);
        int first = calls;
        Assert.Equal(4, first);                                  // all four slots once
        Assert.False(pad.Poll(500).Connected);
        Assert.Equal(first, calls);                              // too early for another scan
        plugged = true;
        Assert.False(pad.Poll(900).Connected);
        Assert.True(pad.Poll(1100).Connected);
        Assert.Equal("Xbox / XInput #3", pad.DeviceName);

        int before = calls;
        Assert.True(pad.Poll(1120).Connected);
        Assert.Equal(before + 1, calls);                         // a known slot is read directly

        plugged = false;
        Assert.False(pad.Poll(1140).Connected);
        Assert.Null(pad.DeviceName);
    }

    // ---------------------------------------------------------------- generic joystick

    [Fact]
    public void JoystickStructHasTheNativeSize() =>
        Assert.Equal(52, Marshal.SizeOf<WinMmJoystick.JoyInfoEx>());

    private static WinMmJoystick.JoyInfoEx Joy(uint x = 32768, uint y = 32768, uint r = 32768, uint pov = 0xFFFF, uint buttons = 0) =>
        new() { dwXpos = x, dwYpos = y, dwRpos = r, dwPOV = pov, dwButtons = buttons };

    [Fact]
    public void JoystickAxesAreCenteredAndUpIsPositive()
    {
        var center = WinMmJoystick.Convert(Joy());
        Assert.Equal(0f, center.LeftX, 2);
        Assert.Equal(0f, center.LeftY, 2);

        var upRight = WinMmJoystick.Convert(Joy(x: 65535, y: 0, r: 0));
        Assert.Equal(1f, upRight.LeftX, 2);
        Assert.Equal(1f, upRight.LeftY, 2);                       // pushed up: small raw value
        Assert.Equal(1f, upRight.RightY, 2);
    }

    [Fact]
    public void JoystickHatAndButtons()
    {
        Assert.Equal(GamepadButtons.None, WinMmJoystick.Convert(Joy(pov: 0xFFFF)).Buttons);
        Assert.Equal(GamepadButtons.None, WinMmJoystick.Convert(Joy(pov: 0xFFFFFFFF)).Buttons);
        Assert.Equal(GamepadButtons.DPadUp, WinMmJoystick.Convert(Joy(pov: 0)).Buttons);
        Assert.Equal(GamepadButtons.DPadRight, WinMmJoystick.Convert(Joy(pov: 9000)).Buttons);
        Assert.Equal(GamepadButtons.DPadDown, WinMmJoystick.Convert(Joy(pov: 18000)).Buttons);
        Assert.Equal(GamepadButtons.DPadLeft, WinMmJoystick.Convert(Joy(pov: 27000)).Buttons);
        Assert.Equal(GamepadButtons.DPadUp | GamepadButtons.DPadRight, WinMmJoystick.Convert(Joy(pov: 4500)).Buttons);
        Assert.Equal(GamepadButtons.DPadUp | GamepadButtons.DPadLeft, WinMmJoystick.Convert(Joy(pov: 31500)).Buttons);

        var b = WinMmJoystick.Convert(Joy(buttons: 0b1000_0101)).Buttons;      // buttons 1, 3 and 8
        Assert.Equal(GamepadButtons.A | GamepadButtons.X | GamepadButtons.Start, b);
    }

    [Fact]
    public void JoystickIsFoundByScanning()
    {
        var pad = new WinMmJoystick(id => (id == 5 ? 0u : 167u, Joy(buttons: 1)));
        var s = pad.Poll(0);
        Assert.True(s.Connected);
        Assert.Equal("Joystick #6", pad.DeviceName);
        Assert.Equal(GamepadButtons.A, s.Buttons);
    }

    // ---------------------------------------------------------------- choosing a device

    private sealed class FakePad : IGamepadSource
    {
        public bool Connected;
        public int Polls;
        public string Name = "";
        public string? DeviceName => Connected ? Name : null;

        public GamepadState Poll(double nowMs)
        {
            Polls++;
            return Connected ? new GamepadState(true, 0, 0, 0, 0, 0, 0, GamepadButtons.None) : default;
        }
    }

    [Fact]
    public void PollerPrefersTheFirstSourceAndFallsBack()
    {
        var xi = new FakePad { Name = "xinput" };
        var wm = new FakePad { Name = "winmm", Connected = true };
        var poller = new GamepadPoller(xi, wm);

        Assert.True(poller.Poll(0).Connected);
        Assert.Equal("winmm", poller.DeviceName);

        // An XInput pad appears, but the joystick in use is kept until it goes away.
        xi.Connected = true;
        Assert.True(poller.Poll(10).Connected);
        Assert.Equal("winmm", poller.DeviceName);

        wm.Connected = false;
        Assert.True(poller.Poll(20).Connected);
        Assert.Equal("xinput", poller.DeviceName);

        xi.Connected = false;
        Assert.False(poller.Poll(30).Connected);
        Assert.Null(poller.DeviceName);
    }
}
