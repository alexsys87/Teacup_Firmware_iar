using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Threading;
using TeacupHost.Core.Input;
using TeacupHost.ViewModels;

namespace TeacupHost.Services;

/// <summary>
/// Manual control by keyboard and gamepad. A timer reads the state of the keys and the gamepad
/// (not their events, so a key released while the focus was elsewhere can't get stuck) and
/// hands it to <see cref="JogRepeater"/>, which decides when to send moves.
/// <para>
/// Keyboard: only while the window is active, the Axes tab is open and the focus is not in a
/// text box or a drop-down list. Gamepad: whenever the window is active. Both: only while the
/// printer can be controlled by hand (online, not printing).
/// </para>
/// </summary>
public sealed class InputController : IDisposable
{
    private static readonly (Key Key, JogDirections Direction)[] JogKeys =
    {
        (Key.Left, JogDirections.XMinus), (Key.Right, JogDirections.XPlus),
        (Key.Up, JogDirections.YPlus), (Key.Down, JogDirections.YMinus),
        (Key.PageUp, JogDirections.ZPlus), (Key.PageDown, JogDirections.ZMinus),
        (Key.NumPad4, JogDirections.XMinus), (Key.NumPad6, JogDirections.XPlus),
        (Key.NumPad8, JogDirections.YPlus), (Key.NumPad2, JogDirections.YMinus),
        (Key.NumPad9, JogDirections.ZPlus), (Key.NumPad3, JogDirections.ZMinus),
    };

    private const ModifierKeys ShortcutModifiers = ModifierKeys.Control | ModifierKeys.Alt | ModifierKeys.Windows;

    private readonly MainViewModel _vm;
    private readonly Window _window;
    private readonly Func<bool> _axesTabOpen;
    private readonly DispatcherTimer _timer;
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly JogRepeater _repeater = new();
    private readonly GamepadMapper _mapper = new();
    private GamepadPoller? _poller;

    /// <param name="axesTabOpen">Tells whether the Axes tab is the one shown (the keys act only there).</param>
    public InputController(MainViewModel vm, Window window, Func<bool> axesTabOpen)
    {
        _vm = vm;
        _window = window;
        _axesTabOpen = axesTabOpen;
        _timer = new DispatcherTimer(DispatcherPriority.Input, window.Dispatcher)
        {
            Interval = TimeSpan.FromMilliseconds(20),
        };
        _timer.Tick += (_, _) => Tick();
        _timer.Start();
    }

    public void Dispose() => _timer.Stop();

    // ---------------------------------------------------------------- keyboard

    private bool KeyboardAllowed() =>
        _vm.KeyboardControl && _window.IsActive && _axesTabOpen() && !IsTextInputFocused();

    /// <summary>These controls use the arrow keys and Home/End themselves.</summary>
    private static bool IsTextInputFocused() =>
        Keyboard.FocusedElement is TextBoxBase or PasswordBox or ComboBox or ComboBoxItem;

    private static JogDirections ReadKeys()
    {
        if ((Keyboard.Modifiers & ShortcutModifiers) != 0)
            return JogDirections.None;
        var d = JogDirections.None;
        foreach (var (key, direction) in JogKeys)
        {
            if (Keyboard.IsKeyDown(key))
                d |= direction;
        }
        return d;
    }

    private static bool IsJogKey(Key key)
    {
        foreach (var (k, _) in JogKeys)
        {
            if (k == key)
                return true;
        }
        return false;
    }

    /// <summary>
    /// Call from the window's PreviewKeyDown. The jog keys are only swallowed here (so lists and
    /// scroll viewers don't move as well); the moves themselves come from the timer. The other keys
    /// act once per press.
    /// </summary>
    public void OnPreviewKeyDown(KeyEventArgs e)
    {
        if (!KeyboardAllowed() || !_vm.CanControl || (Keyboard.Modifiers & ShortcutModifiers) != 0)
            return;

        if (IsJogKey(e.Key))
        {
            e.Handled = true;
            return;
        }

        switch (e.Key)
        {
            case Key.Add:
            case Key.OemPlus:
                if (!e.IsRepeat)
                    _vm.ChangeJogStep(+1);
                break;
            case Key.Subtract:
            case Key.OemMinus:
                if (!e.IsRepeat)
                    _vm.ChangeJogStep(-1);
                break;
            case Key.Home:
                if (!e.IsRepeat)
                    _vm.HomeFromInput("");
                break;
            case Key.End:
                if (!e.IsRepeat)
                    _vm.MotorsOffFromInput();
                break;
            case Key.E:
                if (!e.IsRepeat)
                    _vm.ExtrudeFromInput(+1);
                break;
            case Key.R:
                if (!e.IsRepeat)
                    _vm.ExtrudeFromInput(-1);
                break;
            default:
                return;
        }
        e.Handled = true;
    }

    // ---------------------------------------------------------------- timer

    private void Tick()
    {
        double now = _clock.Elapsed.TotalMilliseconds;
        bool active = _window.IsActive;
        var directions = JogDirections.None;
        var actions = GamepadActions.None;

        if (_vm.GamepadControl)
        {
            _poller ??= new GamepadPoller();
            var pad = _poller.Poll(now);
            _vm.SetGamepadName(pad.Connected ? _poller.DeviceName : null);
            // The buttons are read even while the window is in the background, so that a press
            // made there is not carried out later.
            var pressed = _mapper.Pressed(pad);
            if (active)
            {
                directions |= _mapper.Directions(pad);
                actions = pressed;
            }
        }
        else
        {
            _mapper.Reset();
            _vm.SetGamepadName(null);
        }

        if (KeyboardAllowed())
            directions |= ReadKeys();

        if (!_vm.CanControl)
        {
            directions = JogDirections.None;
            actions = GamepadActions.None;
        }

        var tick = _repeater.Update(directions, now, _vm.CurrentJogSettings, _vm.QueuedCommands);
        if (tick.Move is { } move)
            _vm.JogBy(move);
        if (tick.Released)
            _vm.RefreshPositionAfterJog();
        if (actions != GamepadActions.None)
            Run(actions);
    }

    private void Run(GamepadActions a)
    {
        if (a.HasFlag(GamepadActions.StepDown))
            _vm.ChangeJogStep(-1);
        if (a.HasFlag(GamepadActions.StepUp))
            _vm.ChangeJogStep(+1);
        if (a.HasFlag(GamepadActions.HomeAll))
            _vm.HomeFromInput("");
        if (a.HasFlag(GamepadActions.HomeXY))
            _vm.HomeFromInput("XY");
        if (a.HasFlag(GamepadActions.HomeZ))
            _vm.HomeFromInput("Z");
        if (a.HasFlag(GamepadActions.MotorsOff))
            _vm.MotorsOffFromInput();
        if (a.HasFlag(GamepadActions.Retract))
            _vm.ExtrudeFromInput(-1);
        if (a.HasFlag(GamepadActions.Extrude))
            _vm.ExtrudeFromInput(+1);
    }
}
