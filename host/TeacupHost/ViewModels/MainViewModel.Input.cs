using TeacupHost.Core.Input;
using TeacupHost.Core.Printing;
using TeacupHost.Services;

namespace TeacupHost.ViewModels;

/// <summary>
/// Manual control by keyboard and gamepad: the settings and the state shown in the Axes tab,
/// and the actions that <see cref="InputController"/> calls.
/// </summary>
public sealed partial class MainViewModel
{
    public bool KeyboardControl
    {
        get => _settings.KeyboardControl;
        set
        {
            if (_settings.KeyboardControl == value)
                return;
            _settings.KeyboardControl = value;
            OnPropertyChanged();
        }
    }

    public bool GamepadControl
    {
        get => _settings.GamepadControl;
        set
        {
            if (_settings.GamepadControl == value)
                return;
            _settings.GamepadControl = value;
            OnPropertyChanged();
            OnPropertyChanged(nameof(GamepadStatus));
        }
    }

    private string? _gamepadName;

    /// <summary>"Gamepad: Xbox / XInput #1", "Gamepad: not found" or "Gamepad: off".</summary>
    public string GamepadStatus =>
        !GamepadControl ? Loc.T("S.Pad.Off") :
        _gamepadName == null ? Loc.T("S.Pad.None") :
        Loc.F("S.Pad.Connected", _gamepadName);

    /// <summary>Name of the connected gamepad, null if there is none.</summary>
    public void SetGamepadName(string? name)
    {
        if (_gamepadName == name)
            return;
        _gamepadName = name;
        OnPropertyChanged(nameof(GamepadStatus));
    }

    /// <summary>Step and feed rates as set in the Axes tab.</summary>
    public JogSettings CurrentJogSettings => new(JogStep, JogFeedXY, JogFeedZ);

    /// <summary>Commands waiting to be written to the printer.</summary>
    public int QueuedCommands => _conn.QueuedCommands;

    /// <summary>Send a move made by a key or a stick. Chunks of a held key skip the position request.</summary>
    public void JogBy(JogMove move)
    {
        if (CanControl)
            SendJog(move.X, move.Y, move.Z, move.Feed, poll: !move.IsChunk);
    }

    /// <summary>Update the position display after a held key was released.</summary>
    public void RefreshPositionAfterJog()
    {
        if (IsOnline)
            _conn.Send("M114", SendKind.Poll);
    }

    /// <summary>The next (+1) or previous (-1) step of the list.</summary>
    public void ChangeJogStep(int delta) => JogStep = JogStepPicker.Next(JogSteps, JogStep, delta);

    /// <param name="axes">"" for all, or the letters of the axes: "XY", "Z".</param>
    public void HomeFromInput(string axes)
    {
        if (HomeCommand.CanExecute(axes))
            HomeCommand.Execute(axes);
    }

    public void MotorsOffFromInput()
    {
        if (MotorsOffCommand.CanExecute(null))
            MotorsOffCommand.Execute(null);
    }

    /// <param name="direction">+1 extrude, -1 retract; the length and feed are those of the Axes tab.</param>
    public void ExtrudeFromInput(int direction)
    {
        if (CanControl)
            Extrude(direction * ExtrudeLength, interactive: false);
    }
}
