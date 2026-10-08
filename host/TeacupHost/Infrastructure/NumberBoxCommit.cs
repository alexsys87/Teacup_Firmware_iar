using System.Globalization;
using System.Windows;
using System.Windows.Input;
using Wpf.Ui.Controls;

namespace TeacupHost.Infrastructure;

/// <summary>
/// WPF-UI 3.0.5 NumberBox takes typed text into Value only on Enter; when the
/// box loses focus it puts the old value back. So a number typed just before
/// clicking a button was silently lost. Here the text is committed whenever a
/// NumberBox loses keyboard focus, before the click runs its command.
/// </summary>
public static class NumberBoxCommit
{
    public static void Register() =>
        EventManager.RegisterClassHandler(typeof(NumberBox), UIElement.PreviewLostKeyboardFocusEvent,
            new KeyboardFocusChangedEventHandler(OnLosingFocus));

    private static void OnLosingFocus(object sender, KeyboardFocusChangedEventArgs e)
    {
        if (sender is not NumberBox box || !ReferenceEquals(e.OldFocus, box))
            return;
        string text = box.Text.Trim();
        // The user's culture first ("0,25"), then the dot that the boxes show by default.
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out double value) &&
            !double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            return;
        if (box.Value != value)
            box.Value = value;
    }
}
