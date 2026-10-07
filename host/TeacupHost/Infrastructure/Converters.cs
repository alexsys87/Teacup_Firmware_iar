using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace TeacupHost.Infrastructure;

/// <summary>true → Visible, false → Collapsed; "Invert" as parameter swaps them.</summary>
public sealed class BoolToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture)
    {
        bool v = value is true;
        if (parameter is "Invert")
            v = !v;
        return v ? Visibility.Visible : Visibility.Collapsed;
    }

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}

public sealed class InverseBoolConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) => value is not true;

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) => value is not true;
}

/// <summary>Null → Collapsed, anything else → Visible.</summary>
public sealed class NullToVisibilityConverter : IValueConverter
{
    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture) =>
        value == null || value is string { Length: 0 } ? Visibility.Collapsed : Visibility.Visible;

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}

/// <summary>View model severity → WPF UI InfoBar severity.</summary>
public sealed class SeverityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) => value switch
    {
        ViewModels.MainViewModel.NotifySeverity.Success => Wpf.Ui.Controls.InfoBarSeverity.Success,
        ViewModels.MainViewModel.NotifySeverity.Warning => Wpf.Ui.Controls.InfoBarSeverity.Warning,
        ViewModels.MainViewModel.NotifySeverity.Error => Wpf.Ui.Controls.InfoBarSeverity.Error,
        _ => Wpf.Ui.Controls.InfoBarSeverity.Informational,
    };

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}

/// <summary>Equality with the parameter, e.g. a jog step button is "checked" when it's the current step.</summary>
public sealed class EqualsConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) =>
        value != null && parameter != null &&
        string.Equals(System.Convert.ToString(value, CultureInfo.InvariantCulture),
            System.Convert.ToString(parameter, CultureInfo.InvariantCulture), StringComparison.OrdinalIgnoreCase);

    public object? ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        value is true && parameter != null
            ? System.Convert.ChangeType(parameter, targetType, CultureInfo.InvariantCulture)
            : Binding.DoNothing;
}

/// <summary>Temperature as "205.3°".</summary>
public sealed class TemperatureConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) =>
        value is double d ? d.ToString("0.0", culture) + "°" : "—";

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}

/// <summary>Visible when the value (an index) equals the parameter: content of a segmented tab bar.</summary>
public sealed class IndexToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) =>
        System.Convert.ToString(value, CultureInfo.InvariantCulture) ==
        System.Convert.ToString(parameter, CultureInfo.InvariantCulture)
            ? Visibility.Visible
            : Visibility.Collapsed;

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}

/// <summary>Port list item → shown name: the virtual printer in the current language.</summary>
public sealed class PortNameConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object? parameter, CultureInfo culture) =>
        value as string == ViewModels.MainViewModel.VirtualPortName
            ? Services.Loc.T("S.VirtualPrinter")
            : value ?? "";

    public object ConvertBack(object value, Type targetType, object? parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}
