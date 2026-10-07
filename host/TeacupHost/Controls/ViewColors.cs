using System.Windows;
using System.Windows.Media;
using TeacupHost.Services;

namespace TeacupHost.Controls;

/// <summary>Colors of the drawn views (bed, grid, labels) for the current theme.</summary>
internal static class ViewColors
{
    private sealed record Set(Color Background, Color Bed, Color Grid, Color GridMajor, Color Below, Color Text,
        Color ChartGrid, Color Nozzle);

    private static readonly Set Dark = new(
        Color.FromRgb(0x1C, 0x1E, 0x22), Color.FromRgb(0x26, 0x29, 0x2E), Color.FromRgb(0x38, 0x3C, 0x42),
        Color.FromRgb(0x50, 0x56, 0x5E), Color.FromArgb(0x60, 0x9E, 0xA7, 0xB3), Color.FromRgb(0x9E, 0xA7, 0xB3),
        Color.FromArgb(0x30, 0xFF, 0xFF, 0xFF), Colors.White);

    private static readonly Set Light = new(
        Color.FromRgb(0xEE, 0xF1, 0xF5), Color.FromRgb(0xD9, 0xDE, 0xE5), Color.FromRgb(0xC2, 0xC9, 0xD2),
        Color.FromRgb(0x9A, 0xA4, 0xB1), Color.FromArgb(0x70, 0x8A, 0x94, 0xA3), Color.FromRgb(0x4B, 0x55, 0x63),
        Color.FromArgb(0x30, 0x00, 0x00, 0x00), Color.FromRgb(0x1F, 0x29, 0x37));

    private static Set Current => ThemeService.IsDark ? Dark : Light;

    public static Color Background => Current.Background;
    public static Color Bed => Current.Bed;
    public static Color Grid => Current.Grid;
    public static Color GridMajor => Current.GridMajor;
    public static Color Below => Current.Below;
    public static Color Text => Current.Text;
    public static Color ChartGrid => Current.ChartGrid;
    public static Color Nozzle => Current.Nozzle;

    public static SolidColorBrush Brush(Color c)
    {
        var b = new SolidColorBrush(c);
        b.Freeze();
        return b;
    }

    /// <summary>Call <paramref name="refresh"/> on theme and language changes while the element is loaded.</summary>
    public static void Follow(FrameworkElement element, Action refresh)
    {
        void OnChange() => element.Dispatcher.BeginInvoke(refresh);
        element.Loaded += (_, _) =>
        {
            ThemeService.Changed += OnChange;
            Loc.Changed += OnChange;
            refresh();
        };
        element.Unloaded += (_, _) =>
        {
            ThemeService.Changed -= OnChange;
            Loc.Changed -= OnChange;
        };
    }
}
