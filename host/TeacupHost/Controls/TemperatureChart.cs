using System.Globalization;
using System.Windows;
using System.Windows.Media;

namespace TeacupHost.Controls;

public readonly record struct TemperatureSample(double Time, double Hotend, double HotendTarget, double Bed, double BedTarget);

/// <summary>Temperature history: hotend and bed, actual (solid) and target (dashed).</summary>
public sealed class TemperatureChart : FrameworkElement
{
    public static readonly DependencyProperty SamplesProperty = DependencyProperty.Register(
        nameof(Samples), typeof(IReadOnlyList<TemperatureSample>), typeof(TemperatureChart),
        new FrameworkPropertyMetadata(null, FrameworkPropertyMetadataOptions.AffectsRender));

    /// <summary>Change it to redraw after samples were added to the same list.</summary>
    public static readonly DependencyProperty RevisionProperty = DependencyProperty.Register(
        nameof(Revision), typeof(int), typeof(TemperatureChart),
        new FrameworkPropertyMetadata(0, FrameworkPropertyMetadataOptions.AffectsRender));

    /// <summary>Shown time span, s.</summary>
    public static readonly DependencyProperty SpanProperty = DependencyProperty.Register(
        nameof(Span), typeof(double), typeof(TemperatureChart),
        new FrameworkPropertyMetadata(300.0, FrameworkPropertyMetadataOptions.AffectsRender));

    public IReadOnlyList<TemperatureSample>? Samples
    {
        get => (IReadOnlyList<TemperatureSample>?)GetValue(SamplesProperty);
        set => SetValue(SamplesProperty, value);
    }

    public int Revision
    {
        get => (int)GetValue(RevisionProperty);
        set => SetValue(RevisionProperty, value);
    }

    public double Span
    {
        get => (double)GetValue(SpanProperty);
        set => SetValue(SpanProperty, value);
    }

    public static readonly Color HotendColor = Color.FromRgb(0xFF, 0x70, 0x43);
    public static readonly Color BedColor = Color.FromRgb(0x42, 0xA5, 0xF5);

    private static readonly Pen GridPen = Frozen(new Pen(new SolidColorBrush(Color.FromArgb(0x30, 0xFF, 0xFF, 0xFF)), 1));
    private static readonly Brush LabelBrush = Frozen(new SolidColorBrush(Color.FromRgb(0x9E, 0xA7, 0xB3)));
    private static readonly Pen HotendPen = Frozen(new Pen(new SolidColorBrush(HotendColor), 2));
    private static readonly Pen BedPen = Frozen(new Pen(new SolidColorBrush(BedColor), 2));
    private static readonly Pen HotendTargetPen = Frozen(new Pen(new SolidColorBrush(HotendColor), 1) { DashStyle = DashStyles.Dash });
    private static readonly Pen BedTargetPen = Frozen(new Pen(new SolidColorBrush(BedColor), 1) { DashStyle = DashStyles.Dash });

    private static T Frozen<T>(T f) where T : Freezable
    {
        f.Freeze();
        return f;
    }

    protected override void OnRender(DrawingContext dc)
    {
        double w = ActualWidth, h = ActualHeight;
        dc.DrawRectangle(Brushes.Transparent, null, new Rect(0, 0, w, h));
        if (w < 60 || h < 40)
            return;

        var samples = Samples;
        double now = samples is { Count: > 0 } ? samples[^1].Time : 0;
        double t0 = now - Span;

        double max = 50;
        if (samples != null)
        {
            foreach (var s in samples)
            {
                if (s.Time < t0)
                    continue;
                max = Math.Max(max, Math.Max(Math.Max(s.Hotend, s.HotendTarget), Math.Max(s.Bed, s.BedTarget)));
            }
        }
        max = Math.Ceiling((max + 10) / 50) * 50;

        const double left = 34, bottom = 16, top = 6, right = 6;
        double pw = w - left - right, ph = h - top - bottom;
        double dpi = VisualTreeHelper.GetDpi(this).PixelsPerDip;
        var typeface = new Typeface("Segoe UI");

        double step = max > 200 ? 50 : 25;
        for (double v = 0; v <= max + 0.1; v += step)
        {
            double y = top + ph - v / max * ph;
            dc.DrawLine(GridPen, new Point(left, y), new Point(left + pw, y));
            var ft = new FormattedText(v.ToString("0", CultureInfo.InvariantCulture), CultureInfo.CurrentCulture,
                FlowDirection.LeftToRight, typeface, 10, LabelBrush, dpi);
            dc.DrawText(ft, new Point(left - ft.Width - 4, y - ft.Height / 2));
        }
        for (int m = 0; m <= (int)(Span / 60); m++)
        {
            double x = left + pw - m * 60 / Span * pw;
            var ft = new FormattedText(m == 0 ? "сейчас" : $"-{m} мин", CultureInfo.CurrentCulture,
                FlowDirection.LeftToRight, typeface, 10, LabelBrush, dpi);
            dc.DrawText(ft, new Point(Math.Max(left, x - ft.Width / 2 - (m == 0 ? ft.Width / 2 : 0)), top + ph + 2));
        }

        if (samples == null || samples.Count < 2)
            return;

        Point P(double t, double v) => new(left + (t - t0) / Span * pw, top + ph - Math.Clamp(v, 0, max) / max * ph);

        void Series(Func<TemperatureSample, double> value, Pen pen)
        {
            var g = new StreamGeometry();
            using (var ctx = g.Open())
            {
                bool started = false;
                foreach (var s in samples)
                {
                    if (s.Time < t0)
                        continue;
                    var p = P(s.Time, value(s));
                    if (!started)
                    {
                        ctx.BeginFigure(p, false, false);
                        started = true;
                    }
                    else
                    {
                        ctx.LineTo(p, true, true);
                    }
                }
            }
            g.Freeze();
            dc.DrawGeometry(null, pen, g);
        }

        dc.PushClip(new RectangleGeometry(new Rect(left, top - 2, pw, ph + 4)));
        Series(s => s.BedTarget, BedTargetPen);
        Series(s => s.HotendTarget, HotendTargetPen);
        Series(s => s.Bed, BedPen);
        Series(s => s.Hotend, HotendPen);
        dc.Pop();
    }
}
