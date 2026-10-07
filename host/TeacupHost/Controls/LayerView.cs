using System.Globalization;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using TeacupHost.Core.GCode;

namespace TeacupHost.Controls;

/// <summary>
/// Top view of one layer: the layer being printed (or the highest visible
/// one), the layer below it dimmed, travel moves dashed and the nozzle.
/// Tracks are drawn with their real width. Wheel zooms at the cursor, drag
/// pans, double click fits the bed.
/// </summary>
public sealed class LayerView : FrameworkElement
{
    public static readonly DependencyProperty ToolpathProperty = DependencyProperty.Register(
        nameof(Toolpath), typeof(Toolpath), typeof(LayerView),
        new FrameworkPropertyMetadata(null, FrameworkPropertyMetadataOptions.AffectsRender, (d, e) =>
        {
            var v = (LayerView)d;
            v._cacheKey = default;
            v._fitModelPending = true;          // New file: zoom to the model.
        }));

    public static readonly DependencyProperty MaxLayerProperty = DependencyProperty.Register(
        nameof(MaxLayer), typeof(int), typeof(LayerView),
        new FrameworkPropertyMetadata(int.MaxValue, FrameworkPropertyMetadataOptions.AffectsRender, OnDataChanged));

    public static readonly DependencyProperty SegmentLimitProperty = DependencyProperty.Register(
        nameof(SegmentLimit), typeof(int), typeof(LayerView),
        new FrameworkPropertyMetadata(int.MaxValue, FrameworkPropertyMetadataOptions.AffectsRender, OnDataChanged));

    public static readonly DependencyProperty ShowTravelProperty = DependencyProperty.Register(
        nameof(ShowTravel), typeof(bool), typeof(LayerView),
        new FrameworkPropertyMetadata(true, FrameworkPropertyMetadataOptions.AffectsRender, OnDataChanged));

    public static readonly DependencyProperty NozzlePositionProperty = DependencyProperty.Register(
        nameof(NozzlePosition), typeof(Point3D), typeof(LayerView),
        new FrameworkPropertyMetadata(new Point3D(), FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty ShowNozzleProperty = DependencyProperty.Register(
        nameof(ShowNozzle), typeof(bool), typeof(LayerView),
        new FrameworkPropertyMetadata(false, FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty BedWidthProperty = DependencyProperty.Register(
        nameof(BedWidth), typeof(double), typeof(LayerView),
        new FrameworkPropertyMetadata(220.0, FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty BedDepthProperty = DependencyProperty.Register(
        nameof(BedDepth), typeof(double), typeof(LayerView),
        new FrameworkPropertyMetadata(180.0, FrameworkPropertyMetadataOptions.AffectsRender));

    public Toolpath? Toolpath
    {
        get => (Toolpath?)GetValue(ToolpathProperty);
        set => SetValue(ToolpathProperty, value);
    }

    public int MaxLayer
    {
        get => (int)GetValue(MaxLayerProperty);
        set => SetValue(MaxLayerProperty, value);
    }

    public int SegmentLimit
    {
        get => (int)GetValue(SegmentLimitProperty);
        set => SetValue(SegmentLimitProperty, value);
    }

    public bool ShowTravel
    {
        get => (bool)GetValue(ShowTravelProperty);
        set => SetValue(ShowTravelProperty, value);
    }

    public Point3D NozzlePosition
    {
        get => (Point3D)GetValue(NozzlePositionProperty);
        set => SetValue(NozzlePositionProperty, value);
    }

    public bool ShowNozzle
    {
        get => (bool)GetValue(ShowNozzleProperty);
        set => SetValue(ShowNozzleProperty, value);
    }

    public double BedWidth
    {
        get => (double)GetValue(BedWidthProperty);
        set => SetValue(BedWidthProperty, value);
    }

    public double BedDepth
    {
        get => (double)GetValue(BedDepthProperty);
        set => SetValue(BedDepthProperty, value);
    }

    private static void OnDataChanged(DependencyObject d, DependencyPropertyChangedEventArgs e) =>
        ((LayerView)d)._cacheKey = default;

    // View transform: screen = offset + world * scale, Y up.
    private double _scale = 3;
    private Vector _offset;
    private bool _fitted;
    private bool _fitModelPending;
    private Point _dragStart;
    private Vector _offsetStart;
    private bool _dragging;

    // Geometry cache of the shown layer.
    private (Toolpath? Tp, int Layer, int Limit, bool Travel) _cacheKey;
    private StreamGeometry?[] _featureGeometry = [];
    private StreamGeometry? _travelGeometry;
    private StreamGeometry? _belowGeometry;
    private int _shownLayer = -1;

    private static readonly Brush Background = Freeze(new SolidColorBrush(Color.FromRgb(0x1C, 0x1E, 0x22)));
    private static readonly Brush BedBrush = Freeze(new SolidColorBrush(Color.FromRgb(0x26, 0x29, 0x2E)));
    private static readonly Brush GridBrush = Freeze(new SolidColorBrush(Color.FromRgb(0x34, 0x38, 0x3E)));
    private static readonly Brush GridMajorBrush = Freeze(new SolidColorBrush(Color.FromRgb(0x4A, 0x50, 0x58)));
    private static readonly Brush BelowBrush = Freeze(new SolidColorBrush(Color.FromArgb(0x60, 0x9E, 0xA7, 0xB3)));
    private static readonly Brush TextBrush = Freeze(new SolidColorBrush(Color.FromRgb(0x9E, 0xA7, 0xB3)));

    private static T Freeze<T>(T f) where T : Freezable
    {
        f.Freeze();
        return f;
    }

    public LayerView()
    {
        ClipToBounds = true;
        Focusable = true;
    }

    public void FitBed()
    {
        FitBedCore();
        InvalidateVisual();
    }

    private void FitBedCore()
    {
        if (ActualWidth <= 0 || ActualHeight <= 0)
            return;
        double margin = 24;
        _scale = Math.Min((ActualWidth - 2 * margin) / BedWidth, (ActualHeight - 2 * margin) / BedDepth);
        _scale = Math.Max(_scale, 0.1);
        _offset = new Vector((ActualWidth - BedWidth * _scale) / 2, (ActualHeight + BedDepth * _scale) / 2);
        _fitted = true;
    }

    public void FitModel()
    {
        FitModelCore();
        InvalidateVisual();
    }

    private void FitModelCore()
    {
        var tp = Toolpath;
        if (ActualWidth <= 0 || ActualHeight <= 0)
            return;
        if (tp == null || tp.Max.X <= tp.Min.X)
        {
            FitBedCore();
            return;
        }
        // Room for the view switch at the top and the legend at the bottom.
        double margin = 70;
        double w = Math.Max(tp.Max.X - tp.Min.X, 10), h = Math.Max(tp.Max.Y - tp.Min.Y, 10);
        _scale = Math.Max(0.1, Math.Min((ActualWidth - 2 * margin) / w, (ActualHeight - 2 * margin) / h));
        double cx = (tp.Min.X + tp.Max.X) / 2, cy = (tp.Min.Y + tp.Max.Y) / 2;
        _offset = new Vector(ActualWidth / 2 - cx * _scale, ActualHeight / 2 + cy * _scale);
        _fitted = true;
    }

    protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
    {
        base.OnRenderSizeChanged(sizeInfo);
        if (!_fitted)
            FitBed();
    }

    protected override void OnMouseWheel(MouseWheelEventArgs e)
    {
        base.OnMouseWheel(e);
        var p = e.GetPosition(this);
        double k = e.Delta > 0 ? 1.2 : 1 / 1.2;
        double newScale = Math.Clamp(_scale * k, 0.2, 400);
        k = newScale / _scale;
        _offset = new Vector(p.X - (p.X - _offset.X) * k, p.Y - (p.Y - _offset.Y) * k);
        _scale = newScale;
        _fitted = true;
        InvalidateVisual();
        e.Handled = true;
    }

    protected override void OnMouseDown(MouseButtonEventArgs e)
    {
        base.OnMouseDown(e);
        Focus();
        if (e.ClickCount == 2)
        {
            FitBed();
            return;
        }
        _dragging = true;
        _dragStart = e.GetPosition(this);
        _offsetStart = _offset;
        CaptureMouse();
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (!_dragging)
            return;
        _offset = _offsetStart + (e.GetPosition(this) - _dragStart);
        InvalidateVisual();
    }

    protected override void OnMouseUp(MouseButtonEventArgs e)
    {
        base.OnMouseUp(e);
        _dragging = false;
        ReleaseMouseCapture();
    }

    private void UpdateCache()
    {
        var tp = Toolpath;
        if (tp == null || tp.Layers.Count == 0)
        {
            _featureGeometry = [];
            _travelGeometry = null;
            _belowGeometry = null;
            _shownLayer = -1;
            return;
        }
        int total = tp.Segments.Length;
        int limit = Math.Clamp(SegmentLimit, 0, total);
        int layer = Math.Clamp(MaxLayer, 0, tp.Layers.Count - 1);
        if (limit < total && limit > 0)
            layer = Math.Min(layer, tp.Segments[limit - 1].Layer);
        var key = (tp, layer, limit, ShowTravel);
        if (key == _cacheKey)
            return;
        _cacheKey = key;
        _shownLayer = layer;

        var l = tp.Layers[layer];
        int end = Math.Min(l.EndSegment, limit);
        var figures = new StreamGeometryContext?[FeaturePalette.Count];
        var geometries = new StreamGeometry?[FeaturePalette.Count];
        var travel = ShowTravel ? new StreamGeometry() : null;
        StreamGeometryContext? travelCtx = travel?.Open();

        for (int i = l.FirstSegment; i < end; i++)
        {
            ref readonly var s = ref tp.Segments[i];
            var a = new Point(s.Start.X, s.Start.Y);
            var b = new Point(s.End.X, s.End.Y);
            if (s.Kind == MoveKind.Travel)
            {
                if (travelCtx != null)
                {
                    travelCtx.BeginFigure(a, false, false);
                    travelCtx.LineTo(b, true, false);
                }
                continue;
            }
            int f = (int)s.Feature;
            if (figures[f] == null)
            {
                geometries[f] = new StreamGeometry();
                figures[f] = geometries[f]!.Open();
            }
            figures[f]!.BeginFigure(a, false, false);
            figures[f]!.LineTo(b, true, true);
        }
        for (int f = 0; f < figures.Length; f++)
        {
            if (figures[f] == null)
                continue;
            figures[f]!.Close();
            geometries[f]!.Freeze();
        }
        if (travelCtx != null)
        {
            travelCtx.Close();
            travel!.Freeze();
        }
        _featureGeometry = geometries;
        _travelGeometry = travel;

        _belowGeometry = null;
        if (layer > 0)
        {
            var below = tp.Layers[layer - 1];
            var g = new StreamGeometry();
            using (var ctx = g.Open())
            {
                for (int i = below.FirstSegment; i < below.EndSegment; i++)
                {
                    ref readonly var s = ref tp.Segments[i];
                    if (s.Kind != MoveKind.Extrude)
                        continue;
                    ctx.BeginFigure(new Point(s.Start.X, s.Start.Y), false, false);
                    ctx.LineTo(new Point(s.End.X, s.End.Y), true, true);
                }
            }
            g.Freeze();
            _belowGeometry = g;
        }
    }

    protected override void OnRender(DrawingContext dc)
    {
        if (_fitModelPending && ActualWidth > 0 && ActualHeight > 0)
        {
            _fitModelPending = false;
            FitModelCore();
        }
        dc.DrawRectangle(Background, null, new Rect(RenderSize));
        UpdateCache();

        var world = new MatrixTransform(_scale, 0, 0, -_scale, _offset.X, _offset.Y);
        world.Freeze();
        double px = 1 / _scale;                 // One pixel in mm.

        dc.PushTransform(world);
        dc.DrawRectangle(BedBrush, null, new Rect(0, 0, BedWidth, BedDepth));
        var minor = new Pen(GridBrush, px);
        var major = new Pen(GridMajorBrush, px * 1.5);
        for (int x = 0; x <= (int)BedWidth; x += 10)
            dc.DrawLine(x % 50 == 0 ? major : minor, new Point(x, 0), new Point(x, BedDepth));
        for (int y = 0; y <= (int)BedDepth; y += 10)
            dc.DrawLine(y % 50 == 0 ? major : minor, new Point(0, y), new Point(BedWidth, y));

        double width = Math.Max(ToolpathMesh.DefaultWidth, px);
        if (_belowGeometry != null)
            dc.DrawGeometry(null, new Pen(BelowBrush, width) { StartLineCap = PenLineCap.Round, EndLineCap = PenLineCap.Round }, _belowGeometry);
        for (int f = 0; f < _featureGeometry.Length; f++)
        {
            var g = _featureGeometry[f];
            if (g == null)
                continue;
            var pen = new Pen(FeaturePalette.BrushOf((FeatureType)f), width)
            {
                StartLineCap = PenLineCap.Round,
                EndLineCap = PenLineCap.Round,
                LineJoin = PenLineJoin.Round,
            };
            dc.DrawGeometry(null, pen, g);
        }
        if (_travelGeometry != null)
        {
            var pen = new Pen(FeaturePalette.BrushOf(FeatureType.Travel), px)
            {
                DashStyle = new DashStyle(new double[] { 4, 3 }, 0),
            };
            dc.DrawGeometry(null, pen, _travelGeometry);
        }
        dc.Pop();

        if (ShowNozzle)
        {
            var p = world.Transform(new Point(NozzlePosition.X, NozzlePosition.Y));
            var ring = new Pen(Brushes.White, 1.5);
            dc.DrawEllipse(null, ring, p, 7, 7);
            dc.DrawLine(ring, new Point(p.X - 11, p.Y), new Point(p.X - 4, p.Y));
            dc.DrawLine(ring, new Point(p.X + 4, p.Y), new Point(p.X + 11, p.Y));
            dc.DrawLine(ring, new Point(p.X, p.Y - 11), new Point(p.X, p.Y - 4));
            dc.DrawLine(ring, new Point(p.X, p.Y + 4), new Point(p.X, p.Y + 11));
        }

        var tp = Toolpath;
        if (tp != null && _shownLayer >= 0 && _shownLayer < tp.Layers.Count)
        {
            var l = tp.Layers[_shownLayer];
            var text = new FormattedText(
                string.Format(CultureInfo.CurrentCulture, "Слой {0} / {1}   Z = {2:0.###} мм   h = {3:0.###} мм",
                    _shownLayer + 1, tp.Layers.Count, l.Z, l.Height),
                CultureInfo.CurrentCulture, FlowDirection.LeftToRight, new Typeface("Segoe UI"), 12, TextBrush,
                VisualTreeHelper.GetDpi(this).PixelsPerDip);
            // Bottom right: the top is taken by the view switch and the current command.
            dc.DrawText(text, new Point(Math.Max(10, ActualWidth - text.Width - 12), ActualHeight - text.Height - 10));
        }
    }
}
