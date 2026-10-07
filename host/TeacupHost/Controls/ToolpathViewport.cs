using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using TeacupHost.Core.GCode;

namespace TeacupHost.Controls;

/// <summary>
/// 3D view of a toolpath on the printer bed. Layers are meshed in the
/// background, one model per layer; the layer being printed (simulation or
/// live print) is meshed again up to the current move. Left mouse button
/// rotates, right or middle button pans, the wheel zooms, a double click
/// resets the view.
/// </summary>
public sealed class ToolpathViewport : Border
{
    // ---------------------------------------------------------------- properties

    public static readonly DependencyProperty ToolpathProperty = DependencyProperty.Register(
        nameof(Toolpath), typeof(Toolpath), typeof(ToolpathViewport),
        new PropertyMetadata(null, (d, _) => ((ToolpathViewport)d).OnToolpathChanged()));

    public static readonly DependencyProperty MaxLayerProperty = DependencyProperty.Register(
        nameof(MaxLayer), typeof(int), typeof(ToolpathViewport),
        new PropertyMetadata(int.MaxValue, OnViewChanged));

    public static readonly DependencyProperty SegmentLimitProperty = DependencyProperty.Register(
        nameof(SegmentLimit), typeof(int), typeof(ToolpathViewport),
        new PropertyMetadata(int.MaxValue, OnViewChanged));

    public static readonly DependencyProperty ShowTravelProperty = DependencyProperty.Register(
        nameof(ShowTravel), typeof(bool), typeof(ToolpathViewport),
        new PropertyMetadata(false, OnViewChanged));

    public static readonly DependencyProperty NozzlePositionProperty = DependencyProperty.Register(
        nameof(NozzlePosition), typeof(Point3D), typeof(ToolpathViewport),
        new PropertyMetadata(new Point3D(), (d, _) => ((ToolpathViewport)d).UpdateNozzle()));

    public static readonly DependencyProperty ShowNozzleProperty = DependencyProperty.Register(
        nameof(ShowNozzle), typeof(bool), typeof(ToolpathViewport),
        new PropertyMetadata(false, (d, _) => ((ToolpathViewport)d).UpdateNozzle()));

    public static readonly DependencyProperty BedWidthProperty = DependencyProperty.Register(
        nameof(BedWidth), typeof(double), typeof(ToolpathViewport),
        new PropertyMetadata(220.0, (d, _) => ((ToolpathViewport)d).BuildBed()));

    public static readonly DependencyProperty BedDepthProperty = DependencyProperty.Register(
        nameof(BedDepth), typeof(double), typeof(ToolpathViewport),
        new PropertyMetadata(180.0, (d, _) => ((ToolpathViewport)d).BuildBed()));

    public static readonly DependencyProperty FilamentDiameterProperty = DependencyProperty.Register(
        nameof(FilamentDiameter), typeof(double), typeof(ToolpathViewport),
        new PropertyMetadata(1.75));

    public Toolpath? Toolpath
    {
        get => (Toolpath?)GetValue(ToolpathProperty);
        set => SetValue(ToolpathProperty, value);
    }

    /// <summary>Highest layer shown (zero-based).</summary>
    public int MaxLayer
    {
        get => (int)GetValue(MaxLayerProperty);
        set => SetValue(MaxLayerProperty, value);
    }

    /// <summary>Only segments with a smaller index are shown (simulation / print progress).</summary>
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

    public double FilamentDiameter
    {
        get => (double)GetValue(FilamentDiameterProperty);
        set => SetValue(FilamentDiameterProperty, value);
    }

    private static void OnViewChanged(DependencyObject d, DependencyPropertyChangedEventArgs e) =>
        ((ToolpathViewport)d).ScheduleUpdate();

    // ---------------------------------------------------------------- scene

    private readonly Viewport3D _viewport = new() { ClipToBounds = true, IsHitTestVisible = false };
    private readonly PerspectiveCamera _camera = new() { FieldOfView = 40, NearPlaneDistance = 1, FarPlaneDistance = 20000, UpDirection = new Vector3D(0, 0, 1) };
    private readonly Model3DGroup _bedGroup = new();
    private readonly Model3DGroup _extrudeGroup = new();
    private readonly Model3DGroup _travelGroup = new();
    private readonly Model3DGroup _partialGroup = new();
    private readonly GeometryModel3D _nozzle;
    private readonly TranslateTransform3D _nozzleTransform = new();
    private readonly Material _pathMaterial;

    private GeometryModel3D?[] _layerExtrude = [];
    private GeometryModel3D?[] _layerTravel = [];
    private int _buildVersion;
    private CancellationTokenSource? _buildCts;

    private int _shownExtrude = -1, _shownTravel = -1, _shownVersion = -1;
    private (int Layer, int Limit, bool Travel, Toolpath? Tp) _partialKey = (-1, -1, false, null);
    private bool _updatePending;

    // Orbit camera.
    private Point3D _target = new(110, 90, 0);
    private double _yaw = -60, _pitch = 35, _distance = 350;
    private Point _lastMouse;
    private MouseButton? _dragButton;

    public ToolpathViewport()
    {
        Background = Brushes.Transparent;
        ClipToBounds = true;
        Focusable = true;

        var texture = FeaturePalette.CreateTexture();
        var brush = new ImageBrush(texture)
        {
            ViewportUnits = BrushMappingMode.Absolute,
            Viewport = new Rect(0, 0, 1, 1),
            Stretch = Stretch.Fill,
        };
        RenderOptions.SetBitmapScalingMode(brush, BitmapScalingMode.NearestNeighbor);
        brush.Freeze();
        var group = new MaterialGroup();
        group.Children.Add(new DiffuseMaterial(brush));
        group.Children.Add(new SpecularMaterial(new SolidColorBrush(Color.FromArgb(60, 255, 255, 255)), 30));
        group.Freeze();
        _pathMaterial = group;

        var nozzleMaterial = ToolpathMesh.Solid(Color.FromRgb(0xE0, 0xE6, 0xEA), 40);
        _nozzle = new GeometryModel3D(ToolpathMesh.CreateNozzle(), nozzleMaterial)
        {
            BackMaterial = nozzleMaterial,
            Transform = _nozzleTransform,
        };

        var lights = new Model3DGroup();
        lights.Children.Add(new AmbientLight(Color.FromRgb(0x5A, 0x5A, 0x5A)));
        lights.Children.Add(new DirectionalLight(Color.FromRgb(0xB0, 0xB0, 0xB0), new Vector3D(-0.6, -0.8, -1.4)));
        lights.Children.Add(new DirectionalLight(Color.FromRgb(0x50, 0x50, 0x58), new Vector3D(0.8, 0.6, -0.4)));

        var root = new Model3DGroup();
        root.Children.Add(lights);
        root.Children.Add(_bedGroup);
        root.Children.Add(_extrudeGroup);
        root.Children.Add(_travelGroup);
        root.Children.Add(_partialGroup);
        _viewport.Camera = _camera;
        _viewport.Children.Add(new ModelVisual3D { Content = root });
        Child = _viewport;

        BuildBed();
        ResetView();
        ViewColors.Follow(this, BuildBed);
    }

    // ---------------------------------------------------------------- camera

    /// <summary>Look at the model (or the bed) from the front right.</summary>
    public void ResetView() => SetView(-60, 35);

    public void ViewTop() => SetView(-90, 89);

    public void ViewFront() => SetView(-90, 8);

    public void ViewIso() => SetView(-45, 35);

    private void SetView(double yaw, double pitch)
    {
        var tp = Toolpath;
        double size;
        if (tp != null && tp.Segments.Length > 0 && tp.Max.X > tp.Min.X)
        {
            _target = new Point3D((tp.Min.X + tp.Max.X) / 2, (tp.Min.Y + tp.Max.Y) / 2, (tp.Min.Z + tp.Max.Z) / 2);
            size = Math.Max(Math.Max(tp.Max.X - tp.Min.X, tp.Max.Y - tp.Min.Y), tp.Max.Z - tp.Min.Z);
            size = Math.Max(size, 20);
            _distance = size * 2.4;
        }
        else
        {
            _target = new Point3D(BedWidth / 2, BedDepth / 2, 0);
            size = Math.Max(BedWidth, BedDepth);
            _distance = size * 1.6;
        }
        _yaw = yaw;
        _pitch = pitch;
        UpdateCamera();
    }

    /// <summary>Show the whole bed.</summary>
    public void ViewBed()
    {
        _target = new Point3D(BedWidth / 2, BedDepth / 2, 0);
        _distance = Math.Max(BedWidth, BedDepth) * 1.6;
        UpdateCamera();
    }

    private void UpdateCamera()
    {
        double yaw = _yaw * Math.PI / 180, pitch = _pitch * Math.PI / 180;
        var dir = new Vector3D(Math.Cos(pitch) * Math.Cos(yaw), Math.Cos(pitch) * Math.Sin(yaw), Math.Sin(pitch));
        _camera.Position = _target + dir * _distance;
        _camera.LookDirection = -dir * _distance;
        _camera.NearPlaneDistance = Math.Max(0.1, _distance / 200);
    }

    protected override void OnMouseDown(MouseButtonEventArgs e)
    {
        base.OnMouseDown(e);
        Focus();
        if (e.ClickCount == 2 && e.ChangedButton == MouseButton.Left)
        {
            ResetView();
            return;
        }
        _dragButton = e.ChangedButton;
        _lastMouse = e.GetPosition(this);
        CaptureMouse();
        e.Handled = true;
    }

    protected override void OnMouseUp(MouseButtonEventArgs e)
    {
        base.OnMouseUp(e);
        if (_dragButton == e.ChangedButton)
        {
            _dragButton = null;
            ReleaseMouseCapture();
        }
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (_dragButton == null)
            return;
        var p = e.GetPosition(this);
        var d = p - _lastMouse;
        _lastMouse = p;
        if (_dragButton == MouseButton.Left && Keyboard.Modifiers != ModifierKeys.Shift)
        {
            _yaw -= d.X * 0.4;
            _pitch = Math.Clamp(_pitch + d.Y * 0.4, -89, 89);
        }
        else
        {
            // Pan in the camera plane.
            var look = _camera.LookDirection;
            look.Normalize();
            var right = Vector3D.CrossProduct(look, new Vector3D(0, 0, 1));
            if (right.Length < 1e-6)
                right = new Vector3D(1, 0, 0);
            right.Normalize();
            var up = Vector3D.CrossProduct(right, look);
            double k = _distance * 0.0016;
            _target += (-right * d.X + up * d.Y) * k;
        }
        UpdateCamera();
    }

    protected override void OnMouseWheel(MouseWheelEventArgs e)
    {
        base.OnMouseWheel(e);
        _distance = Math.Clamp(_distance * (e.Delta > 0 ? 0.87 : 1.15), 5, 5000);
        UpdateCamera();
        e.Handled = true;
    }

    // ---------------------------------------------------------------- bed

    private void BuildBed()
    {
        _bedGroup.Children.Clear();
        double w = BedWidth, d = BedDepth;

        var plate = new MeshGeometry3D();
        ToolpathMesh.AddLine(plate, 0, d / 2, w, d / 2, -0.06, d);
        var plateMat = ToolpathMesh.Solid(ViewColors.Bed);
        _bedGroup.Children.Add(new GeometryModel3D(plate, plateMat) { BackMaterial = plateMat });

        var minor = new MeshGeometry3D();
        var major = new MeshGeometry3D();
        for (int x = 0; x <= (int)w; x += 10)
            ToolpathMesh.AddLine(x % 50 == 0 ? major : minor, x, 0, x, d, -0.02, x % 50 == 0 ? 0.45 : 0.25);
        for (int y = 0; y <= (int)d; y += 10)
            ToolpathMesh.AddLine(y % 50 == 0 ? major : minor, 0, y, w, y, -0.02, y % 50 == 0 ? 0.45 : 0.25);
        // Bed outline.
        ToolpathMesh.AddLine(major, 0, 0, w, 0, -0.01, 0.8);
        ToolpathMesh.AddLine(major, w, 0, w, d, -0.01, 0.8);
        ToolpathMesh.AddLine(major, w, d, 0, d, -0.01, 0.8);
        ToolpathMesh.AddLine(major, 0, d, 0, 0, -0.01, 0.8);
        _bedGroup.Children.Add(new GeometryModel3D(minor, ToolpathMesh.Solid(ViewColors.Grid)));
        _bedGroup.Children.Add(new GeometryModel3D(major, ToolpathMesh.Solid(ViewColors.GridMajor)));

        // Axes at the origin: X red, Y green, Z blue.
        AddAxis(new Point3D(0, -0.6, 0), new Point3D(25, 0.6, 1.2), Color.FromRgb(0xE5, 0x39, 0x35));
        AddAxis(new Point3D(-0.6, 0, 0), new Point3D(0.6, 25, 1.2), Color.FromRgb(0x43, 0xA0, 0x47));
        AddAxis(new Point3D(-0.6, -0.6, 0), new Point3D(0.6, 0.6, 25), Color.FromRgb(0x1E, 0x88, 0xE5));

        if (ShowNozzle)
            _bedGroup.Children.Add(_nozzle);
    }

    private void AddAxis(Point3D a, Point3D b, Color c)
    {
        var mesh = new MeshGeometry3D();
        ToolpathMesh.AddBox(mesh, a, b);
        var m = ToolpathMesh.Solid(c);
        _bedGroup.Children.Add(new GeometryModel3D(mesh, m) { BackMaterial = m });
    }

    private void UpdateNozzle()
    {
        var p = NozzlePosition;
        _nozzleTransform.OffsetX = p.X;
        _nozzleTransform.OffsetY = p.Y;
        _nozzleTransform.OffsetZ = p.Z;
        bool inScene = _bedGroup.Children.Contains(_nozzle);
        if (ShowNozzle && !inScene)
            _bedGroup.Children.Add(_nozzle);
        else if (!ShowNozzle && inScene)
            _bedGroup.Children.Remove(_nozzle);
    }

    // ---------------------------------------------------------------- toolpath

    private void OnToolpathChanged()
    {
        _buildCts?.Cancel();
        _buildCts = null;
        _extrudeGroup.Children.Clear();
        _travelGroup.Children.Clear();
        _partialGroup.Children.Clear();
        _partialKey = (-1, -1, false, null);
        _shownExtrude = _shownTravel = -1;
        _buildVersion++;

        var tp = Toolpath;
        if (tp == null || tp.Layers.Count == 0)
        {
            _layerExtrude = [];
            _layerTravel = [];
            ResetView();
            return;
        }
        _layerExtrude = new GeometryModel3D?[tp.Layers.Count];
        _layerTravel = new GeometryModel3D?[tp.Layers.Count];
        ResetView();

        var cts = new CancellationTokenSource();
        _buildCts = cts;
        double area = Math.PI * FilamentDiameter * FilamentDiameter / 4;
        var material = _pathMaterial;
        var dispatcher = Dispatcher;
        int version = _buildVersion;
        Task.Run(() => BuildLayers(tp, area, material, dispatcher, version, cts.Token));
    }

    private void BuildLayers(Toolpath tp, double area, Material material, Dispatcher dispatcher, int version,
        CancellationToken cancel)
    {
        var batch = new List<(int Layer, GeometryModel3D? Extrude, GeometryModel3D? Travel)>();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        foreach (var layer in tp.Layers)
        {
            if (cancel.IsCancellationRequested)
                return;
            batch.Add((layer.Index,
                Model(ToolpathMesh.Build(tp, layer.FirstSegment, layer.EndSegment, false, area), material),
                Model(ToolpathMesh.Build(tp, layer.FirstSegment, layer.EndSegment, true, area), material)));
            if (watch.ElapsedMilliseconds > 150 || layer.Index == tp.Layers.Count - 1)
            {
                var items = batch.ToArray();
                batch.Clear();
                watch.Restart();
                dispatcher.InvokeAsync(() =>
                {
                    if (version != _buildVersion)
                        return;
                    foreach (var (i, ex, tr) in items)
                    {
                        _layerExtrude[i] = ex;
                        _layerTravel[i] = tr;
                    }
                    _shownVersion = -1;            // Models changed, resync the groups.
                    ScheduleUpdate();
                }, DispatcherPriority.Background);
            }
        }
    }

    private static GeometryModel3D? Model(MeshGeometry3D? mesh, Material material)
    {
        if (mesh == null)
            return null;
        var m = new GeometryModel3D(mesh, material) { BackMaterial = material };
        m.Freeze();
        return m;
    }

    private void ScheduleUpdate()
    {
        if (_updatePending)
            return;
        _updatePending = true;
        Dispatcher.InvokeAsync(() =>
        {
            _updatePending = false;
            UpdateVisibility();
        }, DispatcherPriority.Render);
    }

    private void UpdateVisibility()
    {
        var tp = Toolpath;
        if (tp == null || tp.Layers.Count == 0)
            return;

        int total = tp.Segments.Length;
        int limit = Math.Clamp(SegmentLimit, 0, total);
        int maxLayer = Math.Clamp(MaxLayer, 0, tp.Layers.Count - 1);
        int full;
        int partial = -1;
        if (limit < total)
        {
            int current = limit > 0 ? tp.Segments[limit - 1].Layer : 0;
            if (current <= maxLayer)
            {
                full = current;
                partial = limit > tp.Layers[current].FirstSegment ? current : -1;
                if (partial >= 0 && limit >= tp.Layers[current].EndSegment)
                {
                    full = current + 1;
                    partial = -1;
                }
            }
            else
            {
                full = maxLayer + 1;
            }
        }
        else
        {
            full = maxLayer + 1;
        }

        Sync(_extrudeGroup, _layerExtrude, full, ref _shownExtrude);
        Sync(_travelGroup, _layerTravel, ShowTravel ? full : 0, ref _shownTravel);
        _shownVersion = _buildVersion;

        // The layer being printed, up to the current move.
        var key = (partial, partial >= 0 ? limit : -1, ShowTravel, (Toolpath?)tp);
        if (key != _partialKey)
        {
            _partialKey = key;
            _partialGroup.Children.Clear();
            if (partial >= 0)
            {
                var layer = tp.Layers[partial];
                double area = Math.PI * FilamentDiameter * FilamentDiameter / 4;
                var ex = Model(ToolpathMesh.Build(tp, layer.FirstSegment, limit, false, area), _pathMaterial);
                if (ex != null)
                    _partialGroup.Children.Add(ex);
                if (ShowTravel)
                {
                    var tr = Model(ToolpathMesh.Build(tp, layer.FirstSegment, limit, true, area), _pathMaterial);
                    if (tr != null)
                        _partialGroup.Children.Add(tr);
                }
            }
        }
    }

    /// <summary>Make the group hold the models of layers [0, count).</summary>
    private void Sync(Model3DGroup group, GeometryModel3D?[] models, int count, ref int shown)
    {
        count = Math.Min(count, models.Length);
        if (count == shown && _shownVersion == _buildVersion)
            return;
        if (count > shown && shown >= 0 && _shownVersion == _buildVersion)
        {
            for (int i = shown; i < count; i++)
                if (models[i] != null)
                    group.Children.Add(models[i]!);
        }
        else if (count < shown && _shownVersion == _buildVersion)
        {
            // Layers without moves of this kind have no model: count the real ones.
            int keep = 0;
            for (int i = 0; i < count; i++)
                if (models[i] != null)
                    keep++;
            while (group.Children.Count > keep)
                group.Children.RemoveAt(group.Children.Count - 1);
        }
        else
        {
            var children = new Model3DCollection(count);
            for (int i = 0; i < count; i++)
                if (models[i] != null)
                    children.Add(models[i]!);
            group.Children = children;
        }
        shown = count;
    }
}
