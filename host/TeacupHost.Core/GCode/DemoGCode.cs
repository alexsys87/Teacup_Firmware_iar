using System.Globalization;
using System.Text;

namespace TeacupHost.Core.GCode;

/// <summary>
/// Generates the classic calibration cube (10 × 10 × 10 mm by default):
/// skirt, two perimeters, four solid bottom and top layers and 20 % sparse
/// infill at ±45°. Used by the virtual printer and to try out the viewer
/// without a slicer. Comments follow PrusaSlicer (";TYPE:", ";LAYER_CHANGE").
/// </summary>
public static class DemoGCode
{
    private const double FilamentArea = Math.PI * 1.75 * 1.75 / 4;
    private const double Width = 0.45;
    private const int SolidLayers = 4;

    public static string Generate(double centerX = 110, double centerY = 90, double size = 10,
        double height = 10, double layerHeight = 0.2)
    {
        var g = new Writer(layerHeight);
        int layers = Math.Max(1, (int)Math.Round(height / layerHeight));
        double half = size / 2;
        double x0 = centerX - half, y0 = centerY - half, x1 = centerX + half, y1 = centerY + half;

        g.Line($"; Teacup Host calibration cube {g.F(size)} x {g.F(size)} x {g.F(height)} mm");
        g.Line($"; layer height {g.F(layerHeight)} mm, 2 perimeters, {SolidLayers} solid layers, 20% infill");
        g.Line("M140 S60");
        g.Line("M104 S205");
        g.Line("M190 S60");
        g.Line("M109 S205");
        g.Line("G28");
        g.Line("G90");
        g.Line("M82");
        g.Line("G92 E0");
        g.Line("G1 Z5 F240");

        for (int layer = 0; layer < layers; layer++)
        {
            double z = (layer + 1) * layerHeight;
            g.Line(";LAYER_CHANGE");
            g.Line($";Z:{g.F(z)}");
            g.Line($";HEIGHT:{g.F(layerHeight)}");
            if (layer > 0)
                g.Retract();
            g.Line($"G1 Z{g.F(z)} F240");
            if (layer == 1)
                g.Line("M106 S255");

            if (layer == 0)
            {
                // Skirt 3 mm around the cube, two loops.
                g.Line(";TYPE:Skirt");
                for (int loop = 0; loop < 2; loop++)
                {
                    double d = 3 + loop * Width;
                    Rectangle(g, x0 - d, y0 - d, x1 + d, y1 + d, 1800);
                }
            }

            // Perimeters: the inner one first, then the outer one (PrusaSlicer order).
            g.Line(";TYPE:Perimeter");
            double inset = Width * 1.5;
            Rectangle(g, x0 + inset, y0 + inset, x1 - inset, y1 - inset, 2400);
            g.Line(";TYPE:External perimeter");
            inset = Width / 2;
            Rectangle(g, x0 + inset, y0 + inset, x1 - inset, y1 - inset, 1500);

            // Infill inside the perimeters, ±45° by layer.
            bool bottom = layer < SolidLayers;
            bool top = layer >= layers - SolidLayers;
            double spacing = bottom || top ? Width : Width / 0.2;
            g.Line(bottom ? ";TYPE:Solid infill" : top ? ";TYPE:Top solid infill" : ";TYPE:Internal infill");
            double m = Width * 2 + Width * 0.15;
            Diagonals(g, x0 + m, y0 + m, x1 - m, y1 - m, spacing, layer % 2 == 0, bottom || top ? 2400 : 3600);
        }

        g.Retract();
        g.Line($"G1 Z{g.F(height + 10)} F240");
        g.Line("G0 X10 Y170 F6000");
        g.Line("M107");
        g.Line("M104 S0");
        g.Line("M140 S0");
        g.Line("M84");
        g.Line("; end of calibration cube");
        return g.ToString();
    }

    /// <summary>Closed rectangle, starting at the bottom left corner.</summary>
    private static void Rectangle(Writer g, double ax, double ay, double bx, double by, double feed)
    {
        g.TravelTo(ax, ay);
        g.ExtrudeTo(bx, ay, feed);
        g.ExtrudeTo(bx, by, feed);
        g.ExtrudeTo(ax, by, feed);
        g.ExtrudeTo(ax, ay, feed);
    }

    /// <summary>Zig-zag of 45° lines (x + y = c, or x - y = c) clipped to a rectangle.</summary>
    private static void Diagonals(Writer g, double ax, double ay, double bx, double by, double spacing,
        bool rising, double feed)
    {
        if (bx <= ax || by <= ay)
            return;
        double step = spacing * Math.Sqrt(2);          // Spacing measured across the lines.
        var lines = new List<(double X0, double Y0, double X1, double Y1)>();
        if (rising)
        {
            // Lines x - y = c, from c = ax - by to bx - ay.
            for (double c = ax - by + step / 2; c < bx - ay; c += step)
            {
                double sx = Math.Max(ax, ay + c), ex = Math.Min(bx, by + c);
                if (ex - sx > 1e-3)
                    lines.Add((sx, sx - c, ex, ex - c));
            }
        }
        else
        {
            // Lines x + y = c.
            for (double c = ax + ay + step / 2; c < bx + by; c += step)
            {
                double sx = Math.Max(ax, c - by), ex = Math.Min(bx, c - ay);
                if (ex - sx > 1e-3)
                    lines.Add((sx, c - sx, ex, c - ex));
            }
        }
        for (int i = 0; i < lines.Count; i++)
        {
            var (lx0, ly0, lx1, ly1) = lines[i];
            if (i % 2 == 1)
                (lx0, ly0, lx1, ly1) = (lx1, ly1, lx0, ly0);
            if (i == 0)
                g.TravelTo(lx0, ly0);
            else
                g.ExtrudeTo(lx0, ly0, feed);           // Short link along the perimeter.
            g.ExtrudeTo(lx1, ly1, feed);
        }
    }

    private sealed class Writer
    {
        private readonly StringBuilder _sb = new();
        private readonly double _layerHeight;
        private double _e;
        private double _x, _y;
        private bool _retracted;

        public Writer(double layerHeight) => _layerHeight = layerHeight;

        public string F(double v) => v.ToString("0.###", CultureInfo.InvariantCulture);

        private string E(double v) => v.ToString("0.#####", CultureInfo.InvariantCulture);

        public void Line(string s) => _sb.Append(s).Append('\n');

        public void Retract()
        {
            if (_retracted)
                return;
            Line($"G1 E{E(_e - 0.8)} F2100");
            _retracted = true;
        }

        public void TravelTo(double x, double y)
        {
            Line($"G0 X{F(x)} Y{F(y)} F7200");
            _x = x;
            _y = y;
        }

        public void ExtrudeTo(double x, double y, double feed)
        {
            if (_retracted)
            {
                Line($"G1 E{E(_e)} F2100");
                _retracted = false;
            }
            double len = Math.Sqrt((x - _x) * (x - _x) + (y - _y) * (y - _y));
            _e += len * Width * _layerHeight / FilamentArea;
            Line($"G1 X{F(x)} Y{F(y)} E{E(_e)} F{F(feed)}");
            _x = x;
            _y = y;
        }

        public override string ToString() => _sb.ToString();
    }
}
