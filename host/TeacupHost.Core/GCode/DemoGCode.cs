using System.Globalization;
using System.Text;

namespace TeacupHost.Core.GCode;

/// <summary>
/// Generates a small test print (a hollow rounded box with infill bottom)
/// for the virtual printer and for trying out the viewer without a slicer.
/// </summary>
public static class DemoGCode
{
    public static string Generate(double centerX = 110, double centerY = 90, double size = 30,
        double height = 10, double layerHeight = 0.2)
    {
        var sb = new StringBuilder();
        var ci = CultureInfo.InvariantCulture;
        const double filamentArea = Math.PI * 1.75 * 1.75 / 4;
        const double width = 0.45;
        double e = 0;

        void Line(string s) => sb.Append(s).Append('\n');
        string F(double v) => v.ToString("0.###", ci);
        void Move(double x, double y, double feed) => Line($"G0 X{F(x)} Y{F(y)} F{F(feed)}");
        void Extrude(double x0, double y0, double x1, double y1, double feed, double lh)
        {
            double len = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            e += len * width * lh / filamentArea;
            Line($"G1 X{F(x1)} Y{F(y1)} E{e.ToString("0.#####", ci)} F{F(feed)}");
        }

        Line("; Teacup Host demo print");
        Line($"; size {F(size)} x {F(size)} x {F(height)} mm, layer {F(layerHeight)} mm");
        Line("M140 S60");
        Line("M104 S205");
        Line("M190 S60");
        Line("M109 S205");
        Line("G28");
        Line("G90");
        Line("M82");
        Line("G92 E0");
        Line("G1 Z5 F240");

        // Prime line at the front edge.
        Line(";TYPE:Skirt");
        Move(centerX - size / 2 - 5, centerY - size / 2 - 5, 6000);
        Line($"G1 Z{F(layerHeight)} F240");
        double px = centerX - size / 2 - 5, py = centerY - size / 2 - 5;
        Extrude(px, py, px + size + 10, py, 1200, layerHeight);
        Extrude(px + size + 10, py, px + size + 10, py + size + 10, 1200, layerHeight);
        Extrude(px + size + 10, py + size + 10, px, py + size + 10, 1200, layerHeight);
        Extrude(px, py + size + 10, px, py, 1200, layerHeight);

        int layers = (int)Math.Round(height / layerHeight);
        double radius = size * 0.2;
        for (int layer = 0; layer < layers; layer++)
        {
            double z = (layer + 1) * layerHeight;
            Line(";LAYER_CHANGE");
            Line($";Z:{F(z)}");
            Line($";HEIGHT:{F(layerHeight)}");
            Line($"G1 E{(e - 0.8).ToString("0.#####", ci)} F2100");   // Retract.
            Line($"G1 Z{F(z + 0.2)} F240");
            // Slight twist so the layers are easy to tell apart.
            double twist = layer * 0.6 * Math.PI / 180;

            for (int wall = 0; wall < 2; wall++)
            {
                double inset = wall * width;
                var pts = RoundedRect(centerX, centerY, size / 2 - inset, radius - inset, twist);
                Line(wall == 0 ? ";TYPE:External perimeter" : ";TYPE:Perimeter");
                Move(pts[0].x, pts[0].y, 7200);
                Line($"G1 Z{F(z)} F240");
                Line($"G1 E{e.ToString("0.#####", ci)} F2100");       // Unretract.
                double speed = wall == 0 ? 1500 : 2400;
                for (int i = 1; i < pts.Count; i++)
                    Extrude(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y, speed, layerHeight);
                Extrude(pts[^1].x, pts[^1].y, pts[0].x, pts[0].y, speed, layerHeight);
            }

            // Solid bottom and top, sparse infill between.
            bool solid = layer < 3 || layer >= layers - 3;
            double spacing = solid ? width : 3.0;
            double half = size / 2 - 2 * width - 0.2;
            Line(solid ? (layer >= layers - 3 ? ";TYPE:Top solid infill" : ";TYPE:Solid infill") : ";TYPE:Internal infill");
            bool diag = layer % 2 == 0;
            int lines = (int)(2 * half / spacing);
            double lastX = 0, lastY = 0;
            for (int i = 0; i <= lines; i++)
            {
                double o = -half + i * spacing;
                (double x0, double y0, double x1, double y1) = diag
                    ? (centerX + o, centerY - half, centerX + o, centerY + half)
                    : (centerX - half, centerY + o, centerX + half, centerY + o);
                if (i % 2 == 1)
                    (x0, y0, x1, y1) = (x1, y1, x0, y0);
                if (i == 0)
                    Move(x0, y0, 7200);
                else
                    Extrude(lastX, lastY, x0, y0, 3000, layerHeight);
                Extrude(x0, y0, x1, y1, solid ? 2400 : 3600, layerHeight);
                lastX = x1;
                lastY = y1;
            }
        }

        Line("G1 E" + (e - 1).ToString("0.#####", ci) + " F2100");
        Line($"G1 Z{F(height + 10)} F240");
        Line("G0 X10 Y170 F6000");
        Line("M104 S0");
        Line("M140 S0");
        Line("M107");
        Line("M84");
        Line("; end of demo print");
        return sb.ToString();
    }

    private static List<(double x, double y)> RoundedRect(double cx, double cy, double half, double r,
        double angle)
    {
        var pts = new List<(double, double)>();
        r = Math.Max(0.5, r);
        double inner = half - r;
        (double, double)[] corners = { (inner, inner), (-inner, inner), (-inner, -inner), (inner, -inner) };
        double cos = Math.Cos(angle), sin = Math.Sin(angle);
        for (int c = 0; c < 4; c++)
        {
            for (int k = 0; k <= 6; k++)
            {
                double a = (c * 90 + k * 15) * Math.PI / 180;
                double x = corners[c].Item1 + r * Math.Cos(a);
                double y = corners[c].Item2 + r * Math.Sin(a);
                pts.Add((cx + x * cos - y * sin, cy + x * sin + y * cos));
            }
        }
        return pts;
    }
}
