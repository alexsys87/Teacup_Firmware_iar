using System.Windows.Media;
using System.Windows.Media.Imaging;
using TeacupHost.Core.GCode;

namespace TeacupHost.Controls;

/// <summary>Colors and names of the extrusion types, shared by the 3D and the layer view.</summary>
public static class FeaturePalette
{
    public static readonly int Count = Enum.GetValues<FeatureType>().Length;

    private static readonly Color[] Colors =
    {
        Color.FromRgb(0xF5, 0xA6, 0x23),   // Extrude
        Color.FromRgb(0xFF, 0x7D, 0x38),   // OuterWall
        Color.FromRgb(0xFF, 0xD5, 0x4F),   // InnerWall
        Color.FromRgb(0xC0, 0x39, 0x2B),   // Infill
        Color.FromRgb(0x9B, 0x59, 0xB6),   // SolidInfill
        Color.FromRgb(0xF0, 0x40, 0x40),   // TopSolid
        Color.FromRgb(0x4D, 0x80, 0xBA),   // Bridge
        Color.FromRgb(0xEC, 0xEF, 0xF1),   // GapFill
        Color.FromRgb(0x2E, 0xCC, 0x71),   // Support
        Color.FromRgb(0x1A, 0xBC, 0x9C),   // Skirt
        Color.FromRgb(0x78, 0x90, 0x9C),   // Custom
        Color.FromRgb(0x42, 0xA5, 0xF5),   // Travel
    };

    private static readonly SolidColorBrush[] Brushes = Colors.Select(c =>
    {
        var b = new SolidColorBrush(c);
        b.Freeze();
        return b;
    }).ToArray();

    public static Color ColorOf(FeatureType f) => Colors[(int)f];
    public static SolidColorBrush BrushOf(FeatureType f) => Brushes[(int)f];
    public static string NameOf(FeatureType f) => Services.Loc.T("S.Feature." + f);

    /// <summary>U texture coordinate that picks the color of a type from <see cref="CreateTexture"/>.</summary>
    public static double TextureU(FeatureType f) => ((int)f + 0.5) / Count;

    /// <summary>
    /// A strip with an 8 pixel wide block per type, for the 3D meshes. Wide
    /// blocks keep filtering from mixing neighbouring colors.
    /// </summary>
    public static BitmapSource CreateTexture()
    {
        const int block = 8;
        int width = Count * block;
        var pixels = new byte[width * 4];
        for (int x = 0; x < width; x++)
        {
            var c = Colors[x / block];
            pixels[x * 4 + 0] = c.B;
            pixels[x * 4 + 1] = c.G;
            pixels[x * 4 + 2] = c.R;
            pixels[x * 4 + 3] = 255;
        }
        var bmp = BitmapSource.Create(width, 1, 96, 96, PixelFormats.Bgra32, null, pixels, width * 4);
        bmp.Freeze();
        return bmp;
    }
}
