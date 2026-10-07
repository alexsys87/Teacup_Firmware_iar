using System.Numerics;
using TeacupHost.Core.GCode;

namespace TeacupHost.Core.Tests;

public class GCodeTests
{
    [Fact]
    public void ParsesCommandParametersAndComment()
    {
        var c = GCodeCommand.Parse("N12 G1 X10.5 Y-3 E.25 F1200 ; move it*55");
        Assert.Equal('G', c.Letter);
        Assert.Equal(1, c.Code);
        Assert.Equal(10.5, c.Get('X'));
        Assert.Equal(-3, c.Get('Y'));
        Assert.Equal(0.25, c.Get('E'));
        Assert.Equal(1200, c.Get('F'));
        Assert.False(c.Has('Z'));
        Assert.False(c.Has('N'));
        Assert.Equal("move it*55", c.Comment);
    }

    [Fact]
    public void ParsesBareAxisLettersAndSubcodes()
    {
        var home = GCodeCommand.Parse("G28 X Y");
        Assert.True(home.Has('X'));
        Assert.True(home.Has('Y'));
        Assert.False(home.Has('Z'));

        var prusa = GCodeCommand.Parse("M862.3 P \"MK3S\"");
        Assert.Equal(862, prusa.Code);
        Assert.Equal(3, prusa.SubCode);
    }

    [Fact]
    public void KeepsFreeTextOfFileCommands()
    {
        var c = GCodeCommand.Parse("M23 PART.GCO");
        Assert.Equal("PART.GCO", c.Argument);
        Assert.False(c.Has('P'));
    }

    [Fact]
    public void DocumentTracksByteOffsetsWithCrLf()
    {
        var doc = GCodeDocument.FromBytes("x", null, "G28\r\nG1 X1\n\nM104 S0"u8.ToArray());
        Assert.Equal(new[] { "G28", "G1 X1", "", "M104 S0" }, doc.Lines);
        Assert.Equal(new long[] { 0, 5, 11, 12 }, doc.LineOffsets);
        Assert.Equal(0, doc.LineAtOffset(0));
        Assert.Equal(0, doc.LineAtOffset(4));
        Assert.Equal(1, doc.LineAtOffset(5));
        Assert.Equal(3, doc.LineAtOffset(100));
    }

    [Fact]
    public void BuildsLayersAndExtrusionKinds()
    {
        var lines = new[]
        {
            "G28",                       // 0
            "G1 Z0.2 F600",              // 1 travel
            "G1 X10 E1 F1200",           // 2 extrude, layer 0
            "G1 Y10 E2",                 // 3
            "G1 E1.5",                   // 4 retract
            "G1 Z0.4",                   // 5 travel, belongs to layer 1
            "G1 X0 E2.5",                // 6 extrude, layer 1
            "G1 X0 Y0",                  // 7 travel
        };
        var tp = ToolpathBuilder.Build(lines);
        Assert.Equal(2, tp.Layers.Count);
        Assert.Equal(0.2f, tp.Layers[0].Z, 3);
        Assert.Equal(0.4f, tp.Layers[1].Z, 3);
        Assert.Equal(0.2f, tp.Layers[1].Height, 3);

        var s = tp.Segments;
        Assert.Equal(MoveKind.Travel, s[0].Kind);
        Assert.Equal(MoveKind.Extrude, s[1].Kind);
        Assert.Equal(2, s[1].Line);
        Assert.Equal(0, s[0].Layer);
        // The Z move after the retract starts layer 1.
        var zMove = s.Single(x => x.Line == 5);
        Assert.Equal(1, zMove.Layer);
        Assert.Equal(3f, tp.FilamentLength, 3);     // 1 + 1 + 1, the retract isn't counted
        Assert.Equal(new Vector3(0, 0, 0.2f), tp.Min);
        Assert.Equal(new Vector3(10, 10, 0.4f), tp.Max);
    }

    [Fact]
    public void ExpandsFullCircleArc()
    {
        var tp = ToolpathBuilder.Build(new[] { "G1 X10 Y0 Z0.2 F3000", "G2 X10 Y0 I-10 J0 E5" });
        var arc = tp.Segments.Where(x => x.Line == 1).ToArray();
        Assert.True(arc.Length > 20);
        float len = arc.Sum(x => x.Length);
        Assert.InRange(len, 2 * MathF.PI * 10 * 0.995f, 2 * MathF.PI * 10 * 1.001f);
        Assert.Equal(new Vector3(10, 0, 0.2f), arc[^1].End);
        Assert.Equal(5f, arc.Sum(x => x.Extrusion), 3);
        // Clockwise from (10,0) around (0,0): first point goes to negative Y.
        Assert.True(arc[0].End.Y < 0);
    }

    [Fact]
    public void ArcWithRadius()
    {
        var tp = ToolpathBuilder.Build(new[] { "G1 X0 Y0 F3000", "G3 X10 Y0 R5" });
        var arc = tp.Segments.Where(x => x.Line == 1).ToArray();
        float len = arc.Sum(x => x.Length);
        Assert.InRange(len, MathF.PI * 5 * 0.995f, MathF.PI * 5 * 1.001f);
        // Counter-clockwise from (0,0) to (10,0): goes through negative Y.
        Assert.True(arc.Min(x => x.End.Y) < -4.9f);
    }

    [Fact]
    public void RelativeModesAndG92()
    {
        var tp = ToolpathBuilder.Build(new[]
        {
            "G91", "G1 X5 Y5 E1 F600", "G1 X5 E1", "G90", "G92 X0 Y0", "G1 X1 Y0",
        });
        Assert.Equal(new Vector3(10, 5, 0), tp.Segments[1].End);
        Assert.Equal(2f, tp.FilamentLength, 3);
        Assert.Equal(new Vector3(0, 0, 0), tp.Segments[2].Start);
    }

    [Fact]
    public void TimeEstimateUsesTrapezoid()
    {
        // 100 mm at 50 mm/s with 1000 mm/s²: 0.05 s ramp up and down
        // (1.25 mm each), 97.5 mm cruise = 1.95 s + 0.1 s = 2.05 s.
        double t = ToolpathBuilder.TrapezoidTime(100, 0, 50, 0, 1000);
        Assert.Equal(2.05, t, 3);
        // Too short to reach full speed: triangle.
        double tri = ToolpathBuilder.TrapezoidTime(1, 0, 100, 0, 1000);
        Assert.Equal(2 * Math.Sqrt(1 / 1000.0), tri, 4);

        var tp = ToolpathBuilder.Build(new[] { "G1 X100 F3000", "G4 P500", "G1 X0" });
        Assert.Equal(2.05f, tp.Segments[0].Duration, 2);
        Assert.Equal(2.55f, tp.Segments[1].StartTime, 2);
        Assert.Equal(4.6f, tp.TotalTime, 2);
    }

    [Fact]
    public void MapsTimeAndLinesToSegments()
    {
        var tp = ToolpathBuilder.Build(new[] { "G1 X10 F600", "M104 S200", "G1 X20", "G1 X30" });
        Assert.Equal(-1, tp.SegmentAtTime(-1));
        Assert.Equal(0, tp.SegmentAtTime(0.1f));
        Assert.Equal(2, tp.SegmentAtTime(tp.TotalTime));
        Assert.Equal(0, tp.LastSegmentAtOrBeforeLine(1));
        Assert.Equal(1, tp.LastSegmentAtOrBeforeLine(2));
        Assert.Equal(1, tp.FirstSegmentAtOrAfterLine(1));
    }

    [Fact]
    public void ReadsSlicerFeatureTypes()
    {
        Assert.Equal(FeatureType.OuterWall, ToolpathBuilder.ParseFeature("External perimeter"));
        Assert.Equal(FeatureType.InnerWall, ToolpathBuilder.ParseFeature("Perimeter"));
        Assert.Equal(FeatureType.OuterWall, ToolpathBuilder.ParseFeature("WALL-OUTER"));
        Assert.Equal(FeatureType.InnerWall, ToolpathBuilder.ParseFeature("WALL-INNER"));
        Assert.Equal(FeatureType.Infill, ToolpathBuilder.ParseFeature("Internal infill"));
        Assert.Equal(FeatureType.SolidInfill, ToolpathBuilder.ParseFeature("Solid infill"));
        Assert.Equal(FeatureType.TopSolid, ToolpathBuilder.ParseFeature("Top solid infill"));
        Assert.Equal(FeatureType.SolidInfill, ToolpathBuilder.ParseFeature("SKIN"));
        Assert.Equal(FeatureType.Support, ToolpathBuilder.ParseFeature("SUPPORT-INTERFACE"));
        Assert.Equal(FeatureType.Skirt, ToolpathBuilder.ParseFeature("Skirt/Brim"));
        Assert.Equal(FeatureType.Bridge, ToolpathBuilder.ParseFeature("Bridge infill"));
    }

    [Fact]
    public void DemoPrintHasLayersAndFeatures()
    {
        var doc = GCodeDocument.FromText("demo", DemoGCode.Generate());
        var tp = ToolpathBuilder.Build(doc.Lines);
        Assert.Equal(50, tp.Layers.Count);
        Assert.Contains(tp.Segments, s => s.Feature == FeatureType.OuterWall);
        Assert.Contains(tp.Segments, s => s.Feature == FeatureType.Infill);
        Assert.Contains(tp.Segments, s => s.Feature == FeatureType.TopSolid);
        Assert.True(tp.TotalTime > 60);
        // Layers cover all segments without gaps.
        int next = 0;
        foreach (var l in tp.Layers)
        {
            Assert.Equal(next, l.FirstSegment);
            next = l.EndSegment;
        }
        Assert.Equal(tp.Segments.Length, next);
    }
}
