using System.Text;
using TeacupHost.Core.GCode;
using TeacupHost.Core.Printing;

namespace TeacupHost.Core.Tests;

public class ProtocolTests
{
    [Theory]
    [InlineData(1, "G28", "N1 G28*18")]
    [InlineData(0, "M110 N0", "N0 M110 N0*125")]
    [InlineData(42, "G1 X10.5 Y-3 E0.12", "N42 G1 X10.5 Y-3 E0.12*99")]
    public void FormatsNumberedLines(int n, string cmd, string expected) =>
        Assert.Equal(expected, ResponseParser.FormatNumbered(n, cmd));

    [Fact]
    public void ParsesTeacupTemperatureLines()
    {
        Assert.True(ResponseParser.TryParseTemperature("ok T:201.5/205.0 B:59.8/60.0 @:87 B@:255", out var r));
        Assert.Equal(201.5, r.Hotend);
        Assert.Equal(205.0, r.HotendTarget);
        Assert.Equal(59.8, r.Bed);
        Assert.Equal(60.0, r.BedTarget);
        Assert.Equal(87, r.HotendPower);
        Assert.Equal(255, r.BedPower);
        Assert.True(r.HasBed);

        Assert.True(ResponseParser.TryParseTemperature("T:25.0 B:24.5", out r));
        Assert.Equal(25.0, r.Hotend);
        Assert.Equal(24.5, r.Bed);

        Assert.False(ResponseParser.TryParseTemperature(
            "FIRMWARE_NAME:Teacup PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel HEATER_COUNT:1", out _));
    }

    [Fact]
    public void ParsesOtherMessages()
    {
        Assert.True(ResponseParser.TryParsePosition("X:10.000 Y:0.000 Z:0.200 E:1.500 Count X:400 Y:0 Z:0", out var p));
        Assert.Equal(new PrinterPosition(10, 0, 0.2, 1.5), p);

        Assert.True(ResponseParser.TryParseResend("Resend: 42", out int n));
        Assert.Equal(42, n);
        Assert.True(ResponseParser.TryParseResend("rs N7", out n));
        Assert.Equal(7, n);

        Assert.True(ResponseParser.TryParseSdProgress("SD printing byte 1234/56789", out long pos, out long size));
        Assert.Equal(1234, pos);
        Assert.Equal(56789, size);

        Assert.True(ResponseParser.TryParseFileEntry("SUB/PART.GCO 1234", out var f));
        Assert.Equal(new SdFileInfo("SUB/PART.GCO", 1234), f);

        Assert.True(ResponseParser.IsOk("ok"));
        Assert.True(ResponseParser.IsOk("ok T:20"));
        Assert.False(ResponseParser.IsOk("okay"));

        Assert.True(ResponseParser.IsFatalError("Error:Printer halted. kill() called!"));
        Assert.True(ResponseParser.IsFatalError("Error:Heater off but temperature rising, system stopped! Heater_ID: 0"));
        Assert.False(ResponseParser.IsFatalError("Error:checksum mismatch, Last Line: 41"));
    }

    private static List<JobLine> JobFrom(string text)
    {
        var doc = GCodeDocument.FromText("t", text);
        var job = new List<JobLine>();
        for (int i = 0; i < doc.Lines.Count; i++)
        {
            string c = GCodeCommand.StripComment(doc.Lines[i]);
            if (c.Length > 0)
                job.Add(new JobLine(c, i));
        }
        return job;
    }

    private static async Task WaitFor(Func<bool> condition, int seconds = 30)
    {
        var end = DateTime.UtcNow.AddSeconds(seconds);
        while (!condition())
        {
            if (DateTime.UtcNow > end)
                throw new TimeoutException("Condition not met in time");
            await Task.Delay(10);
        }
    }

    private static string FastPrint() =>
        DemoGCode.Generate(size: 10, height: 0.6).Replace("M190 S60\n", "").Replace("M109 S205\n", "");

    private sealed class BusyPort : IPrinterTransport
    {
        public bool Disposed;
        public string Name => "COM99";
        public bool IsOpen => false;
        public event Action<string>? LineReceived { add { } remove { } }
        public event Action<Exception>? Faulted { add { } remove { } }
        public void Open() => throw new UnauthorizedAccessException("Port is busy");
        public void Close() { }
        public void WriteLine(string line) => throw new InvalidOperationException();
        public void Dispose() => Disposed = true;
    }

    [Fact]
    public void FailedOpenLeavesConnectionClean()
    {
        using var conn = new PrinterConnection();
        var port = new BusyPort();
        Assert.Throws<UnauthorizedAccessException>(() => conn.Connect(port));
        Assert.True(port.Disposed);
        Assert.Equal(ConnectionState.Disconnected, conn.State);
        conn.Send("M105");                      // Nothing to send to, no exception.
    }

    [Fact]
    public void NonAsciiBecomesQuestionMarks() =>
        Assert.Equal("M117 ??????", PrinterConnection.ToAscii("M117 Привет"));

    [Fact]
    public async Task ConnectsAndGetsTemperature()
    {
        using var printer = new VirtualPrinter { TimeScale = 100 };
        using var conn = new PrinterConnection();
        TemperatureReading? temp = null;
        conn.TemperatureUpdated += t => temp = t;
        conn.Connect(printer);
        await WaitFor(() => conn.State == ConnectionState.Online && temp != null);
        Assert.InRange(temp!.Value.Hotend, 15, 30);
    }

    [Fact]
    public async Task PrintsJobWithResends()
    {
        using var printer = new VirtualPrinter { TimeScale = 2000, LineErrorRate = 0.05 };
        using var conn = new PrinterConnection();
        var job = JobFrom(FastPrint());
        bool? cancelled = null;
        int resends = 0;
        int last = -1;
        bool ordered = true;
        conn.LineSent += (_, kind) => { if (kind == SendKind.Resend) resends++; };
        conn.JobProgress += i => { if (i <= last) ordered = false; last = i; };
        conn.JobCompleted += (_, c) => cancelled = c;
        conn.Connect(printer);
        await WaitFor(() => conn.State == ConnectionState.Online);
        conn.StartJob(job);
        await WaitFor(() => cancelled != null, 60);

        Assert.False(cancelled);
        Assert.True(resends > 0, "error injection should cause resends");
        Assert.True(ordered);
        Assert.Equal(job.Count - 1, last);
        Assert.Equal(job.Count, conn.JobAcknowledged);
    }

    [Fact]
    public async Task UploadsFileAndPrintsFromFlash()
    {
        using var printer = new VirtualPrinter { TimeScale = 2000, LineErrorRate = 0.02 };
        using var conn = new PrinterConnection { SdPollInterval = 0.05 };
        var lines = JobFrom(FastPrint());
        bool? done = null;
        IReadOnlyList<SdFileInfo>? files = null;
        bool finished = false;
        long lastPos = 0;
        conn.JobCompleted += (_, c) => done = !c;
        conn.SdFilesListed += f => files = f;
        conn.SdPrintFinished += () => finished = true;
        conn.SdProgress += (p, _) => lastPos = p;
        conn.Connect(printer);
        await WaitFor(() => conn.State == ConnectionState.Online);

        conn.StartJob(PrinterConnection.CreateUploadJob("TEST.GCO", lines), JobKind.Upload);
        await WaitFor(() => done != null, 60);
        Assert.True(done);

        conn.SdRefresh();
        await WaitFor(() => files != null);
        var stored = printer.Files.Single(f => f.Name == "TEST.GCO").Data;
        string expected = string.Concat(lines.Select(l => l.Command + "\n"));
        Assert.Equal(expected, Encoding.ASCII.GetString(stored));
        Assert.Contains(files!, f => f.Name == "TEST.GCO" && f.Size == stored.Length);

        // Slow enough that the M27 polls see the print running.
        printer.TimeScale = 5;
        conn.SdStartPrint("TEST.GCO");
        await WaitFor(() => finished, 60);
        Assert.True(lastPos > 0);
    }

    [Fact]
    public async Task EmergencyStopHaltsPrinter()
    {
        using var printer = new VirtualPrinter { TimeScale = 1 };
        using var conn = new PrinterConnection();
        string? error = null;
        conn.PrinterError += e => error = e;
        conn.Connect(printer);
        await WaitFor(() => conn.State == ConnectionState.Online);
        conn.Send("M109 S250");                 // Long wait in the printer.
        await Task.Delay(200);
        conn.Send("M112");
        await WaitFor(() => conn.State == ConnectionState.Halted);
        Assert.Contains("halted", error);

        conn.Send("M999");
        await WaitFor(() => conn.State == ConnectionState.Online);
    }

    [Fact]
    public async Task M108CancelsHeatingWait()
    {
        using var printer = new VirtualPrinter { TimeScale = 1 };
        using var conn = new PrinterConnection();
        var received = new List<string>();
        conn.LineReceived += l => { lock (received) received.Add(l); };
        conn.Connect(printer);
        // Wait for the answer of the M114 sent after connecting.
        await WaitFor(() => { lock (received) return received.Any(l => l.StartsWith("X:")); });
        lock (received)
            received.Clear();
        conn.Send("M109 S250");
        conn.Send("M114");
        await Task.Delay(300);
        lock (received)
            Assert.DoesNotContain(received, l => l.StartsWith("X:"));
        conn.Send("M108");
        // M114 only runs after M109 finished.
        await WaitFor(() => { lock (received) return received.Any(l => l.StartsWith("X:")); }, 10);
        lock (received)
            Assert.Contains(received, l => l.Contains("Wait cancelled"));
    }
}
