using System.Net;
using System.Net.Sockets;
using System.Text;
using TeacupHost.Core.GCode;
using TeacupHost.Core.Printing;

namespace TeacupHost.Core.Tests;

/// <summary>
/// A cable between the host and a virtual printer that can be pulled: the
/// printer keeps running (and keeps its state), lines in either direction
/// are lost while it's out, and opening fails like a missing COM port.
/// </summary>
internal sealed class PluggableLink
{
    private readonly object _lock = new();
    private View? _active;

    public PluggableLink(VirtualPrinter printer)
    {
        Printer = printer;
        printer.LineReceived += line =>
        {
            View? v;
            lock (_lock)
                v = Plugged ? _active : null;
            v?.Receive(line);
        };
        printer.Open();
    }

    public VirtualPrinter Printer { get; }
    public bool Plugged { get; private set; } = true;
    public int Opens { get; private set; }

    public IPrinterTransport NewTransport() => new View(this);

    public void Unplug()
    {
        View? v;
        lock (_lock)
        {
            Plugged = false;
            v = _active;
            _active = null;
        }
        v?.Fail();
    }

    public void Plug()
    {
        lock (_lock)
            Plugged = true;
    }

    private sealed class View : IPrinterTransport
    {
        private readonly PluggableLink _link;
        private bool _open;

        public View(PluggableLink link) => _link = link;

        public string Name => "LINK";
        public bool IsOpen => _open;
        public event Action<string>? LineReceived;
        public event Action<Exception>? Faulted;

        public void Open()
        {
            lock (_link._lock)
            {
                if (!_link.Plugged)
                    throw new IOException("The port does not exist");
                _link._active = this;
                _link.Opens++;
                _open = true;
            }
        }

        public void Receive(string line) => LineReceived?.Invoke(line);

        public void Fail()
        {
            _open = false;
            Faulted?.Invoke(new IOException("Device unplugged"));
        }

        public void WriteLine(string line)
        {
            bool ok;
            lock (_link._lock)
                ok = _open && _link.Plugged && _link._active == this;
            if (ok)
                _link.Printer.WriteLine(line);
        }

        public void Close()
        {
            lock (_link._lock)
            {
                _open = false;
                if (_link._active == this)
                    _link._active = null;
            }
        }

        public void Dispose() => Close();
    }
}

public class ReconnectTests
{
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

    private static string Print() =>
        DemoGCode.Generate(width: 10, depth: 10, height: 1).Replace("M190 S60\n", "").Replace("M109 S205\n", "");

    private static PrinterConnection NewConnection() => new()
    {
        ReconnectInterval = 0.1,
        LinkTimeout = 5,
    };

    [Fact]
    public async Task ReconnectsWithoutJob()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 100 });
        using var conn = NewConnection();
        var states = new List<ConnectionState>();
        conn.StateChanged += s => { lock (states) states.Add(s); };
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);

        link.Unplug();
        await WaitFor(() => conn.State == ConnectionState.Reconnecting);
        await Task.Delay(500);                       // Port missing: attempts fail.
        Assert.Equal(ConnectionState.Reconnecting, conn.State);

        link.Plug();
        await WaitFor(() => conn.State == ConnectionState.Online);
        Assert.True(link.Opens >= 2);
        lock (states)
            Assert.DoesNotContain(ConnectionState.Disconnected, states);
        link.Printer.Dispose();
    }

    [Fact]
    public async Task PrintSurvivesShortOutages()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 10, LineErrorRate = 0.02 });
        using var conn = NewConnection();
        var job = JobFrom(Print());
        bool? cancelled = null;
        int last = -1;
        bool ordered = true;
        var messages = new List<ConnectionMessage>();
        conn.JobProgress += i => { if (i <= last) ordered = false; last = i; };
        conn.JobCompleted += (_, c) => cancelled = c;
        var log = new List<string>();
        conn.Info += (m, d) => { lock (messages) messages.Add(m); lock (log) log.Add("! " + m + " " + d); };
        conn.LineReceived += l => { lock (log) log.Add("< " + l); };
        conn.LineSent += (l, k) => { lock (log) log.Add("> " + l); };
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        conn.StartJob(job);

        // Three short outages while printing.
        for (int k = 0; k < 3; k++)
        {
            await WaitFor(() => last > (k + 1) * job.Count / 5 || cancelled != null, 60);
            if (cancelled != null)
                break;
            link.Unplug();
            await Task.Delay(300);
            link.Plug();
        }
        await WaitFor(() => cancelled != null, 90);

        if (cancelled != false)
            lock (log) Assert.Fail(string.Join("\n", log.TakeLast(60)));
        Assert.False(cancelled);
        Assert.True(ordered);
        Assert.Equal(job.Count - 1, last);
        lock (messages)
        {
            Assert.Equal(3, messages.Count(m => m == ConnectionMessage.Reconnected));
            Assert.Equal(3, messages.Count(m => m == ConnectionMessage.JobResumed));
        }
        link.Printer.Dispose();
    }

    [Fact]
    public async Task PrintStopsWhenPrinterParkedDuringOutage()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 5 });
        using var conn = NewConnection();
        var job = JobFrom(Print());
        bool? cancelled = null;
        int last = -1;
        var messages = new List<ConnectionMessage>();
        conn.JobProgress += i => last = i;
        conn.JobCompleted += (_, c) => cancelled = c;
        conn.Info += (m, _) => { lock (messages) messages.Add(m); };
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        conn.StartJob(job);
        await WaitFor(() => last > 30);
        Assert.True(link.Printer.HotendTarget > 0);

        link.Unplug();
        await WaitFor(() => conn.State == ConnectionState.Reconnecting);
        link.Printer.ParkAsHostLost();               // Firmware: host gone for 2 s.
        await WaitFor(() => link.Printer.HotendTarget == 0);
        link.Plug();

        await WaitFor(() => cancelled != null);
        Assert.True(cancelled);
        await WaitFor(() => conn.State == ConnectionState.Online);
        lock (messages)
            Assert.Contains(ConnectionMessage.HostLostDuringOutage, messages);
        link.Printer.Dispose();
    }

    [Fact]
    public async Task PrintStopsWhenPrinterRestartedDuringOutage()
    {
        // No heating in this job: the restart shows by the line numbers.
        var lines = Enumerable.Range(0, 400).Select(i => $"G1 X{10 + i % 50} Y{10 + i / 50} F6000").ToList();
        var job = JobFrom("G28\n" + string.Join("\n", lines));
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 5 });
        using var conn = NewConnection();
        bool? cancelled = null;
        int last = -1;
        var messages = new List<ConnectionMessage>();
        conn.JobProgress += i => last = i;
        conn.JobCompleted += (_, c) => cancelled = c;
        conn.Info += (m, _) => { lock (messages) messages.Add(m); };
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        conn.StartJob(job);
        await WaitFor(() => last > 20);

        link.Unplug();
        await WaitFor(() => conn.State == ConnectionState.Reconnecting);
        link.Printer.Reboot();
        await Task.Delay(300);
        link.Plug();

        await WaitFor(() => cancelled != null);
        Assert.True(cancelled);
        lock (messages)
            Assert.True(messages.Contains(ConnectionMessage.PrinterResetDuringPrint) ||
                        messages.Contains(ConnectionMessage.HostLostDuringOutage));
        await WaitFor(() => conn.State == ConnectionState.Online);
        link.Printer.Dispose();
    }

    [Fact]
    public async Task SilentLinkIsReopened()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 100 });
        using var conn = NewConnection();
        conn.LinkTimeout = 1.5;
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        int opens = link.Opens;
        // The printer stops reporting temperatures: nothing arrives any more.
        conn.Send("M155 S0");
        await WaitFor(() => link.Opens > opens, 20);
        await WaitFor(() => conn.State == ConnectionState.Online);
        link.Printer.Dispose();
    }

    [Fact]
    public async Task NoReconnectWhenDisabled()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 100 });
        using var conn = NewConnection();
        conn.AutoReconnect = false;
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        link.Unplug();
        await WaitFor(() => conn.State == ConnectionState.Disconnected);
        link.Printer.Dispose();
    }

    [Fact]
    public async Task UserDisconnectStopsReconnecting()
    {
        var link = new PluggableLink(new VirtualPrinter { TimeScale = 100 });
        using var conn = NewConnection();
        conn.Connect(link.NewTransport);
        await WaitFor(() => conn.State == ConnectionState.Online);
        link.Unplug();
        await WaitFor(() => conn.State == ConnectionState.Reconnecting);
        conn.Disconnect();
        Assert.Equal(ConnectionState.Disconnected, conn.State);
        int opens = link.Opens;
        link.Plug();
        await Task.Delay(500);
        Assert.Equal(opens, link.Opens);
        Assert.Equal(ConnectionState.Disconnected, conn.State);
        link.Printer.Dispose();
    }
}

/// <summary>A Telnet serial bridge (like ESP3D or ser2net) in front of a virtual printer.</summary>
internal sealed class TelnetBridge : IDisposable
{
    private readonly TcpListener _listener = new(IPAddress.Loopback, 0);
    private readonly VirtualPrinter _printer;
    private readonly object _lock = new();
    private TcpClient? _client;
    private volatile bool _running = true;

    public TelnetBridge(VirtualPrinter printer)
    {
        _printer = printer;
        printer.LineReceived += line =>
        {
            NetworkStream? s;
            lock (_lock)
                s = _client?.Connected == true ? _client.GetStream() : null;
            try
            {
                var b = Encoding.ASCII.GetBytes(line + "\r\n");
                s?.Write(b, 0, b.Length);
            }
            catch
            {
                // The client is gone.
            }
        };
        printer.Open();
        _listener.Start();
        Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
        new Thread(AcceptLoop) { IsBackground = true }.Start();
    }

    public int Port { get; }
    public int Accepted { get; private set; }

    private void AcceptLoop()
    {
        while (_running)
        {
            TcpClient c;
            try
            {
                c = _listener.AcceptTcpClient();
            }
            catch
            {
                return;
            }
            lock (_lock)
            {
                _client?.Close();
                _client = c;
                Accepted++;
            }
            var s = c.GetStream();
            // Telnet servers open with option negotiation (here: echo, SGA,
            // and a subnegotiation); the client must swallow it.
            s.Write(new byte[] { 255, 251, 1, 255, 251, 3, 255, 250, 24, 1, 255, 240 });
            new Thread(() => ReadLoop(c)) { IsBackground = true }.Start();
        }
    }

    private void ReadLoop(TcpClient c)
    {
        var sb = new StringBuilder();
        var buf = new byte[1024];
        try
        {
            var s = c.GetStream();
            int n;
            while ((n = s.Read(buf, 0, buf.Length)) > 0)
            {
                for (int i = 0; i < n; i++)
                {
                    if (buf[i] == 255)
                    {
                        i += 2;                  // Client's IAC WONT/DONT x.
                        continue;
                    }
                    char ch = (char)buf[i];
                    if (ch == '\n')
                    {
                        _printer.WriteLine(sb.ToString());
                        sb.Clear();
                    }
                    else if (ch != '\r')
                    {
                        sb.Append(ch);
                    }
                }
            }
        }
        catch
        {
            // Dropped.
        }
    }

    /// <summary>Cut the connection like a WiFi drop on the bridge side.</summary>
    public void Drop()
    {
        lock (_lock)
        {
            _client?.Client.Close();
            _client = null;
        }
    }

    public void Dispose()
    {
        _running = false;
        _listener.Stop();
        Drop();
        _printer.Dispose();
    }
}

public class TelnetTests
{
    [Theory]
    [InlineData("192.168.1.50", "192.168.1.50", 23)]
    [InlineData("printer.local:8080", "printer.local", 8080)]
    [InlineData(" 10.0.0.7:2323 ", "10.0.0.7", 2323)]
    [InlineData("[fe80::1]:23", "fe80::1", 23)]
    [InlineData("fe80::1", "fe80::1", 23)]
    public void ParsesAddresses(string text, string host, int port) =>
        Assert.Equal((host, port), TcpTransport.ParseAddress(text));

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

    [Fact]
    public async Task PrintsOverTelnetAndReconnectsAfterDrop()
    {
        using var bridge = new TelnetBridge(new VirtualPrinter { TimeScale = 50 });
        using var conn = new PrinterConnection { ReconnectInterval = 0.2, LinkTimeout = 5 };
        var doc = GCodeDocument.FromText("t", DemoGCode.Generate(width: 10, depth: 10, height: 1)
            .Replace("M190 S60\n", "").Replace("M109 S205\n", ""));
        var job = new List<JobLine>();
        for (int i = 0; i < doc.Lines.Count; i++)
        {
            string c = GCodeCommand.StripComment(doc.Lines[i]);
            if (c.Length > 0)
                job.Add(new JobLine(c, i));
        }
        bool? cancelled = null;
        int last = -1;
        TemperatureReading? temp = null;
        conn.JobProgress += i => last = i;
        conn.JobCompleted += (_, c) => cancelled = c;
        conn.TemperatureUpdated += t => temp = t;

        conn.Connect(() => new TcpTransport("127.0.0.1", bridge.Port));
        await WaitFor(() => conn.State == ConnectionState.Online && temp != null);
        Assert.Equal("127.0.0.1:" + bridge.Port, conn.PortName);

        conn.StartJob(job);
        await WaitFor(() => last > job.Count / 3);
        bridge.Drop();
        await WaitFor(() => cancelled != null, 90);

        Assert.False(cancelled);
        Assert.Equal(job.Count - 1, last);
        Assert.True(bridge.Accepted >= 2);
    }

    [Fact]
    public void RefusedConnectionThrows()
    {
        var l = new TcpListener(IPAddress.Loopback, 0);
        l.Start();
        int port = ((IPEndPoint)l.LocalEndpoint).Port;
        l.Stop();
        using var conn = new PrinterConnection();
        Assert.ThrowsAny<Exception>(() => conn.Connect(() => new TcpTransport("127.0.0.1", port, 1000)));
        Assert.Equal(ConnectionState.Disconnected, conn.State);
    }
}
