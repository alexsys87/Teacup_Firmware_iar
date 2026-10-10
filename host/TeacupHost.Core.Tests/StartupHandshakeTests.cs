using TeacupHost.Core.Printing;

namespace TeacupHost.Core.Tests;

public class StartupHandshakeTests
{
    private sealed class ControlledTransport : IPrinterTransport
    {
        public string Name => "Controlled startup";
        public bool IsOpen { get; private set; }
        public event Action<string>? LineReceived;
        public event Action<Exception>? Faulted { add { } remove { } }
        public void Open() => IsOpen = true;
        public void Close() => IsOpen = false;
        public void Dispose() => Close();
        public void WriteLine(string line) { }
        public void Receive(string line) => LineReceived?.Invoke(line);
    }

    private static void FinishHandshake(ControlledTransport transport, PrinterConnection connection)
    {
        transport.Receive("ok");                    // M110
        Assert.Equal(ConnectionState.Online, connection.State);
        transport.Receive("ok");                    // M115
        transport.Receive("ok");                    // M105
        transport.Receive("ok");                    // M114 (TemperatureInterval=0)
    }

    [Fact]
    public void StartupBannerDoesNotDuplicatePendingHandshakeOrCancelResentJob()
    {
        using var transport = new ControlledTransport();
        using var connection = new PrinterConnection { TemperatureInterval = 0 };
        var sent = new List<(string Line, SendKind Kind)>();
        bool? cancelled = null;
        connection.LineSent += (line, kind) => sent.Add((line, kind));
        connection.JobCompleted += (_, value) => cancelled = value;
        connection.Connect(transport);
        Assert.Single(sent);
        Assert.Equal(SendKind.Handshake, sent[0].Kind);

        // Reproduce boot delivery after Attach and before the first M110 ack.
        transport.Receive("start");
        transport.Receive("echo:Teacup virtual printer");
        transport.Receive("start");                  // A repeated boot banner is coalesced too
        Assert.Single(sent);
        FinishHandshake(transport, connection);

        connection.StartJob(new List<JobLine> { new("G1 X1", 0), new("G1 X2", 1) });
        string first = ResponseParser.FormatNumbered(4, "G1 X1");
        Assert.Equal(first, sent[^1].Line);
        transport.Receive("Error:checksum mismatch, Last Line: 3");
        transport.Receive("Resend: 4");
        transport.Receive("ok");                    // Resend trailer, not a job ack
        Assert.Null(cancelled);
        Assert.Equal(0, connection.JobAcknowledged);
        Assert.Equal((first, SendKind.Resend), sent[^1]);

        transport.Receive("ok");                    // Retried first job line
        Assert.Equal(1, connection.JobAcknowledged);
        Assert.Equal(ResponseParser.FormatNumbered(5, "G1 X2"), sent[^1].Line);
        transport.Receive("ok");                    // Second job line
        Assert.False(cancelled);
        Assert.Equal(2, connection.JobAcknowledged);
        Assert.Single(sent.Where(s => s.Kind == SendKind.Handshake));
    }

    [Fact]
    public void StartupCoalescingDoesNotHideAResetDuringPrinting()
    {
        using var transport = new ControlledTransport();
        using var connection = new PrinterConnection { TemperatureInterval = 0 };
        bool? cancelled = null;
        connection.JobCompleted += (_, value) => cancelled = value;
        connection.Connect(transport);
        FinishHandshake(transport, connection);
        connection.StartJob(new List<JobLine> { new("G1 X1", 0), new("G1 X2", 1) });

        transport.Receive("start");
        Assert.True(cancelled);
        Assert.False(connection.IsJobActive);
        Assert.Equal(ConnectionState.Connecting, connection.State);
    }

    [Fact]
    public async Task DiscardedInitialHandshakeIsStillRetriedAfterStartupBanner()
    {
        using var transport = new ControlledTransport();
        using var connection = new PrinterConnection { TemperatureInterval = 0 };
        int handshakes = 0;
        var retried = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        connection.LineSent += (_, kind) =>
        {
            if (kind == SendKind.Handshake && Interlocked.Increment(ref handshakes) >= 2)
                retried.TrySetResult(true);
        };
        connection.Connect(transport);
        transport.Receive("start");
        Assert.Equal(1, Volatile.Read(ref handshakes));
        // Model firmware discarding the original M110: provide no reply.
        await retried.Task.WaitAsync(TimeSpan.FromSeconds(10));
        FinishHandshake(transport, connection);
        Assert.True(Volatile.Read(ref handshakes) >= 2);
    }
}
