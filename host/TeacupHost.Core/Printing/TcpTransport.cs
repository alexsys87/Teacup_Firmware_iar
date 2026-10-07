using System.Net.Sockets;
using System.Text;

namespace TeacupHost.Core.Printing;

/// <summary>
/// Network link: a Telnet / raw TCP serial bridge in front of the printer's
/// UART (ESP3D, ESP-Link, ser2net, a WiFi-UART module). Telnet option
/// negotiation is answered with "no" to everything, so the link stays a
/// plain 8-bit byte stream; IAC sequences never reach the protocol.
/// </summary>
public sealed class TcpTransport : IPrinterTransport
{
    private const byte Iac = 255, Dont = 254, Do = 253, Wont = 252, Will = 251, Sb = 250, Se = 240;

    private readonly string _host;
    private readonly int _port;
    private readonly int _connectTimeoutMs;
    private readonly object _writeLock = new();
    private TcpClient? _client;
    private NetworkStream? _stream;
    private Thread? _reader;
    private volatile bool _running;

    public TcpTransport(string host, int port, int connectTimeoutMs = 3000)
    {
        _host = host.Trim();
        _port = port;
        _connectTimeoutMs = connectTimeoutMs;
    }

    public string Name => $"{_host}:{_port}";
    public bool IsOpen => _running;

    public event Action<string>? LineReceived;
    public event Action<Exception>? Faulted;

    /// <summary>"host", "host:port" or "[v6]:port"; the default port is Telnet's 23.</summary>
    public static (string Host, int Port) ParseAddress(string text, int defaultPort = 23)
    {
        text = text.Trim();
        if (text.StartsWith('['))
        {
            int close = text.IndexOf(']');
            if (close > 0)
            {
                string h = text[1..close];
                return text.Length > close + 2 && text[close + 1] == ':' &&
                       int.TryParse(text[(close + 2)..], out int p6) ? (h, p6) : (h, defaultPort);
            }
        }
        int colon = text.LastIndexOf(':');
        if (colon > 0 && text.IndexOf(':') == colon && int.TryParse(text[(colon + 1)..], out int p))
            return (text[..colon], p);
        return (text, defaultPort);
    }

    public void Open()
    {
        var client = new TcpClient { NoDelay = true };
        try
        {
            if (!client.ConnectAsync(_host, _port).Wait(_connectTimeoutMs))
                throw new TimeoutException($"No answer from {Name}");
            // Notice a dead network (no RST) within about 10 s.
            var s = client.Client;
            s.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.KeepAlive, true);
            try
            {
                s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveTime, 5);
                s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveInterval, 1);
                s.SetSocketOption(SocketOptionLevel.Tcp, SocketOptionName.TcpKeepAliveRetryCount, 5);
            }
            catch (SocketException)
            {
                // Older systems: the default keep-alive timing.
            }
        }
        catch (AggregateException ex) when (ex.InnerException != null)
        {
            client.Dispose();
            throw ex.InnerException;
        }
        catch
        {
            client.Dispose();
            throw;
        }
        _client = client;
        _stream = client.GetStream();
        _running = true;
        _reader = new Thread(ReadLoop) { IsBackground = true, Name = "TCP reader " + Name };
        _reader.Start();
    }

    private void ReadLoop()
    {
        var stream = _stream!;
        var line = new StringBuilder();
        var buf = new byte[4096];
        // Telnet parser: 0 data, 1 after IAC, 2 option byte of DO/DONT/WILL/WONT,
        // 3 inside a subnegotiation, 4 IAC inside a subnegotiation.
        int state = 0;
        byte verb = 0;
        try
        {
            while (_running)
            {
                int n = stream.Read(buf, 0, buf.Length);
                if (n == 0)
                    throw new IOException("Connection closed by " + Name);
                for (int i = 0; i < n; i++)
                {
                    byte b = buf[i];
                    switch (state)
                    {
                        case 0:
                            if (b == Iac)
                                state = 1;
                            else
                                Data(b, line);
                            break;
                        case 1:
                            if (b == Iac)
                            {
                                Data(b, line);          // Escaped 0xFF.
                                state = 0;
                            }
                            else if (b is Do or Dont or Will or Wont)
                            {
                                verb = b;
                                state = 2;
                            }
                            else
                            {
                                state = b == Sb ? 3 : 0;
                            }
                            break;
                        case 2:
                            // Refuse every option: DO → WONT, WILL → DONT.
                            if (verb == Do)
                                Send(new[] { Iac, Wont, b });
                            else if (verb == Will)
                                Send(new[] { Iac, Dont, b });
                            state = 0;
                            break;
                        case 3:
                            if (b == Iac)
                                state = 4;
                            break;
                        case 4:
                            state = b == Se ? 0 : 3;
                            break;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            if (_running)
            {
                _running = false;
                Faulted?.Invoke(ex);
            }
        }
    }

    private void Data(byte b, StringBuilder line)
    {
        char c = (char)b;
        if (c == '\n' || c == '\r')
        {
            if (line.Length == 0)
                return;
            string text = line.ToString();
            line.Clear();
            try
            {
                LineReceived?.Invoke(text);
            }
            catch
            {
                // A faulty handler must not kill the reader.
            }
        }
        else if (c != '\0')
        {
            line.Append(c);
        }
    }

    private void Send(byte[] bytes)
    {
        lock (_writeLock)
        {
            try
            {
                _stream?.Write(bytes, 0, bytes.Length);
            }
            catch (Exception ex)
            {
                if (_running)
                {
                    _running = false;
                    Faulted?.Invoke(ex);
                }
            }
        }
    }

    public void WriteLine(string line)
    {
        // Lines are ASCII (PrinterConnection.ToAscii), no 0xFF to escape.
        Send(Encoding.ASCII.GetBytes(line + "\n"));
    }

    public void Close()
    {
        _running = false;
        try
        {
            _client?.Client.Shutdown(SocketShutdown.Both);
        }
        catch
        {
            // Already gone.
        }
        _client?.Close();
        _reader?.Join(1000);
        _reader = null;
    }

    public void Dispose()
    {
        Close();
        _client?.Dispose();
        _client = null;
    }
}
