using System.IO.Ports;
using System.Text;

namespace TeacupHost.Core.Printing;

/// <summary>A line-based link to a printer: a COM port or the virtual printer.</summary>
public interface IPrinterTransport : IDisposable
{
    string Name { get; }
    bool IsOpen { get; }

    /// <summary>A complete line from the printer, without EOL. Raised on a background thread.</summary>
    event Action<string>? LineReceived;

    /// <summary>The link broke (cable pulled, port gone).</summary>
    event Action<Exception>? Faulted;

    void Open();
    void Close();

    /// <summary>Send one line; the transport adds "\n".</summary>
    void WriteLine(string line);
}

/// <summary>COM port (USB CDC of the Black Pill or a USB-UART adapter).</summary>
public sealed class SerialPortTransport : IPrinterTransport
{
    private readonly SerialPort _port;
    private Stream? _stream;
    private Thread? _reader;
    private volatile bool _running;
    private readonly object _writeLock = new();

    public SerialPortTransport(string portName, int baudRate)
    {
        _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
        {
            Encoding = Encoding.ASCII,
            NewLine = "\n",
            ReadTimeout = 200,
            WriteTimeout = 2000,
            // The firmware drops USB output while DTR is off (port "closed").
            DtrEnable = true,
            RtsEnable = true,
            Handshake = Handshake.None,
            ReadBufferSize = 65536,
            WriteBufferSize = 16384,
        };
    }

    public string Name => _port.PortName;
    public bool IsOpen => _port.IsOpen;

    public event Action<string>? LineReceived;
    public event Action<Exception>? Faulted;

    public static string[] GetPortNames()
    {
        try
        {
            return SerialPort.GetPortNames().Distinct().OrderBy(p => p.Length).ThenBy(p => p).ToArray();
        }
        catch
        {
            return [];
        }
    }

    public void Open()
    {
        _port.Open();
        // When a USB serial device is unplugged, the finalizer of the port's
        // stream can throw on the finalizer thread and end the process. The
        // stream is closed explicitly in Close() instead.
        _stream = _port.BaseStream;
        GC.SuppressFinalize(_stream);
        _port.DiscardInBuffer();
        _port.DiscardOutBuffer();
        _running = true;
        _reader = new Thread(ReadLoop) { IsBackground = true, Name = "Serial reader " + _port.PortName };
        _reader.Start();
    }

    private void ReadLoop()
    {
        var sb = new StringBuilder();
        var buf = new byte[4096];
        while (_running)
        {
            int n;
            try
            {
                n = _port.Read(buf, 0, buf.Length);
            }
            catch (TimeoutException)
            {
                continue;
            }
            catch (Exception ex)
            {
                if (_running)
                {
                    _running = false;
                    Faulted?.Invoke(ex);
                }
                return;
            }
            for (int i = 0; i < n; i++)
            {
                char c = (char)buf[i];
                if (c == '\n' || c == '\r')
                {
                    if (sb.Length > 0)
                    {
                        string line = sb.ToString();
                        sb.Clear();
                        try
                        {
                            LineReceived?.Invoke(line);
                        }
                        catch
                        {
                            // A faulty handler must not kill the reader.
                        }
                    }
                }
                else if (c != '\0')
                {
                    sb.Append(c);
                }
            }
        }
    }

    public void WriteLine(string line)
    {
        lock (_writeLock)
        {
            try
            {
                var bytes = Encoding.ASCII.GetBytes(line + "\n");
                _port.Write(bytes, 0, bytes.Length);
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

    public void Close()
    {
        _running = false;
        try
        {
            _stream?.Close();
        }
        catch
        {
            // Port already gone (USB unplugged).
        }
        try
        {
            if (_port.IsOpen)
                _port.Close();
        }
        catch
        {
            // Port already gone (USB unplugged).
        }
        _stream = null;
        _reader?.Join(1000);
        _reader = null;
    }

    public void Dispose()
    {
        Close();
        _port.Dispose();
    }
}
