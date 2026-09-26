// STM32F4 OTG_FS (Synopsys DWC2) device-mode model for the Teacup tests,
// together with a small USB host which drives it. Renode 1.16 has no model
// of this core.
//
// Device side (what the firmware sees): the registers of RM0368 / RM0383
// chapter 22 in device mode, slave (non-DMA) operation with dedicated TX
// FIFOs. Modelled at transaction level:
//   - RX FIFO as a queue of status entries (GRXSTSR/GRXSTSP) with their
//     data, read through the FIFO window. STUP and OUT XFRC interrupts are
//     raised when their status entry is popped, like the real core does.
//   - One TX FIFO per IN endpoint, filled word by word through the FIFO
//     window, sized by DIEPTXF0/DIEPTXFx (overflow is reported).
//   - EPENA/NAK/STALL handling, DIEPTSIZ/DOEPTSIZ decrement, XFRC at the
//     end of a transfer, SETUP clears STALL and sets NAK on EP0, a finished
//     OUT transfer disables the endpoint and sets NAK.
//   - Device address check on every transaction. SET_ADDRESS status stage
//     still works with the old address (DWC2 behaviour).
//
// Host side (what the tests use): enumeration like Linux does it, then a
// CDC ACM "terminal": the model implements IUART, so WriteChar() sends
// bytes to the device through bulk OUT (retrying on NAK) and everything
// read from bulk IN comes out through CharReceived. LoggingUartAnalyzer and
// socket terminals work as with a UART.
//
// Test helpers: Open()/Close() (DTR), Plug()/Unplug(), HostReads(bool),
// Control(name, setup_hex, length), Stats(). Protocol violations of the
// firmware are logged as errors starting with "PROTOCOL".
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.USB
{
    public class TeacupSTM32_OTGFS : IDoubleWordPeripheral, IKnownSize, IUART
    {
        public TeacupSTM32_OTGFS(IMachine machine)
        {
            this.machine = machine;
            IRQ = new GPIO();
            timer = new LimitTimer(machine.ClockSource, 1000000, this, "host",
                                   limit: TickUs, enabled: true, eventEnabled: true,
                                   autoUpdate: true);
            timer.LimitReached += HostTick;
            Reset();
        }

        public long Size => 0x40000;
        public GPIO IRQ { get; }

        // ------------------------------------------------------------ IUART
        public uint BaudRate => 115200;
        public Bits StopBits => Bits.One;
        public Parity ParityBit => Parity.None;
        [field: Antmicro.Migrant.Transient]
        public event Action<byte> CharReceived;

        public void WriteChar(byte value)
        {
            lock(sync) { hostTx.Enqueue(value); }
        }

        // ---------------------------------------------------- test helpers
        public void Unplug()
        {
            lock(sync)
            {
                cable = false;
                Detach("cable unplugged");
                // Idle bus for 3 ms: the core reports a suspend.
                dsts |= 1;
                latched |= GINT_USBSUSP;
                UpdateIRQ();
            }
        }

        public void Plug()
        {
            lock(sync) { cable = true; Info("cable plugged"); }
        }

        public void Open()
        {
            lock(sync) { QueueControl("SET_CONTROL_LINE_STATE 3", Setup(0x21, 0x22, 3, 0, 0), null, 0, c => { if(c.Ok) portOpen = true; }); }
        }

        public void Close()
        {
            lock(sync)
            {
                portOpen = false;
                QueueControl("SET_CONTROL_LINE_STATE 0", Setup(0x21, 0x22, 0, 0, 0), null, 0, null);
            }
        }

        public void HostReads(bool on)
        {
            lock(sync) { hostReads = on; Info("host reads: {0}", on); }
        }

        public void Control(string name, string setupHex, int length)
        {
            var setup = Hex(setupHex);
            lock(sync)
            {
                var isIn = (setup[0] & 0x80) != 0;
                byte[] data = null;
                if(!isIn && length > 0)
                {
                    data = new byte[length];
                }
                QueueControl(name, setup, data, length, null);
            }
        }

        public void Stats()
        {
            lock(sync)
            {
                Info("STATS bulk_out_naks={0} bulk_out_packets={1} bulk_in_packets={2} bulk_in_zlps={3} pending_tx={4} enumerations={5} stalls={6}",
                     outNaks, outPackets, inPackets, inZlps, hostTx.Count, enumerations, stallsSeen);
            }
        }

        // ------------------------------------------------------- reset
        public void Reset()
        {
            lock(sync)
            {
                gotgctl = 0x00010000;
                gotgint = 0;
                gahbcfg = 0;
                gusbcfg = 0x00000A00;
                gintmsk = 0;
                latched = 0x04000020;
                grxfsiz = 0x200;
                dieptxf0 = 0x02000200;
                gccfg = 0;
                for(var i = 0; i < 4; i++) { dieptxf[i] = (uint)(0x02000400 + 0x200 * i); }
                dcfg = 0x02200000;
                dctl = 0x00000002;
                dsts = 0x00000010;
                diepmsk = doepmsk = daintmsk = 0;
                diepempmsk = 0;
                pcgcctl = 0;
                for(var i = 0; i < NumEp; i++)
                {
                    inCtl[i] = outCtl[i] = 0;
                    inEna[i] = outEna[i] = false;
                    inNak[i] = outNak[i] = false;
                    inStall[i] = outStall[i] = false;
                    inInt[i] = outInt[i] = 0;
                    inTsiz[i] = outTsiz[i] = 0;
                    txFifo[i].Clear();
                }
                rxQueue.Clear();
                rxData.Clear();

                // Host: the device vanished (reset releases DP), a real host
                // re-enumerates. Data typed by the user stays queued.
                state = HostState.Detached;
                ctlQueue.Clear();
                current = null;
                portOpen = false;
                hostAddr = 0;
                configured = false;
                UpdateIRQ();
            }
            timer.Reset();
            timer.Enabled = true;
        }

        // ------------------------------------------------ register access
        public uint ReadDoubleWord(long offset)
        {
            lock(sync)
            {
                if(offset >= 0x1000 && offset < 0x1000 + 0x1000 * NumEp)
                {
                    return FifoRead();
                }
                switch(offset)
                {
                case 0x000: return gotgctl;
                case 0x004: return gotgint;
                case 0x008: return gahbcfg;
                case 0x00C: return gusbcfg | 0x40;              // PHYSEL reads 1
                case 0x010: return 0x80000000;                  // AHBIDL, resets are instant
                case 0x014: return Gintsts();
                case 0x018: return gintmsk;
                case 0x01C: return RxStatus(false);
                case 0x020: return RxStatus(true);
                case 0x024: return grxfsiz;
                case 0x028: return dieptxf0;
                case 0x02C: return 0;
                case 0x038: return gccfg;
                case 0x03C: return 0x00001200;
                case 0x100: return 0;
                case 0x800: return dcfg;
                case 0x804: return dctl;
                case 0x808: return dsts;
                case 0x810: return diepmsk;
                case 0x814: return doepmsk;
                case 0x818: return Daint();
                case 0x81C: return daintmsk;
                case 0x834: return diepempmsk;
                case 0xE00: return pcgcctl;
                }
                if(offset >= 0x104 && offset < 0x104 + 4 * 4)
                {
                    return dieptxf[(offset - 0x104) / 4];
                }
                if(offset >= 0x900 && offset < 0x900 + 0x20 * NumEp)
                {
                    var ep = (int)((offset - 0x900) / 0x20);
                    switch((offset - 0x900) % 0x20)
                    {
                    case 0x00: return InCtlValue(ep);
                    case 0x08: return inInt[ep] | (txFifo[ep].Count == 0 ? 0x80u : 0u);
                    case 0x10: return inTsiz[ep];
                    case 0x18: return (uint)Math.Max(0, TxDepth(ep) - txFifo[ep].Count);
                    }
                    return 0;
                }
                if(offset >= 0xB00 && offset < 0xB00 + 0x20 * NumEp)
                {
                    var ep = (int)((offset - 0xB00) / 0x20);
                    switch((offset - 0xB00) % 0x20)
                    {
                    case 0x00: return OutCtlValue(ep);
                    case 0x08: return outInt[ep];
                    case 0x10: return outTsiz[ep];
                    }
                    return 0;
                }
                this.Log(LogLevel.Warning, "Unhandled read at 0x{0:X}", offset);
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(sync)
            {
                if(offset >= 0x1000 && offset < 0x1000 + 0x1000 * NumEp)
                {
                    FifoWrite((int)((offset - 0x1000) / 0x1000), value);
                    UpdateIRQ();
                    return;
                }
                switch(offset)
                {
                case 0x000: gotgctl = value; break;
                case 0x004: gotgint &= ~value; break;
                case 0x008: gahbcfg = value; break;
                case 0x00C: gusbcfg = value; break;
                case 0x010: CoreReset(value); break;
                case 0x014: latched &= ~(value & GINT_RC_W1); break;
                case 0x018: gintmsk = value; break;
                case 0x024: grxfsiz = value; break;
                case 0x028: dieptxf0 = value; break;
                case 0x038: gccfg = value; break;
                case 0x800: dcfg = value; break;
                case 0x804:
                    dctl = value & ~0x780u;                     // SGINAK..CGONAK are write-only
                    break;
                case 0x810: diepmsk = value; break;
                case 0x814: doepmsk = value; break;
                case 0x81C: daintmsk = value; break;
                case 0x834: diepempmsk = value; break;
                case 0xE00: pcgcctl = value; break;
                default:
                    if(offset >= 0x104 && offset < 0x104 + 4 * 4)
                    {
                        dieptxf[(offset - 0x104) / 4] = value;
                    }
                    else if(offset >= 0x900 && offset < 0x900 + 0x20 * NumEp)
                    {
                        var ep = (int)((offset - 0x900) / 0x20);
                        switch((offset - 0x900) % 0x20)
                        {
                        case 0x00: InCtlWrite(ep, value); break;
                        case 0x08: inInt[ep] &= ~value; break;
                        case 0x10:
                            if(inEna[ep]) Protocol("DIEPTSIZ{0} written while EP{0} IN enabled", ep);
                            inTsiz[ep] = value;
                            break;
                        }
                    }
                    else if(offset >= 0xB00 && offset < 0xB00 + 0x20 * NumEp)
                    {
                        var ep = (int)((offset - 0xB00) / 0x20);
                        switch((offset - 0xB00) % 0x20)
                        {
                        case 0x00: OutCtlWrite(ep, value); break;
                        case 0x08: outInt[ep] &= ~value; break;
                        case 0x10: outTsiz[ep] = value; break;
                        }
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Unhandled write at 0x{0:X}, value 0x{1:X}", offset, value);
                    }
                    break;
                }
                UpdateIRQ();
            }
        }

        // ---------------------------------------------------- device core
        private uint Gintsts()
        {
            var v = latched;
            if((gusbcfg & (1u << 29)) != 0 && (gusbcfg & (1u << 30)) == 0)
            {
                v |= 1;                                         // CMOD host
            }
            if(rxQueue.Count > 0) v |= GINT_RXFLVL;
            var daint = Daint() & daintmsk;
            if((daint & 0xFFFF) != 0) v |= GINT_IEPINT;
            if((daint >> 16) != 0) v |= GINT_OEPINT;
            return v;
        }

        private uint Daint()
        {
            uint v = 0;
            for(var i = 0; i < NumEp; i++)
            {
                var txfe = txFifo[i].Count == 0 && (diepempmsk & (1u << i)) != 0 && inEna[i];
                if((inInt[i] & diepmsk) != 0 || txfe) v |= 1u << i;
                if((outInt[i] & doepmsk) != 0) v |= 1u << (16 + i);
            }
            return v;
        }

        private void UpdateIRQ()
        {
            IRQ.Set((gahbcfg & 1) != 0 && (Gintsts() & gintmsk) != 0);
        }

        private void CoreReset(uint value)
        {
            if((value & 1) != 0)                                // CSRST
            {
                latched = 0x04000020;
                rxQueue.Clear();
                rxData.Clear();
                for(var i = 0; i < NumEp; i++) txFifo[i].Clear();
            }
            if((value & (1u << 4)) != 0)                        // RXFFLSH
            {
                rxQueue.Clear();
                rxData.Clear();
            }
            if((value & (1u << 5)) != 0)                        // TXFFLSH
            {
                var num = (int)((value >> 6) & 0x1F);
                for(var i = 0; i < NumEp; i++)
                {
                    if(num == 0x10 || num == i) txFifo[i].Clear();
                }
            }
        }

        private uint InCtlValue(int ep)
        {
            var v = inCtl[ep];
            if(inEna[ep]) v |= 1u << 31;
            if(inNak[ep]) v |= 1u << 17;
            if(inStall[ep]) v |= 1u << 21;
            if(ep == 0) v |= 1u << 15;                          // USBAEP
            return v;
        }

        private uint OutCtlValue(int ep)
        {
            var v = outCtl[ep];
            if(outEna[ep]) v |= 1u << 31;
            if(outNak[ep]) v |= 1u << 17;
            if(outStall[ep]) v |= 1u << 21;
            if(ep == 0) v |= 1u << 15;
            return v;
        }

        // Bits stored as written: MPSIZ, USBAEP, EPTYP, TXFNUM.
        private const uint CtlKeep = 0x07CCFFFF & ~(1u << 17);

        private void InCtlWrite(int ep, uint v)
        {
            inCtl[ep] = v & CtlKeep & ~(1u << 21);
            if((v & (1u << 27)) != 0) inNak[ep] = true;         // SNAK
            if((v & (1u << 26)) != 0) inNak[ep] = false;        // CNAK
            if(ep == 0)
            {
                if((v & (1u << 21)) != 0) inStall[ep] = true;   // rs on EP0
            }
            else
            {
                inStall[ep] = (v & (1u << 21)) != 0;
            }
            if((v & (1u << 30)) != 0 && inEna[ep])              // EPDIS
            {
                inEna[ep] = false;
                inInt[ep] |= 2;
                return;
            }
            if((v & (1u << 31)) != 0)
            {
                if(ep != 0 && (inCtl[ep] & (1u << 15)) == 0)
                {
                    Protocol("EP{0} IN enabled but not active (USBAEP)", ep);
                }
                inEna[ep] = true;
            }
        }

        private void OutCtlWrite(int ep, uint v)
        {
            outCtl[ep] = v & CtlKeep & ~(1u << 21);
            if((v & (1u << 27)) != 0) outNak[ep] = true;
            if((v & (1u << 26)) != 0) outNak[ep] = false;
            if(ep == 0)
            {
                if((v & (1u << 21)) != 0) outStall[ep] = true;
            }
            else
            {
                outStall[ep] = (v & (1u << 21)) != 0;
            }
            if((v & (1u << 30)) != 0 && outEna[ep])
            {
                outEna[ep] = false;
                outInt[ep] |= 2;
                return;
            }
            if((v & (1u << 31)) != 0)
            {
                if(ep != 0 && (outCtl[ep] & (1u << 15)) == 0)
                {
                    Protocol("EP{0} OUT enabled but not active (USBAEP)", ep);
                }
                outEna[ep] = true;
            }
        }

        private int TxDepth(int ep)
        {
            var reg = ep == 0 ? dieptxf0 : dieptxf[ep - 1];
            return (int)(reg >> 16);
        }

        private void FifoWrite(int ep, uint value)
        {
            if(txFifo[ep].Count >= TxDepth(ep))
            {
                Protocol("TX FIFO {0} overflow (depth {1} words)", ep, TxDepth(ep));
                return;
            }
            if(!inEna[ep])
            {
                Protocol("TX FIFO {0} written while EP{0} IN disabled", ep);
            }
            txFifo[ep].Add(value);
        }

        private uint FifoRead()
        {
            if(rxData.Count == 0)
            {
                Protocol("RX FIFO read without data");
                return 0;
            }
            return rxData.Dequeue();
        }

        private uint RxStatus(bool pop)
        {
            if(rxQueue.Count == 0)
            {
                if(pop) Protocol("GRXSTSP read with empty RX FIFO");
                return 0;
            }
            var e = rxQueue.Peek();
            var v = (uint)e.Ep | ((uint)e.Data.Length << 4) | ((uint)e.PktSts << 17);
            if(!pop)
            {
                return v;
            }
            if(rxData.Count != 0)
            {
                Protocol("GRXSTSP popped before the previous packet's data was read ({0} words left)", rxData.Count);
                rxData.Clear();
            }
            rxQueue.Dequeue();
            for(var i = 0; i < e.Data.Length; i += 4)
            {
                uint w = 0;
                for(var k = 0; k < 4 && i + k < e.Data.Length; k++)
                {
                    w |= (uint)e.Data[i + k] << (8 * k);
                }
                rxData.Enqueue(w);
            }
            if(e.PktSts == PktSetupDone)
            {
                outInt[0] |= 1u << 3;                           // STUP
            }
            else if(e.PktSts == PktOutDone)
            {
                outInt[e.Ep] |= 1;                              // XFRC
            }
            return v;
        }

        private int RxFifoUsed()
        {
            return rxQueue.Sum(e => 1 + (e.Data.Length + 3) / 4) + rxData.Count;
        }

        private bool AddressMatch(int addr)
        {
            var dad = (int)((dcfg >> 4) & 0x7F);
            return addr == dad || (addressStatusStage && addr == oldAddr);
        }

        private bool Connected => cable && (gccfg & (1u << 16)) != 0 && (dctl & 2) == 0;

        // -------------------------------------------- bus transactions
        private enum Resp { Ack, Nak, Stall, Timeout }

        private Resp TxSetup(int addr, byte[] setup)
        {
            if(!AddressMatch(addr)) return Resp.Timeout;
            if(RxFifoUsed() + 4 > (grxfsiz & 0xFFFF))
            {
                Protocol("RX FIFO full at SETUP");
                return Resp.Timeout;
            }
            inStall[0] = outStall[0] = false;
            inNak[0] = outNak[0] = true;
            var stupcnt = (outTsiz[0] >> 29) & 3;
            if(stupcnt == 0)
            {
                Protocol("SETUP received with STUPCNT = 0");
            }
            else
            {
                outTsiz[0] = (outTsiz[0] & ~(3u << 29)) | ((stupcnt - 1) << 29);
            }
            // A new SETUP aborts a pending IN data stage.
            if(inEna[0])
            {
                inEna[0] = false;
                txFifo[0].Clear();
            }
            rxQueue.Enqueue(new RxEntry(0, PktSetupData, setup));
            rxQueue.Enqueue(new RxEntry(0, PktSetupDone, new byte[0]));
            UpdateIRQ();
            return Resp.Ack;
        }

        private Resp TxOut(int addr, int ep, byte[] data)
        {
            if(!AddressMatch(addr)) return Resp.Timeout;
            if(ep != 0 && (outCtl[ep] & (1u << 15)) == 0) return Resp.Timeout;
            if(outStall[ep]) return Resp.Stall;
            if(!outEna[ep] || outNak[ep]) return Resp.Nak;
            var mps = ep == 0 ? Ep0Mps() : (int)(outCtl[ep] & 0x7FF);
            if(data.Length > mps)
            {
                Protocol("OUT packet of {0} bytes > MPS {1}", data.Length, mps);
            }
            if(RxFifoUsed() + 2 + (data.Length + 3) / 4 > (grxfsiz & 0xFFFF))
            {
                return Resp.Nak;
            }
            var tsiz = outTsiz[ep];
            var pktMask = ep == 0 ? 1u : 0x3FFu;
            var sizeMask = ep == 0 ? 0x7Fu : 0x7FFFFu;
            var pktcnt = (tsiz >> 19) & pktMask;
            var xfrsiz = tsiz & sizeMask;
            if(pktcnt == 0)
            {
                Protocol("EP{0} OUT enabled with PKTCNT = 0", ep);
                return Resp.Nak;
            }
            if(data.Length > xfrsiz)
            {
                Protocol("EP{0} OUT: {1} bytes received, XFRSIZ only {2}", ep, data.Length, xfrsiz);
            }
            xfrsiz = (uint)Math.Max(0, (int)xfrsiz - data.Length);
            pktcnt--;
            outTsiz[ep] = (tsiz & ~((pktMask << 19) | sizeMask)) | (pktcnt << 19) | xfrsiz;
            rxQueue.Enqueue(new RxEntry(ep, PktOutData, data));
            if(data.Length < mps || pktcnt == 0)
            {
                outEna[ep] = false;
                outNak[ep] = true;
                rxQueue.Enqueue(new RxEntry(ep, PktOutDone, new byte[0]));
            }
            UpdateIRQ();
            return Resp.Ack;
        }

        private Resp TxIn(int addr, int ep, out byte[] data)
        {
            data = null;
            if(!AddressMatch(addr)) return Resp.Timeout;
            if(ep != 0 && (inCtl[ep] & (1u << 15)) == 0) return Resp.Timeout;
            if(inStall[ep]) return Resp.Stall;
            if(!inEna[ep] || inNak[ep]) return Resp.Nak;
            var tsiz = inTsiz[ep];
            var pktMask = ep == 0 ? 3u : 0x3FFu;
            var sizeMask = ep == 0 ? 0x7Fu : 0x7FFFFu;
            var pktcnt = (tsiz >> 19) & pktMask;
            var xfrsiz = tsiz & sizeMask;
            if(pktcnt == 0)
            {
                Protocol("EP{0} IN enabled with PKTCNT = 0", ep);
                return Resp.Nak;
            }
            var mps = ep == 0 ? Ep0Mps() : (int)(inCtl[ep] & 0x7FF);
            var n = (int)Math.Min((uint)mps, xfrsiz);
            var words = (n + 3) / 4;
            if(txFifo[ep].Count < words)
            {
                return Resp.Nak;                                // Data not written yet.
            }
            data = new byte[n];
            for(var i = 0; i < words; i++)
            {
                var w = txFifo[ep][0];
                txFifo[ep].RemoveAt(0);
                for(var k = 0; k < 4 && i * 4 + k < n; k++)
                {
                    data[i * 4 + k] = (byte)(w >> (8 * k));
                }
            }
            xfrsiz -= (uint)n;
            pktcnt--;
            inTsiz[ep] = (tsiz & ~((pktMask << 19) | sizeMask)) | (pktcnt << 19) | xfrsiz;
            if(pktcnt == 0)
            {
                if(xfrsiz != 0) Protocol("EP{0} IN: PKTCNT done with {1} bytes left", ep, xfrsiz);
                inEna[ep] = false;
                inInt[ep] |= 1;                                 // XFRC
            }
            UpdateIRQ();
            return Resp.Ack;
        }

        private int Ep0Mps()
        {
            switch(inCtl[0] & 3)
            {
            case 0: return 64;
            case 1: return 32;
            case 2: return 16;
            default: return 8;
            }
        }

        // ------------------------------------------------------ host
        private void HostTick()
        {
            lock(sync)
            {
                try
                {
                    HostStep();
                }
                catch(Exception e)
                {
                    this.Log(LogLevel.Error, "host exception: {0}", e);
                }
                UpdateIRQ();
            }
        }

        private void Detach(string why)
        {
            if(state != HostState.Detached)
            {
                Info("detached ({0})", why);
            }
            state = HostState.Detached;
            ctlQueue.Clear();
            current = null;
            portOpen = false;
            configured = false;
            hostAddr = 0;
        }

        private void HostStep()
        {
            if(!Connected)
            {
                if(state != HostState.Detached)
                {
                    Detach("device disconnected");
                }
                return;
            }
            switch(state)
            {
            case HostState.Detached:
                state = HostState.Debounce;
                countdown = 100000 / TickUs;                    // 100 ms
                return;
            case HostState.Debounce:
                if(--countdown > 0) return;
                Info("attached, bus reset");
                dsts &= ~1u;
                latched |= GINT_USBRST;
                state = HostState.Resetting;
                countdown = 10000 / TickUs;                     // 10 ms
                return;
            case HostState.Resetting:
                if(--countdown > 0) return;
                dsts = (dsts & ~6u) | (3u << 1);                // full speed
                latched |= GINT_ENUMDNE;
                state = HostState.Running;
                hostAddr = 0;
                enumerations++;
                StartEnumeration();
                return;
            }

            if(current == null && ctlQueue.Count > 0)
            {
                current = ctlQueue.Dequeue();
            }
            if(current != null)
            {
                ControlStep();
            }
            if(configured)
            {
                BulkStep();
            }
        }

        private void StartEnumeration()
        {
            QueueControl("GET_DESCRIPTOR DEVICE 64", Setup(0x80, 6, 0x0100, 0, 64), null, 64, null);
            QueueControl("SET_ADDRESS 7", Setup(0x00, 5, 7, 0, 0), null, 0, c =>
            {
                if(c.Ok) hostAddr = 7;
            });
            QueueControl("GET_DESCRIPTOR DEVICE", Setup(0x80, 6, 0x0100, 0, 18), null, 18, null);
            QueueControl("GET_DESCRIPTOR CONFIG 9", Setup(0x80, 6, 0x0200, 0, 9), null, 9, c =>
            {
                if(!c.Ok || c.In.Count < 4) return;
                var total = c.In[2] | (c.In[3] << 8);
                // Insert the full read right after this one.
                var rest = ctlQueue.ToList();
                ctlQueue.Clear();
                QueueControl("GET_DESCRIPTOR CONFIG", Setup(0x80, 6, 0x0200, 0, total), null, total, null);
                foreach(var r in rest) ctlQueue.Enqueue(r);
            });
            QueueControl("GET_DESCRIPTOR STRING 0", Setup(0x80, 6, 0x0300, 0, 255), null, 255, null);
            QueueControl("GET_DESCRIPTOR STRING 2", Setup(0x80, 6, 0x0302, 0x0409, 255), null, 255, null);
            QueueControl("GET_DESCRIPTOR STRING 1", Setup(0x80, 6, 0x0301, 0x0409, 255), null, 255, null);
            QueueControl("GET_DESCRIPTOR STRING 3", Setup(0x80, 6, 0x0303, 0x0409, 255), null, 255, null);
            QueueControl("GET_DESCRIPTOR DEVICE_QUALIFIER", Setup(0x80, 6, 0x0600, 0, 10), null, 10, null);
            QueueControl("GET_STATUS DEVICE", Setup(0x80, 0, 0, 0, 2), null, 2, null);
            QueueControl("SET_CONFIGURATION 1", Setup(0x00, 9, 1, 0, 0), null, 0, c =>
            {
                if(c.Ok) configured = true;
            });
            QueueControl("GET_CONFIGURATION", Setup(0x80, 8, 0, 0, 1), null, 1, null);
            // Like Linux cdc-acm when the tty is opened.
            QueueControl("SET_CONTROL_LINE_STATE 3", Setup(0x21, 0x22, 3, 0, 0), null, 0, c =>
            {
                if(c.Ok) portOpen = true;
            });
            QueueControl("SET_LINE_CODING", Setup(0x21, 0x20, 0, 0, 7),
                         new byte[] { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 }, 7, null);
            QueueControl("GET_LINE_CODING", Setup(0xA1, 0x21, 0, 0, 7), null, 7, null);
        }

        private static byte[] Setup(int type, int req, int value, int index, int length)
        {
            return new byte[] { (byte)type, (byte)req, (byte)value, (byte)(value >> 8),
                                (byte)index, (byte)(index >> 8), (byte)length, (byte)(length >> 8) };
        }

        private void QueueControl(string name, byte[] setup, byte[] outData, int length, Action<Ctl> done)
        {
            ctlQueue.Enqueue(new Ctl { Name = name, SetupPkt = setup, OutData = outData, Length = length, Done = done });
        }

        private void ControlStep()
        {
            var c = current;
            var isIn = (c.SetupPkt[0] & 0x80) != 0;
            Resp r;
            byte[] data;
            if(++c.Tries > 5000)
            {
                Finish(c, "TIMEOUT stage " + c.Stage);
                return;
            }
            switch(c.Stage)
            {
            case 0:
                r = TxSetup(hostAddr, c.SetupPkt);
                if(r != Resp.Ack)
                {
                    Finish(c, "SETUP " + r);
                    return;
                }
                c.Tries = 0;
                c.Stage = isIn && c.Length > 0 ? 1 : (!isIn && c.Length > 0 ? 2 : 3);
                if(c.SetupPkt[1] == 5 && c.SetupPkt[0] == 0)
                {
                    oldAddr = hostAddr;
                    addressStatusStage = true;
                }
                return;
            case 1:                                             // IN data
                r = TxIn(hostAddr, 0, out data);
                if(r == Resp.Nak) return;
                if(r != Resp.Ack)
                {
                    Finish(c, "DATA " + r.ToString().ToUpper());
                    return;
                }
                c.Tries = 0;
                c.In.AddRange(data);
                if(c.In.Count > c.Length)
                {
                    Protocol("{0}: {1} bytes returned, wLength {2}", c.Name, c.In.Count, c.Length);
                }
                if(data.Length < Ep0Mps() || c.In.Count >= c.Length)
                {
                    c.Stage = 4;
                }
                return;
            case 2:                                             // OUT data
            {
                var n = Math.Min(64, c.Length - c.OutPos);
                var pkt = new byte[n];
                Array.Copy(c.OutData, c.OutPos, pkt, 0, n);
                r = TxOut(hostAddr, 0, pkt);
                if(r == Resp.Nak) return;
                if(r != Resp.Ack)
                {
                    Finish(c, "DATA " + r.ToString().ToUpper());
                    return;
                }
                c.Tries = 0;
                c.OutPos += n;
                if(c.OutPos >= c.Length) c.Stage = 3;
                return;
            }
            case 3:                                             // status IN
                r = TxIn(addressStatusStage ? oldAddr : hostAddr, 0, out data);
                if(r == Resp.Nak) return;
                if(r != Resp.Ack)
                {
                    Finish(c, "STATUS " + r.ToString().ToUpper());
                    return;
                }
                if(data.Length != 0)
                {
                    Protocol("{0}: status stage IN with {1} bytes", c.Name, data.Length);
                }
                Finish(c, null);
                return;
            case 4:                                             // status OUT
                r = TxOut(hostAddr, 0, new byte[0]);
                if(r == Resp.Nak) return;
                if(r != Resp.Ack)
                {
                    Finish(c, "STATUS " + r.ToString().ToUpper());
                    return;
                }
                Finish(c, null);
                return;
            }
        }

        private void Finish(Ctl c, string error)
        {
            addressStatusStage = false;
            c.Ok = error == null;
            if(c.Ok)
            {
                Info("HOST {0} -> OK {1}", c.Name, string.Join(" ", c.In.Select(b => b.ToString("X2"))));
            }
            else
            {
                Info("HOST {0} -> {1}", c.Name, error);
            }
            current = null;
            c.Done?.Invoke(c);
        }

        private void BulkStep()
        {
            byte[] data;
            // Host -> device.
            if(hostTx.Count > 0)
            {
                var n = Math.Min(64, hostTx.Count);
                var pkt = hostTx.Take(n).ToArray();
                var r = TxOut(hostAddr, 1, pkt);
                if(r == Resp.Ack)
                {
                    for(var i = 0; i < n; i++) hostTx.Dequeue();
                    outPackets++;
                }
                else if(r == Resp.Nak)
                {
                    outNaks++;
                }
                else if(r == Resp.Stall)
                {
                    stallsSeen++;                               // Endpoint halted by request.
                }
                else
                {
                    Protocol("bulk OUT: {0}", r);
                }
            }
            // Device -> host, like cdc-acm with the tty open.
            if(portOpen && hostReads)
            {
                for(var k = 0; k < 2; k++)
                {
                    var r = TxIn(hostAddr, 1, out data);
                    if(r != Resp.Ack)
                    {
                        if(r == Resp.Stall) stallsSeen++;
                        else if(r != Resp.Nak) Protocol("bulk IN: {0}", r);
                        break;
                    }
                    inPackets++;
                    if(data.Length == 0) inZlps++;
                    foreach(var b in data) CharReceived?.Invoke(b);
                    if(data.Length < 64) break;
                }
            }
            // Notification endpoint, polled every 16 ms. Must never stall.
            if(++notifyCount >= 16000 / TickUs)
            {
                notifyCount = 0;
                var r = TxIn(hostAddr, 2, out data);
                if(r == Resp.Stall || r == Resp.Timeout) Protocol("interrupt IN: {0}", r);
            }
        }

        private void Info(string fmt, params object[] args)
        {
            this.Log(LogLevel.Info, fmt, args);
        }

        private void Protocol(string fmt, params object[] args)
        {
            this.Log(LogLevel.Error, "PROTOCOL " + fmt, args);
        }

        private static byte[] Hex(string s)
        {
            s = s.Replace(" ", "");
            var r = new byte[s.Length / 2];
            for(var i = 0; i < r.Length; i++) r[i] = Convert.ToByte(s.Substring(2 * i, 2), 16);
            return r;
        }

        private class RxEntry
        {
            public RxEntry(int ep, int pktSts, byte[] data) { Ep = ep; PktSts = pktSts; Data = data; }
            public int Ep;
            public int PktSts;
            public byte[] Data;
        }

        private class Ctl
        {
            public string Name;
            public byte[] SetupPkt;
            public byte[] OutData;
            public int Length;
            public int OutPos;
            public int Stage;
            public int Tries;
            public bool Ok;
            public List<byte> In = new List<byte>();
            public Action<Ctl> Done;
        }

        private enum HostState { Detached, Debounce, Resetting, Running }

        private const int NumEp = 4;
        private const int TickUs = 100;
        private const int PktOutData = 2, PktOutDone = 3, PktSetupDone = 4, PktSetupData = 6;
        private const uint GINT_RXFLVL = 1u << 4;
        private const uint GINT_USBSUSP = 1u << 11;
        private const uint GINT_USBRST = 1u << 12;
        private const uint GINT_ENUMDNE = 1u << 13;
        private const uint GINT_IEPINT = 1u << 18;
        private const uint GINT_OEPINT = 1u << 19;
        private const uint GINT_RC_W1 = 0xF030FC0Au;

        private readonly IMachine machine;
        private readonly LimitTimer timer;
        private readonly object sync = new object();

        private uint gotgctl, gotgint, gahbcfg, gusbcfg, gintmsk, latched, grxfsiz, dieptxf0, gccfg;
        private readonly uint[] dieptxf = new uint[4];
        private uint dcfg, dctl, dsts, diepmsk, doepmsk, daintmsk, diepempmsk, pcgcctl;
        private readonly uint[] inCtl = new uint[NumEp], outCtl = new uint[NumEp];
        private readonly bool[] inEna = new bool[NumEp], outEna = new bool[NumEp];
        private readonly bool[] inNak = new bool[NumEp], outNak = new bool[NumEp];
        private readonly bool[] inStall = new bool[NumEp], outStall = new bool[NumEp];
        private readonly uint[] inInt = new uint[NumEp], outInt = new uint[NumEp];
        private readonly uint[] inTsiz = new uint[NumEp], outTsiz = new uint[NumEp];
        private readonly List<uint>[] txFifo = Enumerable.Range(0, NumEp).Select(_ => new List<uint>()).ToArray();
        private readonly Queue<RxEntry> rxQueue = new Queue<RxEntry>();
        private readonly Queue<uint> rxData = new Queue<uint>();

        private HostState state;
        private int countdown;
        private bool cable = true;
        private bool configured;
        private bool portOpen;
        private bool hostReads = true;
        private int hostAddr, oldAddr;
        private bool addressStatusStage;
        private Ctl current;
        private readonly Queue<Ctl> ctlQueue = new Queue<Ctl>();
        private readonly Queue<byte> hostTx = new Queue<byte>();
        private int notifyCount;
        private long outNaks, outPackets, inPackets, inZlps, enumerations, stallsSeen;
    }
}
