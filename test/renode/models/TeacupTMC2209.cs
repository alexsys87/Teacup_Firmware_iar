// TMC2209 model for the Teacup tests: up to four drivers (addresses 0..3)
// on one single wire UART. Datagrams as in the TMC2209 datasheet:
//   write 0x05 addr reg|0x80 d3 d2 d1 d0 crc, read 0x05 addr reg crc,
//   reply 0x05 0xFF reg d3 d2 d1 d0 crc (after 8 bit times).
// Every character is echoed first, like the MCU hears itself on the wire.
// Valid writes count in IFCNT, GSTAT bits are cleared by writing 1.
//
// The model sits on the system bus only to be addressable from the
// monitor (the MCU never accesses that address), connect it to a USART
// through a UART hub:
//   tmc: UART.TeacupTMC2209 @ sysbus <0x50100000, +0x100>
//   emulation CreateUARTHub "tmchub"
//   connector Connect sysbus.usart6 tmchub
//   connector Connect sysbus.tmc tmchub
// Monitor: "sysbus.tmc Reg 0 0x10", "sysbus.tmc PowerCycle 1",
// "sysbus.tmc SetPresent 3 false", "sysbus.tmc SetDrvFlags 0 0x1".
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    public class TeacupTMC2209 : IUART, IDoubleWordPeripheral, IKnownSize
    {
        public TeacupTMC2209(IMachine machine, uint baudRate = 115200)
        {
            this.machine = machine;
            this.baudRate = baudRate;
            for(var a = 0; a < 4; a++)
            {
                regs[a] = new Dictionary<byte, uint>();
                present[a] = true;
            }
            Reset();
        }

        public void Reset()
        {
            rx.Clear();
            for(var a = 0; a < 4; a++)
            {
                PowerCycle(a);
            }
        }

        // Character from the MCU.
        public void WriteChar(byte value)
        {
            Send(new byte[] { value }, 0);     // Echo on the wire.
            rx.Add(value);
            if(rx.Count > 8)
            {
                rx.RemoveAt(0);
            }
            var n = rx.Count;
            if(n >= 8 && rx[n - 8] == 0x05 && rx[n - 8 + 1] < 4 && (rx[n - 8 + 2] & 0x80) != 0
               && Crc(rx, n - 8, 7) == rx[n - 1])
            {
                var a = rx[n - 7];
                var reg = (byte)(rx[n - 6] & 0x7F);
                var v = ((uint)rx[n - 5] << 24) | ((uint)rx[n - 4] << 16) | ((uint)rx[n - 3] << 8) | rx[n - 2];
                rx.Clear();
                if(present[a])
                {
                    WriteReg(a, reg, v);
                }
                return;
            }
            if(n >= 4 && rx[n - 4] == 0x05 && rx[n - 3] < 4 && (rx[n - 2] & 0x80) == 0
               && Crc(rx, n - 4, 3) == rx[n - 1])
            {
                var a = rx[n - 3];
                var reg = rx[n - 2];
                rx.Clear();
                if(!present[a])
                {
                    return;
                }
                var v = ReadReg(a, reg);
                var reply = new List<byte> { 0x05, 0xFF, reg, (byte)(v >> 24), (byte)(v >> 16), (byte)(v >> 8), (byte)v };
                reply.Add(Crc(reply, 0, 7));
                Reads++;
                Send(reply.ToArray(), 8);
            }
        }

        public uint Reg(int addr, int reg)
        {
            return ReadReg(addr, (byte)reg, false);
        }

        // Driver lost its supply: registers back to reset values.
        public void PowerCycle(int addr)
        {
            var r = regs[addr];
            r.Clear();
            r[0x00] = 0x00000001;              // GCONF: I_scale_analog
            r[0x01] = 0x00000001;              // GSTAT: reset
            r[0x02] = 0;                       // IFCNT
            r[0x06] = 0x21000000;              // IOIN: version 0x21
            r[0x10] = 0x00011F10;              // IHOLD_IRUN
            r[0x11] = 20;                      // TPOWERDOWN
            r[0x13] = 0;                       // TPWMTHRS
            r[0x6C] = 0x10000053;              // CHOPCONF
            r[0x70] = 0xC10D0024;              // PWMCONF
            drvFlags[addr] = 0;
        }

        public void SetPresent(int addr, bool on)
        {
            present[addr] = on;
        }

        // DRV_STATUS flag bits 0..11 (otpw, ot, s2g, ola, ...).
        public void SetDrvFlags(int addr, uint flags)
        {
            drvFlags[addr] = flags & 0xFFF;
        }

        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 0x100;
        public uint Writes { get; private set; }
        public uint Reads { get; private set; }
        public uint BaudRate => baudRate;
        public Bits StopBits => Bits.One;
        public Parity ParityBit => Parity.None;

        [field: Antmicro.Migrant.Transient]
        public event Action<byte> CharReceived;

        private void WriteReg(int a, byte reg, uint v)
        {
            var r = regs[a];
            Writes++;
            r[0x02] = (r[0x02] + 1) & 0xFF;     // IFCNT
            switch(reg)
            {
                case 0x01:                     // GSTAT: write 1 to clear
                    r[0x01] &= ~v & 0x7;
                    break;
                case 0x02:
                case 0x06:
                case 0x6F:
                    break;                     // read only
                default:
                    r[reg] = v;
                    break;
            }
            this.Log(LogLevel.Debug, "driver {0} reg 0x{1:X2} = 0x{2:X8}", a, reg, v);
        }

        private uint ReadReg(int a, byte reg, bool count = true)
        {
            var r = regs[a];
            if(reg == 0x6F)                    // DRV_STATUS
            {
                var ihold_irun = r[0x10];
                var cs = (ihold_irun >> 8) & 0x1F;
                var stealth = (r[0x00] & 0x4) == 0 ? 1u : 0u;
                return (1u << 31) | (stealth << 30) | (cs << 16) | drvFlags[a];
            }
            return r.TryGetValue(reg, out var v) ? v : 0;
        }

        // Characters with one character time (10 bits) spacing, the first
        // one after 'delayBits'.
        private void Send(byte[] data, int delayBits)
        {
            var bitUs = 1000000.0 / baudRate;
            for(var i = 0; i < data.Length; i++)
            {
                var b = data[i];
                var us = (ulong)((delayBits + 10 * (i + 1)) * bitUs);
                machine.ScheduleAction(TimeInterval.FromMicroseconds(us), _ => CharReceived?.Invoke(b));
            }
        }

        private static byte Crc(IList<byte> d, int start, int n)
        {
            byte crc = 0;
            for(var i = 0; i < n; i++)
            {
                var b = d[start + i];
                for(var j = 0; j < 8; j++)
                {
                    if(((crc >> 7) ^ (b & 1)) != 0)
                        crc = (byte)((crc << 1) ^ 0x07);
                    else
                        crc = (byte)(crc << 1);
                    b >>= 1;
                }
            }
            return crc;
        }

        private readonly IMachine machine;
        private readonly uint baudRate;
        private readonly List<byte> rx = new List<byte>();
        private readonly Dictionary<byte, uint>[] regs = new Dictionary<byte, uint>[4];
        private readonly bool[] present = new bool[4];
        private readonly uint[] drvFlags = new uint[4];
    }
}
