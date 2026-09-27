// MAX31865 RTD-to-digital converter model for the Teacup tests. Chip
// select from a GPIO (connect the CS pin to input 0), like TeacupW25Q.
// Registers: 00 configuration, 01/02 RTD (15 bit << 1 | fault), 03..06
// fault thresholds, 07 fault status; address bit 7 = write, reads and
// writes auto-increment. The RTD value follows Resistance (Ohm) and Rref,
// Fault sets the fault bit (cleared by writing config bit 1, if the
// fault is gone).
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class TeacupMAX31865 : ISPIPeripheral, IGPIOReceiver
    {
        public TeacupMAX31865()
        {
            Resistance = 108.0;
            Rref = 430.0;
            regs = new byte[8];
            regs[3] = 0xFF; regs[4] = 0xFF;          // High fault threshold.
        }

        public void OnGPIO(int number, bool value)
        {
            if(!value && !selected) { selected = true; count = 0; }
            else if(value && selected) { selected = false; }
        }

        public byte Transmit(byte data)
        {
            if(!selected) return 0xFF;
            byte result = 0xFF;
            if(count == 0)
            {
                write = (data & 0x80) != 0;
                address = data & 0x07;
            }
            else
            {
                if(write)
                {
                    if(address == 0)
                    {
                        regs[0] = (byte)(data & ~0x02);   // Clear bit is self clearing.
                        Config = data;
                        ConfigWrites++;
                        if((data & 0x02) != 0 && !Fault) regs[7] = 0;
                    }
                    else if(address >= 3 && address <= 6)
                    {
                        regs[address] = data;
                    }
                }
                else
                {
                    result = Read(address);
                    if(address == 2) Reads++;
                }
                address = (address + 1) & 0x07;
            }
            count++;
            return result;
        }

        private byte Read(int a)
        {
            var code = (int)Math.Round(Resistance / Rref * 32768.0);
            if(code > 0x7FFF) code = 0x7FFF;
            if(code < 0) code = 0;
            var rtd = (code << 1) | (Fault ? 1 : 0);
            if(Fault) regs[7] = 0x04;                 // REFIN- > 0.85 VBIAS (open).
            switch(a)
            {
            case 1: return (byte)(rtd >> 8);
            case 2: return (byte)rtd;
            default: return regs[a];
            }
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            selected = false;
            count = 0;
        }

        // Chip select state, for the bus model (TeacupSPIMux).
        public bool Selected => selected;
        public double Resistance { get; set; }
        public double Rref { get; set; }
        public bool Fault { get; set; }
        public byte Config { get; private set; }
        public int ConfigWrites { get; private set; }
        public int Reads { get; private set; }

        private readonly byte[] regs;
        private bool selected;
        private bool write;
        private int address;
        private int count;
    }
}
