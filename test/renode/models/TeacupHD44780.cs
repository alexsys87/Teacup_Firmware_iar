// HD44780 character LCD behind a PCF8574 I2C backpack, for the Teacup
// tests. Backpack pins: P0 RS, P1 RW, P2 E, P3 backlight, P4..P7 D4..D7.
// A falling edge of E latches a nibble; before the function set with
// DL = 0 each nibble is an 8 bit instruction (initialisation), then two
// nibbles make a byte. DDRAM with auto increment. Attach with
//   lcd: I2C.TeacupHD44780 @ i2c1 0x27
//       cols: 20
//       lines: 4
// "sysbus.i2c1.lcd Text" returns the lines separated by '|'.
using System;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class TeacupHD44780 : II2CPeripheral
    {
        public TeacupHD44780(int cols = 20, int lines = 4)
        {
            this.cols = cols;
            this.lines = lines;
            Reset();
        }

        public void Write(byte[] data)
        {
            foreach(var b in data)
            {
                if((pins & 0x04) != 0 && (b & 0x04) == 0)
                {
                    Nibble((byte)(pins >> 4), (pins & 0x01) != 0);
                }
                pins = b;
                Backlight = (b & 0x08) != 0;
            }
        }

        private void Nibble(byte n, bool rs)
        {
            if(!fourBit)
            {
                Execute((byte)(n << 4), rs);
                return;
            }
            if(!haveHigh)
            {
                high = n;
                haveHigh = true;
                return;
            }
            haveHigh = false;
            Execute((byte)((high << 4) | n), rs);
        }

        private void Execute(byte v, bool rs)
        {
            if(rs)
            {
                if(!cgram)
                {
                    ddram[address & 0x7F] = v;
                    address = (address + 1) & 0x7F;
                }
                Characters++;
                return;
            }
            Instructions++;
            if(v >= 0x80) { address = v & 0x7F; cgram = false; }
            else if(v >= 0x40) { cgram = true; }
            else if(v >= 0x20) { fourBit = (v & 0x10) == 0; TwoLines = (v & 0x08) != 0; }
            else if(v >= 0x08) { DisplayOn = (v & 0x04) != 0; }
            else if(v == 0x01) { for(var i = 0; i < ddram.Length; i++) ddram[i] = 0x20; address = 0; cgram = false; Clears++; }
            else if(v >= 0x02 && v <= 0x03) { address = 0; }
        }

        public byte[] Read(int count = 1)
        {
            var r = new byte[count];
            for(var i = 0; i < count; i++) r[i] = pins;
            return r;
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            ddram = new byte[128];
            for(var i = 0; i < ddram.Length; i++) ddram[i] = 0x20;
            pins = 0xFF;
            fourBit = false;
            haveHigh = false;
            address = 0;
            DisplayOn = false;
        }

        public string Text
        {
            get
            {
                int[] start = { 0x00, 0x40, cols, 0x40 + cols };
                var sb = new StringBuilder();
                for(var l = 0; l < lines; l++)
                {
                    for(var c = 0; c < cols; c++)
                    {
                        var ch = ddram[(start[l] + c) & 0x7F];
                        sb.Append(ch >= 0x20 && ch < 0x7F ? (char)ch : '?');
                    }
                    sb.Append('|');
                }
                return sb.ToString();
            }
        }

        public bool DisplayOn { get; private set; }
        public bool TwoLines { get; private set; }
        public bool Backlight { get; private set; }
        public int Instructions { get; private set; }
        public int Characters { get; private set; }
        public int Clears { get; private set; }

        private readonly int cols, lines;
        private byte[] ddram;
        private byte pins, high;
        private bool fourBit, haveHigh, cgram;
        private int address;
    }
}
