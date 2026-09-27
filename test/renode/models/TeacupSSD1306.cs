// SSD1306 / SH1106 OLED model for the Teacup tests: 8 pages of 132
// columns of RAM, page addressing mode (also horizontal mode for the
// clear of older code). Each I2C transmission starts with a control byte:
// 0x00 commands follow, 0x40 data. Renode's STM32F4_I2C hands over one
// transmission per Write() (and doesn't call FinishTransmission()), so
// each Write() starts with the control byte. Attach with
//   oled: I2C.TeacupSSD1306 @ i2c1 0x3C
// "sysbus.i2c1.oled Dump" returns the RAM as hex, one page per line
// (the test decodes the characters with the firmware's font).
using System;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class TeacupSSD1306 : II2CPeripheral
    {
        public TeacupSSD1306()
        {
            Reset();
        }

        public void Write(byte[] data)
        {
            Writes++;
            expectControl = true;
            foreach(var b in data)
            {
                if(expectControl)
                {
                    dataMode = (b & 0x40) != 0;
                    expectControl = false;
                    continue;
                }
                if(dataMode)
                {
                    if(page < 8 && column < 132) ram[page, column] = b;
                    column++;
                    DataBytes++;
                    if(horizontal && column > colEnd)
                    {
                        column = colStart;
                        page = page >= pageEnd ? pageStart : page + 1;
                    }
                }
                else
                {
                    Command(b);
                }
            }
        }

        private void Command(byte b)
        {
            if(paramsLeft > 0)
            {
                paramsLeft--;
                switch(lastCommand)
                {
                case 0x20: horizontal = (b & 3) == 0; break;
                case 0x21: if(paramsLeft == 1) colStart = column = b; else colEnd = b; break;
                case 0x22: if(paramsLeft == 1) pageStart = page = b & 7; else pageEnd = b & 7; break;
                case 0xA8: Multiplex = b + 1; break;
                }
                return;
            }
            lastCommand = b;
            if(b >= 0xB0 && b <= 0xB7) { page = b & 7; return; }
            if(b <= 0x0F) { column = (column & 0xF0) | b; return; }
            if(b >= 0x10 && b <= 0x1F) { column = (column & 0x0F) | ((b & 0x0F) << 4); return; }
            switch(b)
            {
            case 0x20: case 0xA8: case 0xD3: case 0xD5: case 0xDA: case 0x81:
            case 0xD9: case 0xDB: case 0x8D:
                paramsLeft = 1; break;
            case 0x21: case 0x22:
                paramsLeft = 2; break;
            case 0xAF: DisplayOn = true; break;
            case 0xAE: DisplayOn = false; break;
            }
            Commands++;
        }

        public byte[] Read(int count = 1)
        {
            return new byte[count];
        }

        public void FinishTransmission()
        {
            expectControl = true;
        }

        public void Reset()
        {
            ram = new byte[8, 132];
            expectControl = true;
            page = column = 0;
            colStart = 0; colEnd = 127; pageStart = 0; pageEnd = 7;
            horizontal = false;
            DisplayOn = false;
        }

        public string Dump()
        {
            var sb = new StringBuilder();
            for(var p = 0; p < 8; p++)
            {
                for(var c = 0; c < 132; c++) sb.Append(ram[p, c].ToString("X2"));
                sb.Append('\n');
            }
            return sb.ToString();
        }

        public bool DisplayOn { get; private set; }
        public int Multiplex { get; private set; }
        public int Commands { get; private set; }
        public int DataBytes { get; private set; }
        public int Writes { get; private set; }

        private byte[,] ram;
        private bool expectControl, dataMode, horizontal;
        private int page, column, colStart, colEnd, pageStart, pageEnd;
        private int paramsLeft;
        private byte lastCommand;
    }
}
