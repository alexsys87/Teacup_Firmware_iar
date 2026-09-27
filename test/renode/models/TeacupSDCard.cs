// SD card (SDHC, SPI mode) for the Teacup tests. Chip select from a GPIO
// (connect the CS pin to input 0), like TeacupW25Q. Contents from an image
// file (FAT, e.g. made with mkfs.fat and mcopy), read only.
//
// Commands: CMD0 (enters SPI mode, needs CRC 0x95), CMD8 (CRC 0x87),
// CMD55 + ACMD41 (ready after BusyCount of them), CMD58 (OCR with CCS:
// block addressing), CMD16, CMD17 (read one 512 byte block). Others are
// illegal. Before CMD0 the card is in SD mode and doesn't answer.
// Inserted = false: no card, MISO stays high.
using System;
using System.Collections.Generic;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class TeacupSDCard : ISPIPeripheral, IGPIOReceiver
    {
        public TeacupSDCard(string imageFile = "")
        {
            image = imageFile != "" ? File.ReadAllBytes(imageFile) : new byte[1024 * 1024];
            Inserted = true;
            BusyCount = 5;
        }

        public void OnGPIO(int number, bool value)
        {
            if(!value && !selected) { selected = true; pos = 0; }
            else if(value && selected) { selected = false; pos = 0; output.Clear(); }
        }

        public byte Transmit(byte data)
        {
            if(!selected || !Inserted) return 0xFF;
            byte result = output.Count > 0 ? output.Dequeue() : (byte)0xFF;
            if(pos == 0)
            {
                if((data & 0xC0) != 0x40) return result;    // Not a command start.
                output.Clear();
            }
            frame[pos++] = data;
            if(pos == 6)
            {
                pos = 0;
                Execute();
            }
            return result;
        }

        private void Execute()
        {
            var cmd = frame[0] & 0x3F;
            var arg = (uint)(frame[1] << 24 | frame[2] << 16 | frame[3] << 8 | frame[4]);
            var app = appCmd;
            appCmd = false;
            Commands++;
            if(!spiMode)
            {
                // SD mode: only CMD0 with CS low switches to SPI mode.
                if(cmd != 0) return;
            }
            output.Enqueue(0xFF);                          // Ncr: one byte.
            byte idle = (byte)(ready ? 0x00 : 0x01);
            if((cmd == 0 && frame[5] != 0x95) || (cmd == 8 && frame[5] != 0x87))
            {
                output.Enqueue((byte)(idle | 0x08));        // CRC error.
                return;
            }
            switch(cmd)
            {
            case 0:
                spiMode = true;
                ready = false;
                busyLeft = BusyCount;
                output.Enqueue(0x01);
                break;
            case 8:
                output.Enqueue(idle);
                output.Enqueue(0x00); output.Enqueue(0x00);
                output.Enqueue((byte)((arg >> 8) & 0x0F)); output.Enqueue((byte)arg);
                break;
            case 55:
                appCmd = true;
                output.Enqueue(idle);
                break;
            case 41:
                if(!app) { output.Enqueue((byte)(idle | 0x04)); break; }
                if(busyLeft > 0) busyLeft--;
                ready = busyLeft == 0;
                AcmdCount++;
                output.Enqueue((byte)(ready ? 0x00 : 0x01));
                break;
            case 58:
                output.Enqueue(idle);
                output.Enqueue((byte)(ready ? 0xC0 : 0x40)); output.Enqueue(0xFF);
                output.Enqueue(0x80); output.Enqueue(0x00);
                break;
            case 16:
                output.Enqueue(idle);
                break;
            case 17:
                if(!ready) { output.Enqueue((byte)(idle | 0x04)); break; }
                if((long)arg * 512 + 512 > image.Length) { output.Enqueue(0x40); break; }
                output.Enqueue(0x00);
                output.Enqueue(0xFF); output.Enqueue(0xFF);  // Access time.
                output.Enqueue(0xFE);                        // Data token.
                for(var i = 0; i < 512; i++) output.Enqueue(image[arg * 512 + i]);
                output.Enqueue(0xFF); output.Enqueue(0xFF);  // CRC.
                Reads++;
                break;
            default:
                output.Enqueue((byte)(idle | 0x04));         // Illegal command.
                break;
            }
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            selected = false;
            pos = 0;
            output.Clear();
        }

        /// Power cycle: back to SD mode (the card keeps power over a CPU reset).
        public void PowerCycle()
        {
            spiMode = false;
            ready = false;
            appCmd = false;
        }

        public bool Selected => selected;
        public bool Inserted { get; set; }
        public int BusyCount { get; set; }
        public int AcmdCount { get; set; }
        public int Commands { get; set; }
        public int Reads { get; set; }
        public bool Ready => ready;

        private readonly byte[] image;
        private readonly byte[] frame = new byte[6];
        private readonly Queue<byte> output = new Queue<byte>();
        private bool selected, spiMode, ready, appCmd;
        private int pos, busyLeft;
    }
}
