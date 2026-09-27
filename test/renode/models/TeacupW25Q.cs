// SPI NOR flash model (Winbond W25Q64 style) for the Teacup tests.
// Chip select comes from a GPIO (connect the CS pin to input 0), because
// Renode's STM32SPI doesn't frame transfers. Commands: 9F JEDEC ID,
// 05 status, 06/04 write enable/disable, 03 read, 0B fast read, 02 page
// program, 20 4 kB erase, D8 64 kB erase, C7/60 chip erase, AB release
// power down, B9 power down. Program and erase complete instantly.
// Contents survive machine resets (not a power cycle of Renode).
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class TeacupW25Q : ISPIPeripheral, IGPIOReceiver
    {
        public TeacupW25Q(int sizeMB = 8)
        {
            memory = new byte[sizeMB * 1024 * 1024];
            for(var i = 0; i < memory.Length; i++) memory[i] = 0xFF;
            capacityCode = (byte)(Math.Log(memory.Length, 2));
        }

        public void OnGPIO(int number, bool value)
        {
            // CS: low = selected. Rising edge ends a command.
            if(!value && !selected) { selected = true; count = 0; command = 0; }
            else if(value && selected) { selected = false; FinishCommand(); }
        }

        public byte Transmit(byte data)
        {
            if(!selected) return 0xFF;
            byte result = 0xFF;
            if(count == 0)
            {
                command = data;
                address = 0;
                if(command == 0x06) writeEnabled = true;
                if(command == 0x04) writeEnabled = false;
            }
            else
            {
                switch(command)
                {
                case 0x9F:
                    result = count == 1 ? (byte)0xEF : count == 2 ? (byte)0x40 : capacityCode;
                    break;
                case 0x05:
                    result = (byte)(writeEnabled ? 0x02 : 0x00);
                    break;
                case 0xAB:
                    result = count >= 4 ? (byte)(capacityCode - 1) : (byte)0xFF;
                    break;
                case 0x03: case 0x0B: case 0x02: case 0x20: case 0xD8:
                    var dummy = command == 0x0B ? 1 : 0;
                    if(count <= 3)
                    {
                        address = (address << 8) | data;
                    }
                    else if(count > 3 + dummy)
                    {
                        if(command == 0x03 || command == 0x0B)
                        {
                            result = memory[address % memory.Length];
                            address++;
                        }
                        else if(command == 0x02 && writeEnabled)
                        {
                            // Page program wraps within the 256 byte page.
                            var page = address & ~0xFF;
                            var a = page | ((address + programmed) & 0xFF);
                            memory[a % memory.Length] &= data;
                            programmed++;
                        }
                    }
                    break;
                }
            }
            count++;
            return result;
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            // Keep the contents.
            selected = false;
            writeEnabled = false;
        }

        // Chip select state, for the bus model (TeacupSPIMux).
        public bool Selected => selected;

        private void FinishCommand()
        {
            switch(command)
            {
            case 0x20:
                if(writeEnabled && count >= 4) Erase(address & ~0xFFF, 0x1000);
                break;
            case 0xD8:
                if(writeEnabled && count >= 4) Erase(address & ~0xFFFF, 0x10000);
                break;
            case 0xC7: case 0x60:
                if(writeEnabled) Erase(0, memory.Length);
                break;
            }
            if(command == 0x02 || command == 0x20 || command == 0xD8 || command == 0xC7 || command == 0x60)
            {
                writeEnabled = false;
            }
            programmed = 0;
        }

        private void Erase(int start, int length)
        {
            for(var i = 0; i < length; i++) memory[(start + i) % memory.Length] = 0xFF;
            this.Log(LogLevel.Debug, "Erased 0x{0:X} bytes at 0x{1:X}", length, start);
        }

        private readonly byte[] memory;
        private readonly byte capacityCode;
        private bool selected;
        private bool writeEnabled;
        private int count;
        private byte command;
        private int address;
        private int programmed;
    }
}
