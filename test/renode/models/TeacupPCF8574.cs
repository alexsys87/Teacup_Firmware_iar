// PCF8574 I/O expander model for the Teacup tests: every byte written
// sets the 8 outputs (Output), a read returns the pin states: outputs
// AND inputs (Inputs, 0xFF = nothing pulls low; a pressed button to GND
// clears its bit). Attach with
//   pcf: I2C.TeacupPCF8574 @ i2c1 0x20
// read the outputs with "sysbus.i2c1.pcf Output", press a button on P0
// with "sysbus.i2c1.pcf Inputs 0xFE".
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class TeacupPCF8574 : II2CPeripheral
    {
        public TeacupPCF8574()
        {
            Reset();
        }

        public void Write(byte[] data)
        {
            foreach(var b in data)
            {
                Output = b;
                Writes++;
                this.Log(LogLevel.Debug, "outputs 0x{0:X2}", b);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = (byte)(Output & Inputs);
                Reads++;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            Output = 0xFF;          // Power up: all high.
            Inputs = 0xFF;
        }

        public byte Output { get; private set; }
        public int Writes { get; private set; }
        public byte Inputs { get; set; }
        public int Reads { get; private set; }
    }
}
