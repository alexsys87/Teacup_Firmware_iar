// PCF8574 I/O expander model for the Teacup tests: every byte written
// sets the 8 outputs (Output), a read returns them. Attach with
//   pcf: I2C.TeacupPCF8574 @ i2c1 0x20
// and read the outputs in the monitor with "sysbus.i2c1.pcf Output".
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
                result[i] = Output;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
            Output = 0xFF;          // Power up: all high.
        }

        public byte Output { get; private set; }
        public int Writes { get; private set; }
    }
}
