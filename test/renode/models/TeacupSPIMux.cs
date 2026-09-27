// SPI bus with several devices for the Teacup tests. Renode's STM32SPI
// holds one peripheral: register this at the SPI, the devices at it
// (numbers 0, 1, ...), their chip selects from GPIOs as before.
//
// Every byte goes to all devices; unselected ones ignore it and answer
// 0xFF. MISO is the AND of all answers, like a bus where any driver pulls
// low. Conflicts counts bytes sent with more than one device selected
// (the device models have a Selected property).
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class TeacupSPIMux : SimpleContainer<ISPIPeripheral>, ISPIPeripheral
    {
        public TeacupSPIMux(IMachine machine) : base(machine)
        {
        }

        public byte Transmit(byte data)
        {
            byte result = 0xFF;
            var selected = new List<string>();
            foreach(var child in ChildCollection)
            {
                var p = child.Value.GetType().GetProperty("Selected");
                if(p != null && (bool)p.GetValue(child.Value))
                {
                    selected.Add(child.Key.ToString());
                }
                result &= child.Value.Transmit(data);
            }
            Bytes++;
            if(selected.Count > 1)
            {
                Conflicts++;
                if(Conflicts <= 10)
                {
                    this.Log(LogLevel.Warning, "SPI bus conflict: devices {0} selected", string.Join(",", selected));
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
            foreach(var child in ChildCollection.Values)
            {
                child.FinishTransmission();
            }
        }

        public override void Reset()
        {
        }

        public int Conflicts { get; set; }
        public int Bytes { get; set; }
    }
}
