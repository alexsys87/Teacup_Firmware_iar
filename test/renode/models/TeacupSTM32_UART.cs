// Minimal STM32F4 USART model for the Teacup tests, written after
// renode-infrastructure's STM32_UART.cs (MIT). Adds what the stock model of
// Renode 1.16 lacks: DMA reception requests (CR3.DMAR -> DMARequest) and
// IDLE line detection (one frame after the last received character).
// Transmission: DR writes go out immediately, TXE is always set.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    [AllowedTranslations(AllowedTranslation.WordToDoubleWord | AllowedTranslation.ByteToDoubleWord)]
    public class TeacupSTM32_UART : BasicDoubleWordPeripheral, IUART
    {
        public TeacupSTM32_UART(IMachine machine, uint frequency = 42000000) : base(machine)
        {
            this.frequency = frequency;
            DefineRegisters();
        }

        public void WriteChar(byte value)
        {
            if(!ue.Value || !re.Value)
            {
                this.Log(LogLevel.Warning, "Receiver not enabled, dropping 0x{0:X}", value);
                return;
            }
            fifo.Enqueue(value);
            rxne.Value = true;
            receivedCount++;
            var mine = receivedCount;
            // IDLE: one frame (10 bits) after the last character.
            var baud = BaudRate == 0 ? 115200u : BaudRate;
            machine.ScheduleAction(TimeInterval.FromMicroseconds(10000000UL / baud),
                _ => { if(mine == receivedCount) { idle.Value = true; Update(); } });
            if(dmar.Value)
            {
                DMARequest.Blink();
            }
            Update();
        }

        public override void Reset()
        {
            base.Reset();
            fifo.Clear();
        }

        public GPIO IRQ { get; } = new GPIO();
        public GPIO DMARequest { get; } = new GPIO();
        public uint BaudRate => brr.Value == 0 ? 0 : (uint)(frequency / brr.Value);
        public Bits StopBits => Bits.One;
        public Parity ParityBit => Parity.None;

        [field: Antmicro.Migrant.Transient]
        public event Action<byte> CharReceived;

        private void DefineRegisters()
        {
            Registers.Status.Define(this, 0xC0, name: "USART_SR")
                .WithTaggedFlag("PE", 0)
                .WithTaggedFlag("FE", 1)
                .WithTaggedFlag("NF", 2)
                .WithFlag(3, FieldMode.Read, valueProviderCallback: _ => false, name: "ORE")
                .WithFlag(4, out idle, FieldMode.Read, name: "IDLE")
                .WithFlag(5, out rxne, FieldMode.Read | FieldMode.WriteZeroToClear, name: "RXNE")
                .WithFlag(6, out tc, FieldMode.Read | FieldMode.WriteZeroToClear, name: "TC")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => true, name: "TXE")
                .WithReservedBits(8, 24)
                .WithWriteCallback((_, __) => Update());
            Registers.Data.Define(this, name: "USART_DR")
                .WithValueField(0, 9, valueProviderCallback: _ =>
                {
                    uint v = 0;
                    idle.Value = false;               // SR read + DR read clears IDLE
                    if(fifo.Count > 0) v = fifo.Dequeue();
                    rxne.Value = fifo.Count > 0;
                    Update();
                    return v;
                }, writeCallback: (_, value) =>
                {
                    if(!ue.Value || !te.Value) return;
                    CharReceived?.Invoke((byte)value);
                    tc.Value = true;
                    Update();
                }, name: "DR")
                .WithReservedBits(9, 23);
            Registers.BaudRate.Define(this, name: "USART_BRR")
                .WithValueField(0, 16, out brr, name: "BRR")
                .WithReservedBits(16, 16);
            Registers.Control1.Define(this, name: "USART_CR1")
                .WithTaggedFlag("SBK", 0)
                .WithTaggedFlag("RWU", 1)
                .WithFlag(2, out re, name: "RE")
                .WithFlag(3, out te, name: "TE")
                .WithFlag(4, out idleie, name: "IDLEIE")
                .WithFlag(5, out rxneie, name: "RXNEIE")
                .WithFlag(6, out tcie, name: "TCIE")
                .WithFlag(7, out txeie, name: "TXEIE")
                .WithTaggedFlag("PEIE", 8)
                .WithTaggedFlag("PS", 9)
                .WithTaggedFlag("PCE", 10)
                .WithTaggedFlag("WAKE", 11)
                .WithTaggedFlag("M", 12)
                .WithFlag(13, out ue, name: "UE")
                .WithReservedBits(14, 1)
                .WithTaggedFlag("OVER8", 15)
                .WithReservedBits(16, 16)
                .WithWriteCallback((_, __) => Update());
            Registers.Control2.Define(this, name: "USART_CR2")
                .WithValueField(0, 32, name: "CR2");
            Registers.Control3.Define(this, name: "USART_CR3")
                .WithValueField(0, 6, name: "CR3_LOW")
                .WithFlag(6, out dmar, name: "DMAR")
                .WithFlag(7, name: "DMAT")
                .WithValueField(8, 24, name: "CR3_HIGH");
            Registers.GuardTimeAndPrescaler.Define(this, name: "USART_GTPR")
                .WithValueField(0, 32, name: "GTPR");
        }

        private void Update()
        {
            IRQ.Set((idleie.Value && idle.Value) || (rxneie.Value && rxne.Value) ||
                    txeie.Value || (tcie.Value && tc.Value));
        }

        private readonly uint frequency;
        private readonly Queue<byte> fifo = new Queue<byte>();
        private ulong receivedCount;
        private IFlagRegisterField idle, rxne, tc, re, te, idleie, rxneie, tcie, txeie, ue, dmar;
        private IValueRegisterField brr;

        private enum Registers : long
        {
            Status = 0x00,
            Data = 0x04,
            BaudRate = 0x08,
            Control1 = 0x0C,
            Control2 = 0x10,
            Control3 = 0x14,
            GuardTimeAndPrescaler = 0x18,
        }
    }
}
