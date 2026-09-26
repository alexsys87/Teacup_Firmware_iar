/** \file
  \brief CPU initialisation: clocks, flash, cycle counter, fault handling.
*/

#ifndef _CPU_H
#define _CPU_H

#include "arch.h"

/// Clock source actually in use after cpu_init().
enum cpu_clock_e {
  CPU_CLOCK_HSE = 0,    ///< PLL from the crystal, F_CPU exact.
  CPU_CLOCK_HSI,        ///< Crystal failed, PLL from internal RC, F_CPU +-1 %.
  CPU_CLOCK_FAILED      ///< PLL failed, running at 16 MHz HSI, timing wrong!
};

extern uint8_t cpu_clock_source;

/// RCC->CSR at startup, tells the reset cause (RCC_CSR_IWDGRSTF etc.).
extern uint32_t cpu_reset_flags;

/** Interrupt priorities, 0 = highest. All bits are preemption bits. */
#define IRQ_PRIO_STEP     0   ///< TIM5 step interrupt.
#define IRQ_PRIO_SERIAL   1   ///< Host UART and USB.
#define IRQ_PRIO_SERVO    1   ///< Servo pulses (TIM9, BLTouch).
#define IRQ_PRIO_ENDSTOP  2   ///< Endstop edges (EXTI).
#define IRQ_PRIO_SYSTICK  3   ///< System clock tick.
#define IRQ_PRIO_I2C      4   ///< Display bus.
#define IRQ_PRIO_PENDSV   15  ///< dda_clock(), lowest.

void cpu_init(void);

/* Interrupt handlers implemented by the firmware (names from the startup
   file's vector table). */
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SysTick_Handler(void);
void PendSV_Handler(void);
void TIM5_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);
void USART1_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void USART2_IRQHandler(void);
void USART6_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif /* _CPU_H */
