/** \file
  \brief Endstop interrupts.

  Without interrupts, dda_clock() polls the endstops every 2 ms and needs
  ENDSTOP_STEPS consecutive readings (8 ms with the default 4) before it
  stops a move. With ENDSTOP_INTERRUPTS, any edge on an endstop pin
  triggers dda_clock() right away (via PendSV, so no new concurrency) and
  the first matching reading counts. Polling stays active, it catches an
  endstop already triggered when a move starts.

  EXTI lines are shared between ports (PA3 and PB3 are both line 3). If two
  endstops share a line number, the second one is polled only.
*/

#include "endstops.h"
#include "arch.h"
#include "config_wrapper.h"
#include "cpu.h"

#ifndef ENDSTOP_INTERRUPTS

void endstops_init(void) { }
void endstops_irq_enable(uint8_t on) { (void)on; }
uint8_t endstops_irq_take(void) { return 0; }

#else

static volatile uint8_t endstop_edge = 0;
static uint32_t endstop_lines = 0;          ///< EXTI lines in use.

static void setup_line(uint8_t id) {
  uint32_t line = PIN_ID_NUM(id);
  uint32_t port = (id >> 4) & 0x0F;
  uint32_t shift = (line & 3) * 4;

  if (endstop_lines & MASK(line))
    return;                                 // Line taken, poll only.
  endstop_lines |= MASK(line);

  SYSCFG->EXTICR[line >> 2] = (SYSCFG->EXTICR[line >> 2] & ~(0xFUL << shift)) |
                              (port << shift);
  EXTI->RTSR |= MASK(line);                 // Both edges: trigger and
  EXTI->FTSR |= MASK(line);                 // release (back off) matter.
}

void endstops_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  (void)RCC->APB2ENR;

  #ifdef X_MIN_PIN
    setup_line(PIN_ID(X_MIN_PIN));
  #endif
  #ifdef X_MAX_PIN
    setup_line(PIN_ID(X_MAX_PIN));
  #endif
  #ifdef Y_MIN_PIN
    setup_line(PIN_ID(Y_MIN_PIN));
  #endif
  #ifdef Y_MAX_PIN
    setup_line(PIN_ID(Y_MAX_PIN));
  #endif
  #ifdef Z_MIN_PIN
    setup_line(PIN_ID(Z_MIN_PIN));
  #endif
  #ifdef Z_MAX_PIN
    setup_line(PIN_ID(Z_MAX_PIN));
  #endif

  EXTI->IMR &= ~endstop_lines;
  EXTI->PR = endstop_lines;

  // Above SysTick, below the UART: an edge is handled within microseconds.
  NVIC_SetPriority(EXTI0_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI1_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI2_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI3_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI4_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI9_5_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_SetPriority(EXTI15_10_IRQn, IRQ_PRIO_ENDSTOP);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void endstops_irq_enable(uint8_t on) {
  if (on) {
    EXTI->PR = endstop_lines;
    EXTI->IMR |= endstop_lines;
  }
  else {
    EXTI->IMR &= ~endstop_lines;
  }
}

uint8_t endstops_irq_take(void) {
  uint8_t e = endstop_edge;

  endstop_edge = 0;
  return e;
}

/// Common handler: acknowledge, remember, run dda_clock() now.
TEACUP_HOT
static void endstop_irq(void) {
  uint32_t pending = EXTI->PR & endstop_lines;

  EXTI->PR = pending;
  if (pending) {
    endstop_edge = 1;
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }
}

void EXTI0_IRQHandler(void)     { endstop_irq(); }
void EXTI1_IRQHandler(void)     { endstop_irq(); }
void EXTI2_IRQHandler(void)     { endstop_irq(); }
void EXTI3_IRQHandler(void)     { endstop_irq(); }
void EXTI4_IRQHandler(void)     { endstop_irq(); }
void EXTI9_5_IRQHandler(void)   { endstop_irq(); }
void EXTI15_10_IRQHandler(void) { endstop_irq(); }

#endif /* ENDSTOP_INTERRUPTS */
