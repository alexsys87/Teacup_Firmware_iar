/** \file
  \brief Critical sections.

  Usage:  ATOMIC_START();  ...  ATOMIC_END();
  Nesting is allowed, the previous interrupt state is restored.
*/

#ifndef _ATOMIC_H
#define _ATOMIC_H

#include "arch.h"
#include "debug.h"

/* Older CMSIS versions (e.g. bundled with older IAR) lack this macro. */
#ifndef __COMPILER_BARRIER
  #define __COMPILER_BARRIER() __ASM volatile("" ::: "memory")
#endif

#define ATOMIC_START() do { \
  uint32_t atomic_primask_save_ = __get_PRIMASK(); \
  __disable_irq(); \
  __COMPILER_BARRIER()

#define ATOMIC_END() \
  __COMPILER_BARRIER(); \
  __set_PRIMASK(atomic_primask_save_); \
} while (0)

/**
  Critical section which keeps the step interrupt (priority 0) running.
  For data shared with the UART, SysTick, I2C or PendSV interrupts only,
  never for data used by the step interrupt. Uses BASEPRI, so steps aren't
  delayed by the main loop.

  With DEBUG the step interrupt may print, then this falls back to a full
  critical section.
*/
#ifdef DEBUG
  #define ATOMIC_START_NOSTEP()  ATOMIC_START()
  #define ATOMIC_END_NOSTEP()    ATOMIC_END()
#else
  #define ATOMIC_START_NOSTEP() do { \
    uint32_t atomic_basepri_save_ = __get_BASEPRI(); \
    __set_BASEPRI_MAX(1UL << (8U - __NVIC_PRIO_BITS)); \
    __COMPILER_BARRIER()

  #define ATOMIC_END_NOSTEP() \
    __COMPILER_BARRIER(); \
    __set_BASEPRI(atomic_basepri_save_); \
  } while (0)
#endif

/** Atomic set/clear of a single bit in a peripheral register (bit-band). */
#define PERIPH_BIT(reg, bit) \
  (*(volatile uint32_t *)(PERIPH_BB_BASE + \
     (((uint32_t)&(reg) - PERIPH_BASE) * 32U) + ((uint32_t)(bit) * 4U)))

#endif /* _ATOMIC_H */
