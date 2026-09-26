/** \file
  \brief Delay routines.

  Uses the DWT cycle counter, so timing doesn't depend on flash wait states,
  caches or interrupts (an interrupt only extends the delay).
*/

#include "delay.h"
#include "config_wrapper.h"
#include "watchdog.h"

void delay_us(uint32_t delay) {
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = delay * (F_CPU / 1000000UL);

  while ((DWT->CYCCNT - start) < cycles)
    ;
}

void delay_ms(uint32_t delay) {
  wd_reset();
  while (delay--) {
    delay_us(1000);
    wd_reset();
  }
}
