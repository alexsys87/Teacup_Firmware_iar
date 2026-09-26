/** \file
  \brief Busy waiting delays based on the DWT cycle counter.
*/

#ifndef _DELAY_H
#define _DELAY_H

#include "arch.h"

/// Current value of the free running CPU cycle counter.
#define cycle_count()   (DWT->CYCCNT)

/// Microsecond delay, does NOT reset the watchdog.
void delay_us(uint32_t delay);

/// Millisecond delay, resets the watchdog.
void delay_ms(uint32_t delay);

#endif /* _DELAY_H */
