/** \file
  \brief Endstop interrupts (EXTI): react to endstop edges immediately.
*/

#ifndef _ENDSTOPS_H
#define _ENDSTOPS_H

#include <stdint.h>

/// Configure EXTI lines of all endstop pins (interrupts stay disabled).
void endstops_init(void);

/// Enable/disable the endstop interrupts (while homing / M119).
void endstops_irq_enable(uint8_t on);

/**
  Whether an endstop edge happened since the last call (clears it).
  dda_clock() then accepts the endstop without debouncing.
*/
uint8_t endstops_irq_take(void);

#endif /* _ENDSTOPS_H */
