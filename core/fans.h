/** \file
  \brief Hotend auto fan and controller fan, on MCU pins or on a PCF8574
  I/O expander (I2C).
*/

#ifndef _FANS_H
#define _FANS_H

#include <stdint.h>
#include "config_wrapper.h"

#if defined HOTEND_FAN_PIN || defined HOTEND_FAN_EXPANDER_BIT || \
    defined CONTROLLER_FAN_PIN || defined CONTROLLER_FAN_EXPANDER_BIT
  #define FANS
#endif

#ifdef PCF8574_ADDRESS
/// Set an output of the PCF8574 (sent within 10 ms).
void expander_set(uint8_t bit, uint8_t on);
/// Send pending expander changes, every 10 ms.
void expander_tick(void);
#endif

#ifdef FANS
void fans_init(void);

/// Every 250 ms: hotend fan by temperature, controller fan by activity.
void fans_tick(void);

/**
  printer_kill(): hotend fan on (MCU pin only, the I2C interrupt is off
  then; an expander output keeps its state, on while the hotend is hot).
*/
void fans_emergency(void);

/// Current states, for M503 / tests: bit 0 hotend fan, bit 1 controller fan.
uint8_t fans_state(void);
#endif

#endif /* _FANS_H */
