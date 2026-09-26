/** \file
  \brief Thermal protection: runaway, heating failed, MINTEMP, MAXTEMP,
         heater stuck on.
*/

#ifndef _THERMAL_PROTECTION_H
#define _THERMAL_PROTECTION_H

#include <stdint.h>

/**
  Check one temperature sensor with an attached heater. Call every 250 ms.

  \param sensor  Temperature sensor index.
  \param current Latest reading, 14.2 fixed point (quarter degrees).
  \param target  Target temperature, same unit. 0 = heater off.
  \param age     250 ms ticks since the last valid reading (1 = fresh).
  \param heater_on 1 while the heater output is on (PWM > 0).

  Calls printer_kill() on any violation, so it doesn't return then.
*/
void thermal_protection_check(uint8_t sensor, uint16_t current,
                              uint16_t target, uint8_t age,
                              uint8_t heater_on);

/**
  Limit a new target temperature to MAXTEMP minus the allowed overshoot.
  \return The target to use.
*/
uint16_t thermal_protection_limit_target(uint8_t sensor, uint16_t target);

#endif /* _THERMAL_PROTECTION_H */
