/** \file
  \brief PID autotune (M303).
*/

#ifndef _PID_AUTOTUNE_H
#define _PID_AUTOTUNE_H

#include <stdint.h>
#include "heater.h"
#include "temp.h"

/**
  Relay autotune, Marlin's algorithm (Astrom-Hagglund relay + classic
  Ziegler-Nichols). Blocks until done, keeps the clock running.

  \param h       Heater to tune.
  \param s       Its temperature sensor.
  \param target  Temperature to oscillate around, degree Celsius.
  \param cycles  Number of oscillations, 3..20.
  \param apply   Use the result right away (M303 U1).
*/
void pid_autotune(heater_t h, temp_sensor_t s, uint16_t target,
                  uint8_t cycles, uint8_t apply);

#endif /* _PID_AUTOTUNE_H */
