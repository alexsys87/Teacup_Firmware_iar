/** \file
  \brief Servo signal for the BLTouch (M280), software pulses by TIM9.

  The servo pin (PC13 on the P3 Steel board) has no timer channel, so TIM9
  runs as a plain 1 MHz timer with a 20 ms period: its update interrupt
  raises the pin, compare 1 lowers it again. Pulse jitter is a few us at
  most (only the step interrupt has a higher priority).
*/

#ifndef _SERVO_H
#define _SERVO_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef BLTOUCH_SERVO_PIN

/// Set up TIM9 and the pin. No pulses until servo_write_*().
void servo_init(void);

/// Pulse width in us (500..2500), 0 = no pulses (pin low).
void servo_write_us(uint16_t us);

/**
  Angle 0..180 (Arduino Servo scale: 544..2400 us, as Marlin and the
  BLTouch documentation use it). Values >= 544 are taken as us.
*/
void servo_write_angle(uint16_t angle);

/// Last angle written (M280 report).
uint16_t servo_read_angle(void);

#endif /* BLTOUCH_SERVO_PIN */

#endif /* _SERVO_H */
