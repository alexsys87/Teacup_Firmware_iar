/** \file
  \brief Spindle or laser on a PWM output (SPINDLE_LASER), M3 / M4 / M5
  like Marlin.
*/

#ifndef _SPINDLE_H
#define _SPINDLE_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef SPINDLE_LASER

void spindle_init(void);

/**
  M3 (cw = 1) / M4 (cw = 0) with power S (0..SPEED_POWER_MAX, s_seen = 0:
  full power). Spindle: waits for the moves, then runs. Laser mode: the
  power of the following G1 / G2 / G3 moves (inline), M4 = dynamic power.
*/
void spindle_on(uint8_t cw, uint8_t s_seen, uint32_t s);

/// M5: off (spindle after the moves, laser for the following moves).
void spindle_off(void);

/// Laser mode: S on a G1 / G2 / G3 line sets the inline power.
void spindle_set_inline(uint32_t s);

/**
  Laser mode: power for the moves queued from now on, 0 for G0 and moves
  not from G-code (homing etc.). Set by gcode_process() around enqueue().
*/
void spindle_move_begin(uint8_t g);
void spindle_move_end(void);

/// Power (PWM counts) of a new move, dda_create().
uint16_t spindle_move_power(void);

/// Step interrupt: a move starts (its power) or the queue ran empty (0).
void spindle_apply(uint16_t power);

/**
  dda_clock(): laser dynamic mode (M4), power scaled with the speed:
  power * c_min / c.
*/
void spindle_dynamic(uint16_t power, uint32_t c, uint32_t c_min);

/// Emergency: off now.
void spindle_emergency_off(void);

/// Current output, PWM counts (M-code report, tests).
uint16_t spindle_output(void);

#endif /* SPINDLE_LASER */

#endif /* _SPINDLE_H */
