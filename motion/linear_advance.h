/** \file
  \brief Linear advance (M900 K): the extruder runs ahead of its nominal
  position by K * extrusion speed.
*/

#ifndef _LINEAR_ADVANCE_H
#define _LINEAR_ADVANCE_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef LINEAR_ADVANCE

#include "arch.h"

/**
  E position in steps the Bresenham algorithm of the moves gives (nominal),
  and the one of the E motor. Both count from zero at startup, only their
  difference matters. Written in the step interrupt only.
*/
extern volatile int32_t la_e_nominal;
extern volatile int32_t la_e_actual;

/// Set by la_step_e(): the E generator has to look at the new position.
extern volatile uint8_t la_kicked;

/**
  One E step of the Bresenham algorithm, called by dda_step() instead of
  stepping E directly. The E generator (la_isr()) follows right after in
  the same step interrupt.
*/
TEACUP_INLINE void la_step_e(uint8_t forward) {
  la_e_nominal += forward ? 1 : -1;
  la_kicked = 1;
}

/// E generator, called by the step interrupt (TIM5 compare 4 or kicked).
void la_isr(void);

/**
  New advance in E steps, from dda_clock() every TICK_TIME. The change is
  spread evenly over the next TICK_TIME.
*/
void la_set_advance(int32_t steps);

/// Whether E still has to catch up with its goal (queue_wait()).
uint8_t la_busy(void);

/**
  Stop, keep the E motor where it is (queue_flush()). Call with interrupts
  disabled.
*/
void la_flush(void);

/**
  The step timer counter was set back by 'offset' (timer_reset()), shift the
  time stamps. Call with interrupts disabled.
*/
void la_rebase(uint32_t offset);

#endif /* LINEAR_ADVANCE */
#endif /* _LINEAR_ADVANCE_H */
