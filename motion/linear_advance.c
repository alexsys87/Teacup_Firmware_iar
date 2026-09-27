/** \file
  \brief Linear advance (M900 K).

  Pressure in the nozzle lags behind the extruder: when a move speeds up,
  too little comes out, when it slows down, too much (bulging corners, a
  blob at the seam). Linear advance runs the extruder ahead of its nominal
  position by

    advance = K * extrusion speed   (K in seconds, Marlin units)

  so the extruder pushes extra filament while accelerating and takes it
  back while decelerating.

  In the moves E is one axis of the Bresenham algorithm, stepping together
  with the fast axis at most once per step interrupt. That's too slow for
  the extra steps at low speed, and they can go against the direction of
  the move. So E gets a generator of its own:

   - dda_step() only counts E steps (la_e_nominal, la_step_e()).
   - dda_clock() sets the advance from the current speed every TICK_TIME
     (la_set_advance()). The change is spread evenly over the next
     TICK_TIME, the speed changes in steps of TICK_TIME, too.
   - la_service() steps the E motor (la_e_actual) towards nominal +
     advance, one step at a time, at least LA_STEP_GAP apart, setting the
     direction pin as needed. It's part of the auxiliary step generator
     (hal/timer.c) in the step interrupt: it runs right after each
     Bresenham E step, and again when it asks for it while the advance
     changes or E has to catch up.

  Moves without linear advance (K = 0, retracts, travel) get advance 0, E
  then follows the Bresenham steps one by one.
*/

#include "linear_advance.h"

#ifdef LINEAR_ADVANCE

#include "pinio.h"
#include "timer.h"
#include "atomic.h"

/// A change of the advance is spread over this many timer ticks.
#define LA_INTERP     ((uint32_t)TICK_TIME)

/**
  Minimum time between two E steps. Three pulse widths: with GPIO pulses
  a pulse of another axis can extend the E pulse (shared pulse end),
  there must still be enough low time before the next one.
*/
#define LA_STEP_GAP   (3UL * STEP_PULSE_CYCLES)

/// Time from a change of the direction pin to the next step.
#define LA_DIR_SETUP  ((uint32_t)(2 US))

volatile int32_t la_e_nominal;
volatile int32_t la_e_actual;

/// Advance: from adv_from at adv_t0 linearly to adv_to at adv_t0 +
/// LA_INTERP. adv_slope is (adv_to - adv_from) / LA_INTERP, 16.16 fixed
/// point.
static int32_t adv_from, adv_to, adv_slope;
static uint32_t adv_t0;

/// Last E step and last direction change, step timer counts.
static uint32_t last_step, last_dir;

/// Current state of the direction pin: 1 forward, 0 backward, 2 unknown.
static uint8_t e_dir = 2;

/// Advance at timer count 'now'.
TEACUP_HOT
TEACUP_STEP_RAMFUNC static int32_t advance_now(uint32_t now) {
  uint32_t t = now - adv_t0;

  if (t >= LA_INTERP)
    return adv_to;
  return adv_from + (int32_t)(((int64_t)adv_slope * (int32_t)t) >> 16);
}

/// One E step pulse.
TEACUP_HOT
TEACUP_STEP_RAMFUNC static void e_pulse(void) {
  #ifdef STEP_TIMER_PULSES
    TIM_TypeDef *tim = step_timer_e();

    if (tim) {
      STEP_TRIGGER(tim);
      return;
    }
  #endif
  {
    step_set_t s;

    step_set_clear(&s);
    step_add_e(&s);
    step_output(&s);
    timer_step_pulse_end();
  }
}

/**
  E is at its goal: come back when the advance has moved by one step, or
  sleep until the next Bresenham step or the next la_set_advance().
*/
TEACUP_HOT
TEACUP_STEP_RAMFUNC static uint32_t schedule_idle(uint32_t now) {
  uint32_t t = now - adv_t0;
  int32_t diff = adv_to - adv_from;
  uint32_t dt, rest;

  if (t >= LA_INTERP || diff == 0)
    return AUX_NONE;
  if (diff < 0)
    diff = -diff;
  dt = LA_INTERP / (uint32_t)diff;
  rest = LA_INTERP - t;
  if (dt > rest)
    dt = rest;
  if (dt < LA_STEP_GAP)
    dt = LA_STEP_GAP;
  return dt;
}

TEACUP_HOT
TEACUP_STEP_RAMFUNC uint32_t la_service(uint32_t now) {
  // One volatile access per statement (IAR Pa082).
  int32_t nominal = la_e_nominal;
  int32_t actual = la_e_actual;
  int32_t d = nominal + advance_now(now) - actual;
  uint8_t dir;
  uint32_t since;

  if (d == 0)
    return schedule_idle(now);

  dir = (d > 0) ? 1 : 0;
  if (dir != e_dir) {
    e_direction(dir);
    e_dir = dir;
    last_dir = now;
    return LA_DIR_SETUP;
  }

  since = now - last_dir;
  if (since < LA_DIR_SETUP)
    return LA_DIR_SETUP - since;
  since = now - last_step;
  if (since < LA_STEP_GAP)
    return LA_STEP_GAP - since;

  e_pulse();
  la_e_actual += dir ? 1 : -1;
  last_step = now;

  if (d > 1 || d < -1)
    return LA_STEP_GAP;
  return schedule_idle(now);
}

void la_set_advance(int32_t steps) {
  ATOMIC_START();
    if (steps != adv_to) {
      uint32_t now = TIM5->CNT;
      int32_t cur = advance_now(now);

      adv_from = cur;
      adv_to = steps;
      adv_t0 = now;
      adv_slope = (int32_t)(((int64_t)(steps - cur) << 16) /
                            (int64_t)LA_INTERP);
      timer_aux_kick();                       // Look at it right away.
    }
  ATOMIC_END();
}

uint8_t la_busy(void) {
  uint8_t busy;

  ATOMIC_START();
    // One volatile access per statement (IAR Pa082).
    uint32_t now = TIM5->CNT;
    int32_t nominal = la_e_nominal;
    int32_t actual = la_e_actual;

    busy = adv_to != 0 || advance_now(now) != 0 || actual != nominal;
  ATOMIC_END();
  return busy;
}

void la_flush(void) {
  int32_t actual = la_e_actual;           // IAR Pa082.

  adv_from = adv_to = adv_slope = 0;
  la_e_nominal = actual;
}

#endif /* LINEAR_ADVANCE */
