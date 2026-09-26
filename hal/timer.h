/** \file
  \brief Timer management - step pulse clock and system clock.
*/

#ifndef _TIMER_H
#define _TIMER_H

#include <stdint.h>
#include "config_wrapper.h"

// time-related constants
#define US * (F_CPU / 1000000)
#define MS * (F_CPU / 1000)

/// How often we overflow and update our clock.
/// SysTick is 24 bits, so TICK_TIME must stay below 16.7 million cycles.
#define TICK_TIME (2 MS)

/// Convert back to ms from cpu ticks so our system clock runs
/// properly if you change TICK_TIME.
#define TICK_TIME_MS (TICK_TIME / (F_CPU / 1000))

void timer_init(void);

uint8_t timer_set(int32_t delay, uint8_t check_short);

void timer_reset(void);

void timer_stop(void);

/**
  Schedule the end of the step pulses just raised: compare channel 2 of the
  step timer fires after MIN_STEP_PULSE_US and lowers all step pins. No
  busy waiting in the step interrupt.
*/
void timer_step_pulse_end(void);

#ifdef STEP_AUX
/**
  Auxiliary step generator, see hal/timer.c. Each part (la_service(),
  shaper_service()) returns the CPU ticks until it wants to run again, or
  AUX_NONE.
*/
#define AUX_NONE 0xFFFFFFFFUL

/// Set in the step interrupt: run the auxiliary generator after this step.
extern volatile uint8_t aux_kicked;

/// Run the auxiliary generator right away, from any context.
void timer_aux_kick(void);
#endif

/// Step interrupt statistics (M9001), all in CPU cycles.
typedef struct {
  uint32_t count;       ///< Step interrupts.
  uint32_t min, max;    ///< Duration of one step interrupt.
  uint64_t sum;         ///< Total duration, for average and load.
  uint32_t max_latency; ///< Compare match to handler entry.
  uint32_t late;        ///< Steps scheduled too late (fired immediately).
  uint32_t since;       ///< clock_millis() at reset.
  uint32_t pulses;      ///< Pulse end interrupts.
} step_stats_t;

extern volatile step_stats_t step_stats;

/// Reset the statistics.
void step_stats_reset(void);

#endif /* _TIMER_H */
