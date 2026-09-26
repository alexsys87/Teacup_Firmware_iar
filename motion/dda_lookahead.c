
/** \file
  \brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include "dda_lookahead.h"

#ifdef LOOKAHEAD

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include "dda_maths.h"
#include "dda.h"
#include "timer.h"
#include "delay.h"
#include "dda_queue.h"
#include "sersendf.h"
#include "pinio.h"
#include "atomic.h"
#include "settings.h"

#ifdef DEBUG
  // Total number of moves joined together.
  uint32_t lookahead_joined = 0;
  // Moves that did not compute in time to be actually joined.
  uint32_t lookahead_timeout = 0;
#endif

/// Maximum jerk per axis, settings.max_jerk (M205).
#define maximum_jerk_P settings.max_jerk


/**
 * \brief Find maximum corner speed between two moves.
 * \details Find out how fast we can move around around a corner without
 * exceeding the expected jerk. Worst case this speed is zero, which means a
 * full stop between both moves. Best case it's the lower of the maximum speeds.
 *
 * This function is expected to be called from within dda_create().
 *
 * \param [in] prev is the DDA structure of the move previous to the current one.
 * \param [in] current is the DDA structure of the move currently created.
 *
 * \return dda->crossF
 */
TEACUP_HOT
void dda_find_crossing_speed(DDA *prev, DDA *current) {
  uint32_t F, dv, speed_factor, max_speed_factor;
  axes_int32_t prevF, currF;
  enum axis_e i;

  // Joining needs identical steps per mm on X and Y (M92 can change them).
  if (settings.steps_per_m[X] != settings.steps_per_m[Y]) {
    current->crossF = 0;
    return;
  }

  // Bail out if there's nothing to join (e.g. first movement after a pause).
  if ( ! prev)
    return;

  // Endstop moves stop on their own, don't join them.
  if (prev->endstop_check || current->endstop_check) {
    current->crossF = 0;
    return;
  }

  // We always look at the smaller of both combined speeds,
  // else we'd interpret intended speed changes as jerk.
  F = prev->endpoint.F;
  if (current->endpoint.F < F)
    F = current->endpoint.F;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("Distance: %lu, then %lu\n"),
               prev->distance, current->distance);

  // Find individual axis speeds. Eight muldiv()s, cheap since they use a
  // 64 bit intermediate (UMULL + UDIV) instead of the AVR bit loop.
  for (i = X; i < AXIS_COUNT; i++) {
    prevF[i] = muldiv(prev->delta[i], F, prev->total_steps);
    currF[i] = muldiv(current->delta[i], F, current->total_steps);
  }

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("prevF: %ld  %ld  %ld  %ld\ncurrF: %ld  %ld  %ld  %ld\n"),
               prevF[X], prevF[Y], prevF[Z], prevF[E],
               currF[X], currF[Y], currF[Z], currF[E]);

  /**
   * What we want is (for each axis):
   *
   *   delta velocity = dv = |v1 - v2| < max_jerk
   *
   * In case this isn't satisfied, we can slow down by some factor x until
   * the equitation is satisfied:
   *
   *   x * |v1 - v2| < max_jerk
   *
   * Now computation is pretty straightforward:
   *
   *        max_jerk
   *   x = -----------
   *        |v1 - v2|
   *
   *   if x > 1: continue full speed
   *   if x < 1: v = v_max * x
   *
   * See also: https://github.com/Traumflug/Teacup_Firmware/issues/45
   */
  max_speed_factor = (uint32_t)2 << 8;

  for (i = X; i < AXIS_COUNT; i++) {
    if (get_direction(prev, i) == get_direction(current, i))
      dv = currF[i] > prevF[i] ? currF[i] - prevF[i] : prevF[i] - currF[i];
    else
      dv = currF[i] + prevF[i];

    if (dv) {
      speed_factor = (maximum_jerk_P[i] << 8) / dv;
      if (speed_factor < max_speed_factor)
        max_speed_factor = speed_factor;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(("%c: dv %lu of %lu   factor %lu of %lu\n"),
                   'X' + i, dv, maximum_jerk_P[i],
                   speed_factor, (uint32_t)1 << 8);
    }
  }

  if (max_speed_factor >= ((uint32_t)1 << 8))
    current->crossF = F;
  else
    current->crossF = (F * max_speed_factor) >> 8;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("Cross speed reduction from %lu to %lu\n"),
               F, current->crossF);

  return;
}

/// Previous slot in the movement queue ring buffer.
static DDA *mb_prev(DDA *dda) {
  return (dda == movebuffer) ? &movebuffer[MOVEBUFFER_SIZE - 1] : dda - 1;
}

/// Next slot in the movement queue ring buffer.
static DDA *mb_next(DDA *dda) {
  return (dda == &movebuffer[MOVEBUFFER_SIZE - 1]) ? movebuffer : dda + 1;
}

/**
 * \brief Set the speed at the junction of two moves.
 *
 * \param [in] prev is the earlier move.
 * \param [in] dda is the move following prev.
 * \param [in] vsq is the speed at the junction, squared, (mm/min)^2.
 *
 * \return 1 on success, 0 if prev became live or finished meanwhile. Then
 *         nothing is written and the junction keeps its speed.
 *
 * Exit speed of prev and entry speed of dda are written together with
 * interrupts locked, so the step interrupt always sees matching speeds at
 * each junction. As prev isn't live, dda isn't live either.
 *
 * The speed along the path is the same on both sides, but speed and
 * acceleration of the fast axis may differ, so each move gets its own ramp
 * position.
 */
static uint8_t plan_commit(DDA *prev, DDA *dda, float vsq) {
  uint32_t end_steps, start_steps, c;
  uint8_t ok = 0;

  end_steps = (uint32_t)(vsq * prev->n_per_vsq);
  start_steps = (uint32_t)(vsq * dda->n_per_vsq);
  c = dda_c_for_n(dda, start_steps);

  ATOMIC_START();
    if ( ! prev->live && ! prev->done) {
      prev->end_steps = end_steps;
      dda->start_steps = start_steps;
      dda->n = (int32_t)start_steps;
      dda->c = c;
      ok = 1;
    }
  ATOMIC_END();

  if (ok)
    dda->entry_vsq = vsq;

  #ifdef DEBUG
    if (ok)
      lookahead_joined++;
    else
      lookahead_timeout++;
  #endif

  return ok;
}

/**
 * \brief Plan the speeds of all queued moves after adding a new one.
 * \details Full look-ahead over the movement queue: the new move ends with a
 * full stop, all moves before it get the highest entry and exit speeds which
 * are reachable within the acceleration limits. Many short moves (arcs,
 * rounded perimeters) no longer need to be able to stop within each single
 * move, they share the deceleration distance.
 *
 * Speeds are handled along the path, squared, in (mm/min)^2. A move of
 * length s with acceleration a can change the squared speed by 2 * a * s
 * (dda->delta_vsq, derived from total_steps). Limits per junction are the
 * crossing speed (dda->crossF, see dda_find_crossing_speed(), which is also
 * below the feedrate of both moves).
 *
 * 1. Walk back from the new move to the oldest move whose entry speed can
 *    still change. The previous move is live (entry speed can't change), or
 *    the entry speed is known to be final (dda->plan_fixed).
 * 2. Reverse pass, newest to oldest: each move must be able to decelerate to
 *    the entry speed of the next move, the new move to zero.
 * 3. Forward pass, oldest to newest: each move must be able to accelerate to
 *    the entry speed of the next move. The result is written junction by
 *    junction, see plan_commit().
 *
 * A move becoming live meanwhile freezes its junctions. Then planning starts
 * over, with all junctions written so far being valid. Adding a move never
 * lowers planned speeds. Between two writes one move has its new entry
 * speed and its old exit speed; it could run like this only if the move
 * before it started and finished within these few microseconds.
 *
 * An entry speed is final when it equals the crossing speed, or when the
 * previous move accelerates all the way from a final entry speed. This keeps
 * the walk back short on long queues.
 *
 * This function is expected to be called from within dda_create(), after
 * dda_find_crossing_speed(). The new move isn't queued yet.
 *
 * \param [in] current is the DDA structure of the move currently created.
 */
TEACUP_HOT
void dda_plan(DDA *current) {
  DDA *first, *prev, *dda;
  float ratio, next_vsq, vsq, limit;
  uint8_t i, retries, fixed;

  // The new move isn't queued yet, the step interrupt can't start it.
  // It ends with a full stop.
  current->start_steps = 0;
  current->end_steps = 0;
  current->n = 0;
  current->c = dda_c_for_n(current, 0);
  current->entry_vsq = 0.f;
  current->plan_fixed = 1;

  // Path speed -> fast axis speed -> ramp steps.
  ratio = current->distance ?
          (float)current->fast_um / (float)current->distance : 0.f;
  current->n_per_vsq = ratio * ratio * acc_ramp_per_fsq(current->fast_axis);
  if (current->n_per_vsq > 0.f)
    current->delta_vsq = (float)current->total_steps / current->n_per_vsq;
  else {
    current->delta_vsq = 0.f;
    current->crossF = 0;
  }
  current->max_entry_vsq = (float)current->crossF * (float)current->crossF;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("Plan: crossF %lu, %lu steps\n"),
               current->crossF, current->total_steps);

  // Not joined with the previous move: nothing to plan.
  if (current->crossF == 0)
    return;
  current->plan_fixed = 0;

  for (retries = MOVEBUFFER_SIZE; retries; retries--) {
    // 1. Find the oldest move whose entry speed can still change.
    first = current;
    for (i = MOVEBUFFER_SIZE - 1; i; i--) {
      if (first->plan_fixed)
        break;
      prev = mb_prev(first);
      if (prev->live || prev->done) {
        // The previous move runs already, its exit speed is final.
        first->plan_fixed = 1;
        break;
      }
      first = prev;
    }

    if (first == current)
      return;

    // 2. Reverse pass.
    next_vsq = 0.f;
    for (dda = current; dda != first; dda = mb_prev(dda)) {
      vsq = next_vsq + dda->delta_vsq;
      if (vsq > dda->max_entry_vsq)
        vsq = dda->max_entry_vsq;
      dda->plan_vsq = vsq;
      next_vsq = vsq;
    }

    // 3. Forward pass and writing the results.
    prev = first;
    for (;;) {
      dda = mb_next(prev);
      vsq = dda->plan_vsq;
      limit = prev->entry_vsq + prev->delta_vsq;
      if (vsq >= limit) {
        vsq = limit;
        fixed = prev->plan_fixed;
      }
      else
        fixed = 0;
      if (vsq >= dda->max_entry_vsq)
        fixed = 1;

      if (vsq != dda->entry_vsq && ! plan_commit(prev, dda, vsq))
        break;                            // prev became live, start over.
      dda->plan_fixed = fixed;

      if (dda == current)
        return;
      prev = dda;
    }
  }
}

#endif /* LOOKAHEAD */
