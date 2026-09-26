#include "dda.h"

/** \file
  \brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "dda_maths.h"
#include "dda_kinematics.h"
#include "dda_lookahead.h"
#include "cpu.h"
#include "timer.h"
#include "serial.h"
#include "clock.h"
#include "gcode_parse.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"
#include "pinio.h"
#include "home.h"
#include "bed_leveling.h"
#include "graycode.h"
#include "settings.h"
#include "filament.h"
#include "linear_advance.h"
#include "input_shaping.h"

#include "atomic.h"

#ifdef DC_EXTRUDER
  #include "heater.h"
#endif

/*
  position tracking
*/

/// \var startpoint
/// \brief target position of last move in queue
TARGET startpoint;

/// \var startpoint_steps
/// \brief target position of last move in queue, expressed in steps
TARGET startpoint_steps;

/// \var current_position
/// \brief actual position of extruder head
/// \todo make current_position = real_position (from endstops) + offset from G28 and friends
TARGET current_position;

/// \var move_state
/// \brief numbers for tracking the current state of movement
MOVE_STATE move_state;

/// Maximum allowed feedrate on each axis: settings.max_feedrate (M203).
#define maximum_feedrate_P settings.max_feedrate

/*! Set the direction of the 'n' axis
*/
static void set_direction(DDA *dda, enum axis_e n, int32_t delta) {
  uint8_t dir = (delta >= 0) ? 1 : 0;

  if (n == X)
    dda->x_direction = dir;
  else if (n == Y)
    dda->y_direction = dir;
  else if (n == Z)
    dda->z_direction = dir;
  else if (n == E)
    dda->e_direction = dir;
}

/*! Find the direction of the 'n' axis
*/
int8_t get_direction(DDA *dda, enum axis_e n) {
  if ((n == X && dda->x_direction) ||
      (n == Y && dda->y_direction) ||
      (n == Z && dda->z_direction) ||
      (n == E && dda->e_direction))
    return 1;
  else
    return -1;
}

#ifdef ACCELERATION_RAMPING
/**
  Step interval at ramp position n, n steps from standstill on the fast axis.

  \param dda the move, needs c0 and c_min.
  \param n   ramp position, steps of the fast axis from standstill.

  \return Timer ticks between two steps, not below dda->c_min.
*/
TEACUP_HOT
uint32_t dda_c_for_n(const DDA *dda, uint32_t n) {
  uint32_t c;

  if (n == 0)
    c = dda->c0;
  else
    // Explicit formula: c0 * (sqrt(n + 1) - sqrt(n)),
    // approximation here: c0 * (1 / (2 * sqrt(n))).
    // This >> 13 looks odd, but is verified with the explicit formula.
    #if __FPU_PRESENT
    // Exact square root, not rounded to an integer: with look-ahead moves
    // run near the top of their ramps, where n is a few hundred only and
    // one unit of sqrt(n) would be a speed step of 5 %.
    c = (uint32_t)((float)dda->c0 /
                   (2.0f * teacup_sqrtf((float)n)));
    #else
    c = (dda->c0 * int_inv_sqrt(n)) >> 13;
    #endif

  if (c < dda->c_min)
    c = dda->c_min;

  return c;
}
#endif /* ACCELERATION_RAMPING */

#ifdef ACCELERATION_RAMPING
/**
  Acceleration of a move along its path, mm/s^2.

  \param dda      the move, needs delta[].
  \param delta_um distance of each axis, um.
  \param distance length of the path, um.
  \param la_kr    linear advance K (s) times mm of filament per mm of path,
                  0 without linear advance.

  M204 R for retracts and primes (E only), M204 T for travel (no E),
  M204 P for printing, like Marlin. Each axis gets its share of the
  acceleration along the path, delta_um[i] / distance; where this exceeds
  the M201 limit of that axis, the acceleration is reduced. So a diagonal
  accelerates with the M204 value, not with the value of its fast axis
  times sqrt(2).

  With linear advance the extruder gets an extra speed of K * a * (E per
  path) while accelerating. Like Marlin, the acceleration is reduced so this
  stays below the E jerk (M205 E).
*/
static float move_acceleration(const DDA *dda, const axes_uint32_t delta_um,
                               uint32_t distance, float la_kr) {
  float acc, limit;
  enum axis_e i;

  if (dda->delta[X] == 0 && dda->delta[Y] == 0 && dda->delta[Z] == 0)
    acc = (float)settings.accel_retract;
  else if (dda->delta[E] == 0)
    acc = (float)settings.accel_travel;
  else
    acc = (float)settings.acceleration;

  for (i = X; i < AXIS_COUNT; i++) {
    if (delta_um[i] == 0)
      continue;
    limit = (float)settings.max_accel[i] * (float)distance / (float)delta_um[i];
    if (limit < acc)
      acc = limit;
  }

  if (la_kr > 0.f && settings.max_jerk[E]) {
    limit = (float)settings.max_jerk[E] / 60.f / la_kr;
    if (limit < acc)
      acc = limit;
  }

  return (acc < 1.f) ? 1.f : acc;
}
#endif /* ACCELERATION_RAMPING */

void dda_init(void) {

  // set up default feedrate
  if (startpoint.F == 0)
    startpoint.F = next_target.target.F = SEARCH_FEEDRATE_Z;
  if (startpoint.e_multiplier == 0)
    startpoint.e_multiplier = next_target.target.e_multiplier = 256;
  if (startpoint.f_multiplier == 0)
    startpoint.f_multiplier = next_target.target.f_multiplier = 256;
}

/*! Distribute a new startpoint to DDA's internal structures without any movement.

  This is needed for example after homing or a G92. The new location must be in startpoint already.
*/
void dda_new_startpoint(void) {
  if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
    int32_t z_offset = bed_level_offset(startpoint.axis);
    sersendf_P(("\nReset: X %lq  Y %lq  Z %lq  Zofs %lq  F %lu\n"),
               startpoint.axis[X], startpoint.axis[Y],
               startpoint.axis[Z], z_offset, startpoint.F);
  }
  axes_um_to_steps(startpoint.axis, startpoint_steps.axis);
  startpoint_steps.axis[E] = um_to_steps(startpoint.axis[E], E);
}

/**
  Create a DDA using startpoint, startpoint_steps and a target, save to passed
  location so we can write directly into the queue.

  \param *dda pointer to a dda_queue entry to overwrite
  \param *target the target position of this move

  \ref startpoint the beginning position of this move

  This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

  It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

  This algorithm is the main limiting factor when queuing movements and can
  become a limitation to print speed if there are lots of tiny, fast movements.

 * Regarding lookahead, we can distinguish everything into these cases:
 *
 * 1. Standard movement. To be joined with the previous move.
 * 2. Movement after a pause. This interrupts lookahead, and invalidates
 *    prev_dda and prev_distance.
 * 3. Non-move, e.g. a wait for temp. This also interrupts lookahead and makes
 *    prev_dda and prev_distance invalid. There might be more such cases in the
 *    future, e.g. when heater or fan changes are queued up, too.
 * 4. Nullmove due to no movement expected, e.g. a pure speed change. This
 *    shouldn't interrupt lookahead and be handled af if the change would come
 *    with the next movement.
 * 5. Nullmove due to movement smaller than a single step. Shouldn't interrupt
 *    lookahead either, but this small distance should be added to the next
 *    movement.
 * 6. Lookahead calculation too slow, a move became live meanwhile. This is
 *    handled in dda_plan() already.
 */
TEACUP_HOT
void dda_create(DDA *dda, const TARGET *target) {
  axes_uint32_t delta_um;
  axes_int32_t steps;
  uint32_t distance;
  #ifndef ACCELERATION_TEMPORAL
  uint32_t c_limit, c_limit_calc;
  #endif
  enum axis_e i;
  #ifdef BACKLASH_COMPENSATION
  uint32_t backlash_um = 0;
  #endif
  #ifdef ACCELERATION_RAMPING
  // Number the moves to identify them; allowed to overflow.
  static uint8_t idcnt = 0;
  #endif
  #ifdef LOOKAHEAD
  static DDA* prev_dda = NULL;

  if (prev_dda && prev_dda->done)
    prev_dda = NULL;
  #endif

  // We end at the passed target.
  memcpy(&(dda->endpoint), target, sizeof(TARGET));

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("\nCreate: X %lq  Y %lq  Z %lq  F %lu\n"),
              dda->endpoint.axis[X], dda->endpoint.axis[Y],
              dda->endpoint.axis[Z], dda->endpoint.F);

  // Apply feedrate multiplier.
  if (dda->endpoint.f_multiplier != 256 && ! dda->endstop_check) {
    dda->endpoint.F *= dda->endpoint.f_multiplier;
    dda->endpoint.F += 128;
    dda->endpoint.F /= 256;
  }

  #ifdef LOOKAHEAD
    // Set the start and stop speeds to zero for now = full stops between
    // moves. Also fallback if lookahead calculations fail to finish in time.
    dda->crossF = 0;
    dda->start_steps = 0;
    dda->end_steps = 0;
  #endif
  #ifdef ACCELERATION_RAMPING
    // Give this move an identifier.
    dda->id = idcnt++;
  #endif

  // Handle bot axes. They're subject to kinematics considerations.
  code_axes_to_stepper_axes(&startpoint, target, delta_um, steps);
  for (i = X; i < E; i++) {
    int32_t delta_steps;

    delta_steps = steps[i] - startpoint_steps.axis[i];
    dda->delta[i] = (uint32_t)labs(delta_steps);
    startpoint_steps.axis[i] = steps[i];

    set_direction(dda, i, delta_steps);
  }

  #ifdef BACKLASH_COMPENSATION
    /**
      Z backlash (M425 Z F): when Z reverses, the nut first crosses the play
      of the thread. The move gets that many extra Z steps (spread over the
      move by the Bresenham algorithm), the position doesn't count them.
      Homing and probing moves take up the play themselves.
    */
    if (dda->delta[Z]) {
      static uint8_t z_last_dir = 2;          // 2 = unknown yet.
      uint8_t dir = dda->z_direction;

      if (z_last_dir != 2 && dir != z_last_dir && ! dda->endstop_check &&
          settings.backlash_z && settings.backlash_f) {
        backlash_um = settings.backlash_z * settings.backlash_f / 1000;
        dda->delta[Z] += (uint32_t)um_to_steps((int32_t)backlash_um, Z);
      }
      z_last_dir = dir;
    }
  #endif

  // Handle extruder axes. They act independently from the bots kinematics
  // type, but are subject to other special handling.
  steps[E] = um_to_steps(target->axis[E], E);

  // Apply extrusion multiplier.
  if (target->e_multiplier != 256) {
    steps[E] *= target->e_multiplier;
    steps[E] += 128;
    steps[E] /= 256;
  }

  if ( ! target->e_relative) {
    int32_t delta_steps;

    delta_um[E] = (uint32_t)labs(target->axis[E] - startpoint.axis[E]);
    delta_steps = steps[E] - startpoint_steps.axis[E];
    dda->delta[E] = (uint32_t)labs(delta_steps);
    startpoint_steps.axis[E] = steps[E];

    set_direction(dda, E, delta_steps);
  }
  else {
    /*
      Relative E: carry the fraction of a step from move to move. Rounding
      each move on its own loses or gains up to half a step per move, which
      adds up over many short moves (M83 slicer output, arc segments).
      Remainder in 1 / (UM_PER_METER * 256) steps, starting at one half, so
      the running total is rounded to nearest.
    */
    static int64_t e_rel_rem = (int64_t)UM_PER_METER * 256 / 2;
    const int64_t den = (int64_t)UM_PER_METER * 256;
    int64_t num = (int64_t)target->axis[E] * steps_per_m_P[E] *
                  target->e_multiplier + e_rel_rem;
    int64_t st = num / den;

    if (num - st * den < 0)
      st--;                               // Floor, also for negative moves.
    e_rel_rem = num - st * den;
    steps[E] = (int32_t)st;

    // When we get more extruder axes:
    // for (i = E; i < AXIS_COUNT; i++) { ...
    delta_um[E] = (uint32_t)labs(target->axis[E]);
    dda->delta[E] = (uint32_t)labs(steps[E]);
    dda->e_direction = (target->axis[E] >= 0)?1:0;
  }

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("[%ld,%ld,%ld,%ld]"),
              target->axis[X] - startpoint.axis[X], target->axis[Y] - startpoint.axis[Y],
              target->axis[Z] - startpoint.axis[Z], target->axis[E] - startpoint.axis[E]);

  // Admittedly, this looks like it's overcomplicated. Why store three 32-bit
  // values if storing an axis number would be fully sufficient? Well, I'm not
  // sure, but my feeling says that when we achieve true circles and Beziers,
  // we'll have total_steps which matches neither of X, Y, Z or E. Accordingly,
  // keep it for now. --Traumflug
  for (i = X; i < AXIS_COUNT; i++) {
    if (i == X || dda->delta[i] > dda->total_steps) {
      dda->total_steps = dda->delta[i];
      dda->fast_um = delta_um[i];
      dda->fast_axis = i;
    }
  }

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P((" [ts:%lu"), dda->total_steps);

  if (dda->total_steps == 0) {
    dda->nullmove = 1;
    startpoint.F = dda->endpoint.F;
  }
  else {
    // get steppers ready to go
    power_on();
    stepper_enable();
    x_enable();
    y_enable();
    #ifndef Z_AUTODISABLE
      z_enable();
    // #else Z is enabled in dda_start().
    #endif
    e_enable();

    // Exact length of the move (FPU), the speed along it depends on it.
    if (delta_um[Z] == 0)
      distance = distance_2d(delta_um[X], delta_um[Y]);
    else if (delta_um[X] == 0 && delta_um[Y] == 0)
      distance = delta_um[Z];
    else
      distance = distance_3d(delta_um[X], delta_um[Y], delta_um[Z]);

    if (distance < 1)
      distance = delta_um[E];

    #ifdef BACKLASH_COMPENSATION
      // The backlash steps count for the Z speed and acceleration limits,
      // not for the length of the move.
      delta_um[Z] += backlash_um;
      if (dda->fast_axis == Z)
        dda->fast_um = delta_um[Z];
    #endif

    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P((",ds:%lu"), distance);

    #ifdef	ACCELERATION_TEMPORAL
      // bracket part of this equation in an attempt to avoid overflow:
      // 60 * 16 MHz * 5 mm is > 32 bits
      uint32_t move_duration, md_candidate;

      // md = um * (60s/min) * (ticks/s) / (1000um/mm) / (mm/min)
      //    = um * (mm/1000um) * (min/mm) * (60s/min) * (ticks/s)
      //    =      (mm/1000)   * (   /mm) * (60s)     * (ticks/s)
      //    =      (1/1000              ) * (60       *  ticks  ) 
      move_duration = distance * (60UL * (F_CPU / 1000) / dda->endpoint.F);
      for (i = X; i < AXIS_COUNT; i++) {
        md_candidate = delta_um[i] * (60UL * (F_CPU / 1000) /
                      maximum_feedrate_P[i]);
        if (md_candidate > move_duration)
          move_duration = md_candidate;
      }
    #else
      // pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
      // mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
      //   note: um (distance) * 60000 == mm * 60000000
      // so in the interrupt we must simply calculate
      // mm.us per step.min / mm per min (F) = us per step

      // break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
      // calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
      // 2^32/6000 is about 715mm which should be plenty

      // changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
      // timer.c timer_set() routine altered for same reason

      // changed distance * 6000 .. * F_CPU / 100000 to
      //         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
      uint32_t move_duration = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);

      // similarly, find out how fast we can run our axes.
      // do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.
      // TODO: instead of calculating c_min directly, it's probably more simple
      //       to calculate (maximum) move_duration for each axis, like done for
      //       ACCELERATION_TEMPORAL above. This should make re-calculating the
      //       allowed F easier.
      c_limit = 0;
      for (i = X; i < AXIS_COUNT; i++) {
        c_limit_calc = (delta_um[i] * 2400L) /
                      dda->total_steps * (F_CPU / 40000) /
                      (maximum_feedrate_P[i]);
        if (c_limit_calc > c_limit)
          c_limit = c_limit_calc;
      }
    #endif
    #ifdef ACCELERATION_REPRAP
      // c is initial step time in IOclk ticks
      dda->c = move_duration / startpoint.F;
      if (dda->c < c_limit)
        dda->c = c_limit;
      dda->end_c = move_duration / dda->endpoint.F;
      if (dda->end_c < c_limit)
        dda->end_c = c_limit;

      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P((",md:%lu,c:%lu"), move_duration, dda->c);

      if (dda->c != dda->end_c) {
        uint32_t stF = startpoint.F / 4;
        uint32_t enF = dda->endpoint.F / 4;
        // now some constant acceleration stuff, courtesy of http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
        uint32_t ssq = (stF * stF);
        uint32_t esq = (enF * enF);
        int32_t dsq = (int32_t) (esq - ssq) / 4;

        uint8_t msb_ssq = msbloc(ssq);
        uint8_t msb_tot = msbloc(dda->total_steps);

        // the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
        // at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
        // but if ssq-esq is small, ssq/dsq is only a few bits
        // we'll have to do it a few different ways depending on the msb locations of each
        if ((msb_tot + msb_ssq) <= 30) {
          // we have room to do all the multiplies first
          if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
            serial_writechar('A');
          dda->n = ((int32_t) (dda->total_steps * ssq) / dsq) + 1;
        }
        else if (msb_tot >= msb_ssq) {
          // total steps has more precision
          if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
            serial_writechar('B');
          dda->n = (((int32_t) dda->total_steps / dsq) * (int32_t) ssq) + 1;
        }
        else {
          // otherwise
          if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
            serial_writechar('C');
          dda->n = (((int32_t) ssq / dsq) * (int32_t) dda->total_steps) + 1;
        }

        if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
          sersendf_P(("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->end_c, dda->n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

        dda->accel = 1;
      }
      else
        dda->accel = 0;
    #elif defined ACCELERATION_RAMPING
      dda->c_min = move_duration / dda->endpoint.F;
      if (dda->c_min < c_limit) {
        dda->c_min = c_limit;
        dda->endpoint.F = move_duration / dda->c_min;
      }

      // Lookahead can deal with 16 bits ( = 1092 mm/s), only.
      if (dda->endpoint.F > 65535)
        dda->endpoint.F = 65535;

      {
        /**
          Ramps count steps of the fast axis. Its acceleration is its share
          of the acceleration along the path. Ramp position n (steps from
          standstill) of fast axis speed v (mm/min) is v^2 / ramp_div,
          s = v^2 / (2 * a); 7200000 = 60 * 60 * 1000 * 2 (mm/min -> mm/s,
          steps/m -> steps/mm, factor 2).
        */
        float ratio = distance ? (float)dda->fast_um / (float)distance : 1.f;
        float la_kr = 0.f;
        float acc_fast, spm;

        #ifdef LINEAR_ADVANCE
          /**
            Linear advance for printing moves: X or Y with E forward, not
            much more E than path (no primes or wipes with E), like Marlin.
            Advance in E steps = la_factor / c, K * E steps per second.
          */
          dda->la_factor = 0.f;
          if (settings.la_k && dda->delta[E] && dda->e_direction &&
              (dda->delta[X] || dda->delta[Y]) && distance &&
              delta_um[E] <= 3 * distance) {
            float k = (float)settings.la_k * 0.0001f;

            la_kr = k * (float)delta_um[E] / (float)distance;
            dda->la_factor = k * (float)F_CPU * (float)dda->delta[E] /
                             (float)dda->total_steps;
          }
        #endif
        acc_fast = move_acceleration(dda, delta_um, distance, la_kr) * ratio;
        spm = (float)settings.steps_per_m[dda->fast_axis];
        float ramp_div, ramp, fast_f;

        if (acc_fast < 1.f)
          acc_fast = 1.f;
        ramp_div = 7200000.f * acc_fast / spm;
        // Step interval from standstill, c0 = F_CPU * sqrt(2 / a) in steps.
        dda->c0 = (uint32_t)((float)F_CPU /
                             teacup_sqrtf(spm * acc_fast / 2000.f));

        fast_f = (float)dda->endpoint.F * ratio;
        ramp = fast_f * fast_f / ramp_div;
        if (ramp < (float)(dda->total_steps / 2))
          dda->rampup_steps = (uint32_t)ramp;
        else
          dda->rampup_steps = dda->total_steps / 2;
        dda->rampdown_steps = dda->total_steps - dda->rampup_steps;

        #ifdef LOOKAHEAD
          // Path speed^2 -> ramp position, for the planner.
          dda->n_per_vsq = ratio * ratio / ramp_div;
        #endif
      }

      #ifdef LOOKAHEAD
        dda->distance = distance;
        dda_find_crossing_speed(prev_dda, dda);
        // Re-plan entry and exit speeds of the whole queue, this move ends
        // with a full stop. Also sets dda->n and dda->c.
        dda_plan(dda);
      #else
        dda->n = 0;
        dda->c = dda->c0;
      #endif

    #elif defined ACCELERATION_TEMPORAL
      // TODO: calculate acceleration/deceleration for each axis
      for (i = X; i < AXIS_COUNT; i++) {
        dda->step_interval[i] = 0xFFFFFFFF;
        if (dda->delta[i])
          dda->step_interval[i] = move_duration / dda->delta[i];
      }

      dda->c = 0xFFFFFFFF;
      dda->axis_to_step = X; // Safety value
      for (i = X; i < AXIS_COUNT; i++) {
        if (dda->step_interval[i] < dda->c) {
          dda->axis_to_step = i;
          dda->c = dda->step_interval[i];
        }
      }

    #else
      dda->c = move_duration / dda->endpoint.F;
      if (dda->c < c_limit)
        dda->c = c_limit;
    #endif

    // next dda starts where we finish
    memcpy(&startpoint, &dda->endpoint, sizeof(TARGET));
    if (startpoint.e_relative)
      startpoint.axis[E] = 0;
    #ifdef LOOKAHEAD
      prev_dda = dda;
    #endif
  } /* ! dda->total_steps == 0 */

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    serial_writestr_P(("] }\n"));
}

/** Start a prepared DDA

  \param *dda Pointer to entry in the movement queue to start.

  This function actually begins the move described by the passed DDA entry.
  Called from both, inside and outside of interrupts.
*/
TEACUP_HOT
TEACUP_STEP_RAMFUNC void dda_start(DDA *dda) {

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(("Start: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.axis[X], dda->endpoint.axis[Y],
               dda->endpoint.axis[Z], dda->endpoint.F);

  // Get ready to go.
  psu_timeout = 0;
  #ifdef Z_AUTODISABLE
    if (dda->delta[Z])
      z_enable();
  #endif
  if (dda->endstop_check)
    endstops_on();

  #ifdef FILAMENT_RUNOUT_PIN
    // Filament fed forward, for the runout distance.
    if (dda->e_direction)
      filament_e_steps += dda->delta[E];
  #endif

  // Set direction outputs. With linear advance the E generator sets the E
  // direction, see motion/linear_advance.c, with input shaping the shaper
  // those of shaped axes, see motion/input_shaping.c. Moves with endstop
  // checks aren't shaped.
  #ifdef INPUT_SHAPING
    if (dda->endstop_check)
      shaper_dir_unknown();
    if ( ! shaper_on[0] || dda->endstop_check)
      x_direction(dda->x_direction);
    if ( ! shaper_on[1] || dda->endstop_check)
      y_direction(dda->y_direction);
  #else
    x_direction(dda->x_direction);
    y_direction(dda->y_direction);
  #endif
  z_direction(dda->z_direction);
  #ifndef LINEAR_ADVANCE
    e_direction(dda->e_direction);
  #endif

  #ifdef DC_EXTRUDER
    if (dda->delta[E])
      heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
  #endif

  // Initialise state variables.
  move_state.counter[X] = move_state.counter[Y] = move_state.counter[Z] = \
    move_state.counter[E] = -(dda->total_steps >> 1);
  move_state.endstop_stop = 0;
  memcpy(&move_state.steps[X], &dda->delta[X], sizeof(uint32_t) * 4);
  #ifdef ACCELERATION_TEMPORAL
    move_state.time[X] = move_state.time[Y] = \
      move_state.time[Z] = move_state.time[E] = 0UL;
  #endif

  // Ensure this DDA starts.
  dda->live = 1;

  // Set timeout for first step.
  timer_set(dda->c, 0);
}

/**
  \brief Do per-step movement maintenance.

  \param *dda the current move

  \details Most important task here is to update the Bresenham algorithm and
  to generate step pulses accordingly, this guarantees geometrical accuracy
  of the movement. Other tasks, like acceleration calculations, are moved
  into dda_clock() as much as possible.

  This is called from our timer interrupt every time a step needs to occur.
  Keep it as simple and fast as possible, this is most critical for the
  achievable step frequency.

  Note: it was tried to do this in loops instead of straight, repeating code.
        However, this resulted in at least 16% performance loss, no matter
        how it was done. On how to measure, see commit "testcases: Add
        config.h". On the various tries and measurement results, see commits
        starting with "DDA: Move axis calculations into loops, part 6".
*/
TEACUP_HOT
TEACUP_STEP_RAMFUNC void dda_step(DDA *dda) {
  #if defined ACCELERATION_TEMPORAL
    // Step pulses start here, see step_pulse_wait() before unstep().
    uint32_t step_start = DWT->CYCCNT;
  #else
    // Steps of all axes are collected and written with one BSRR access
    // per port. The step interrupt ends the pulses later, see timer.c.
    step_set_t steps;

    step_set_clear(&steps);
  #endif

  #if ! defined ACCELERATION_TEMPORAL
    if (move_state.steps[X]) {
      move_state.counter[X] -= dda->delta[X];
      if (move_state.counter[X] < 0) {
        move_state.counter[X] += dda->total_steps;
        #ifdef INPUT_SHAPING
          if (shaper_on[0] && ! dda->endstop_check)
            shaper_step(0, dda->x_direction);
          else
        #endif
        step_x(&steps);
        move_state.steps[X]--;
      }
    }
    if (move_state.steps[Y]) {
      move_state.counter[Y] -= dda->delta[Y];
      if (move_state.counter[Y] < 0) {
        move_state.counter[Y] += dda->total_steps;
        #ifdef INPUT_SHAPING
          if (shaper_on[1] && ! dda->endstop_check)
            shaper_step(1, dda->y_direction);
          else
        #endif
        step_y(&steps);
        move_state.steps[Y]--;
      }
    }
    if (move_state.steps[Z]) {
      move_state.counter[Z] -= dda->delta[Z];
      if (move_state.counter[Z] < 0) {
        move_state.counter[Z] += dda->total_steps;
        step_z(&steps);
        move_state.steps[Z]--;
      }
    }
    if (move_state.steps[E]) {
      move_state.counter[E] -= dda->delta[E];
      if (move_state.counter[E] < 0) {
        move_state.counter[E] += dda->total_steps;
        #ifdef LINEAR_ADVANCE
          la_step_e(dda->e_direction);
        #else
          step_e(&steps);
        #endif
        move_state.steps[E]--;
      }
    }

    if (step_set_any(&steps)) {
      step_output(&steps);
      timer_step_pulse_end();       // Schedule the end of the pulses.
    }
  #endif

  #ifdef ACCELERATION_REPRAP
    // linear acceleration magic, courtesy of http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
    if (dda->accel) {
      if ((dda->c > dda->end_c) && (dda->n > 0)) {
        uint32_t new_c = dda->c - (dda->c * 2) / dda->n;
        if (new_c <= dda->c && new_c > dda->end_c) {
          dda->c = new_c;
          dda->n += 4;
        }
        else
          dda->c = dda->end_c;
      }
      else if ((dda->c < dda->end_c) && (dda->n < 0)) {
        uint32_t new_c = dda->c + ((dda->c * 2) / -dda->n);
        if (new_c >= dda->c && new_c < dda->end_c) {
          dda->c = new_c;
          dda->n += 4;
        }
        else
          dda->c = dda->end_c;
      }
      else if (dda->c != dda->end_c) {
        dda->c = dda->end_c;
      }
      // else we are already at target speed
    }
  #endif

  #ifdef ACCELERATION_TEMPORAL
    /** How is this ACCELERATION TEMPORAL expected to work?

      All axes work independently of each other, as if they were on four
      different, synchronized timers. As we have not enough suitable timers,
      we have to share one for all axes.

      To do this, each axis maintains the time of its last step in
      move_state.time[]. This time is updated as the step is done, see early
      in dda_step(). To find out which axis is the next one to step, the time
      of each axis' next step is compared to the time of the step just done.
      Zero means this actually is the axis just stepped, the smallest value > 0
      wins.

      One problem undoubtedly arising is, steps should sometimes be done at
      {almost,exactly} the same time. We trust the timer to deal properly with
      very short or even zero periods. If a step can't be done in time, the
      timer shall do the step as soon as possible and compensate for the delay
      later. In turn we promise here to send a maximum of four such
      short-delays consecutively and to give sufficient time on average.
    */
    // This is the time which led to this call of dda_step().
    move_state.last_time = move_state.time[dda->axis_to_step] +
                           dda->step_interval[dda->axis_to_step];

    do {
      uint32_t c_candidate;
      enum axis_e i;

      step_start = DWT->CYCCNT;

      if (dda->axis_to_step == X) {
        x_step();
        move_state.steps[X]--;
        move_state.time[X] += dda->step_interval[X];
      }
      if (dda->axis_to_step == Y) {
        y_step();
        move_state.steps[Y]--;
        move_state.time[Y] += dda->step_interval[Y];
      }
      if (dda->axis_to_step == Z) {
        z_step();
        move_state.steps[Z]--;
        move_state.time[Z] += dda->step_interval[Z];
      }
      if (dda->axis_to_step == E) {
        e_step();
        move_state.steps[E]--;
        move_state.time[E] += dda->step_interval[E];
      }
      step_pulse_wait(step_start);
      unstep();

      // Find the next stepper to step.
      dda->c = 0xFFFFFFFF;
      for (i = X; i < AXIS_COUNT; i++) {
        if (move_state.steps[i]) {
          c_candidate = move_state.time[i] + dda->step_interval[i] -
                        move_state.last_time;
          if (c_candidate < dda->c) {
            dda->axis_to_step = i;
            dda->c = c_candidate;
          }
        }
      }

      // No stepper to step found? Then we're done.
      if (dda->c == 0xFFFFFFFF) {
        dda->live = 0;
        dda->done = 1;
        break;
      }
    } while (timer_set(dda->c, 1));

  #endif /* ACCELERATION_TEMPORAL */

  // If there are no steps left or an endstop stop happened, we have finished.
  //
  // TODO: with ACCELERATION_TEMPORAL this duplicates some code. See where
  //       dda->live is zero'd, about 10 lines above.
  #if ! defined ACCELERATION_TEMPORAL
    if (move_state.steps[dda->fast_axis] == 0)
  #else
    if (move_state.steps[X] == 0 && move_state.steps[Y] == 0 &&
        move_state.steps[Z] == 0 && move_state.steps[E] == 0)
  #endif
  {
    dda->live = 0;
    dda->done = 1;
    #ifdef LOOKAHEAD
    // If look-ahead was using this move, it could have missed our activation:
    // make sure the ids do not match.
    dda->id--;
    #endif
    #ifdef	DC_EXTRUDER
      heater_set(DC_EXTRUDER, 0);
    #endif
    #ifdef Z_AUTODISABLE
      // Z stepper is only enabled while moving.
      z_disable();
    #endif

    // No need to restart timer here.
    // After having finished, dda_start() will do it.
  }
  else {
    psu_timeout = 0;
    #ifndef ACCELERATION_TEMPORAL
      timer_set(dda->c, 0);
    #endif
  }

  // Step outputs go low by the pulse end compare (timer.c), except with
  // ACCELERATION_TEMPORAL, which does its pulses above.
}

/*! Do regular movement maintenance.

  This should be called pretty often, like once every 1 or 2 milliseconds.

  Currently, this is checking the endstops and doing acceleration maths. These
  don't need to be checked/recalculated on every single step, so this code
  can be moved out of the highly time critical dda_step(). At high precision
  (slow) searches of the endstop, this function is called more often than
  dda_step() anyways.

  In the future, arc movement calculations might go here, too. Updating
  movement direction 500 times a second is easily enough for smooth and
  accurate curves!
*/
TEACUP_HOT
void dda_clock(void) {
  DDA *dda;
  static DDA *last_dda = NULL;
  uint8_t endstop_trigger = 0;
  #ifdef ACCELERATION_RAMPING
  uint32_t move_step_no, move_step, move_c;
  int32_t move_n;
  #ifndef LOOKAHEAD
  uint8_t recalc_speed;
  #endif
  #ifdef LINEAR_ADVANCE
  float la_adv;
  int32_t la_adv_steps;
  #endif
  uint8_t current_id ;
  #endif

  ATOMIC_START();
    dda = mb_tail_dda;
  ATOMIC_END();
  if (dda != last_dda) {
    move_state.debounce_count_x =
    move_state.debounce_count_z =
    move_state.debounce_count_y = 0;
    last_dda = dda;
  }

  if (dda == NULL) {
    #ifdef LINEAR_ADVANCE
      la_set_advance(0);
    #endif
    return;
  }

  // Caution: we mangle step counters here without locking interrupts. This
  //          means, we trust dda isn't changed behind our back, which could
  //          in principle (but rarely) happen if endstops are checked not as
  //          endstop search, but as part of normal operations.
  if (dda->endstop_check && ! move_state.endstop_stop) {
    // After an endstop edge interrupt the first matching reading counts.
    uint8_t endstop_need = endstops_irq_take() ? 1 : ENDSTOP_STEPS;

    #ifdef X_MIN_PIN
    if (dda->endstop_check & X_MIN_ENDSTOP) {
      if (x_min() == dda->endstop_stop_cond)
        move_state.debounce_count_x++;
      else
        move_state.debounce_count_x = 0;
      endstop_trigger = move_state.debounce_count_x >= endstop_need;
    }
    #endif
    #ifdef X_MAX_PIN
    if (dda->endstop_check & X_MAX_ENDSTOP) {
      if (x_max() == dda->endstop_stop_cond)
        move_state.debounce_count_x++;
      else
        move_state.debounce_count_x = 0;
      endstop_trigger = move_state.debounce_count_x >= endstop_need;
    }
    #endif

    #ifdef Y_MIN_PIN
    if (dda->endstop_check & Y_MIN_ENDSTOP) {
      if (y_min() == dda->endstop_stop_cond)
        move_state.debounce_count_y++;
      else
        move_state.debounce_count_y = 0;
      endstop_trigger = move_state.debounce_count_y >= endstop_need;
    }
    #endif
    #ifdef Y_MAX_PIN
    if (dda->endstop_check & Y_MAX_ENDSTOP) {
      if (y_max() == dda->endstop_stop_cond)
        move_state.debounce_count_y++;
      else
        move_state.debounce_count_y = 0;
      endstop_trigger = move_state.debounce_count_y >= endstop_need;
    }
    #endif

    #ifdef Z_MIN_PIN
    if (dda->endstop_check & Z_MIN_ENDSTOP) {
      if (z_min() == dda->endstop_stop_cond)
        move_state.debounce_count_z++;
      else
        move_state.debounce_count_z = 0;
      endstop_trigger = move_state.debounce_count_z >= endstop_need;
    }
    #endif
    #ifdef Z_MAX_PIN
    if (dda->endstop_check & Z_MAX_ENDSTOP) {
      if (z_max() == dda->endstop_stop_cond)
        move_state.debounce_count_z++;
      else
        move_state.debounce_count_z = 0;
      endstop_trigger = move_state.debounce_count_z >= endstop_need;
    }
    #endif

    // If an endstop is definitely triggered, stop the movement.
    if (endstop_trigger) {
      #ifdef ACCELERATION_RAMPING
        // For always smooth operations, don't halt apruptly,
        // but start deceleration here.
        ATOMIC_START();
          move_state.endstop_stop = 1;
          move_step_no = dda->total_steps - move_state.steps[dda->fast_axis];
          move_state.endstop_steps_trigger = move_step_no;

          if (move_step_no > dda->rampup_steps) {  // cruising?
            move_step_no = dda->total_steps - dda->rampdown_steps;
          }

          dda->rampdown_steps = move_step_no;
          dda->total_steps = move_step_no * 2;
          move_state.steps[dda->fast_axis] = move_step_no;
          move_state.endstop_steps_total =
            move_state.endstop_steps_trigger + move_step_no;
        ATOMIC_END();
        // Not atomic, because not used in dda_step().
        dda->rampup_steps = 0; // in case we're still accelerating
      #else
        // Bitfields share a byte with flags written in the step interrupt.
        ATOMIC_START();
          move_state.endstop_stop = 1;
          move_state.endstop_steps_trigger = move_state.endstop_steps_total =
            dda->total_steps - move_state.steps[dda->fast_axis];
          dda->live = 0;
          dda->done = 1;
        ATOMIC_END();
        #ifdef Z_AUTODISABLE
          z_disable();
        #endif
      #endif

      endstops_off();
    }
  } /* ! move_state.endstop_stop */

  #ifdef ACCELERATION_RAMPING
    // For maths about stepper speed profiles, see
    // http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
    // and http://www.atmel.com/images/doc8017.pdf (Atmel app note AVR446)
    ATOMIC_START();
      current_id = dda->id;
      move_step = move_state.steps[dda->fast_axis];
      // All other variables are read-only or unused in dda_step(),
      // so no need for atomic operations.
    ATOMIC_END();

    move_step_no = dda->total_steps - move_step;

  #ifdef LOOKAHEAD
    /**
      The speed profile is the lowest of: accelerating from the entry speed
      (start_steps), decelerating to the exit speed (end_steps) and
      cruising (c_min). This needs no rampup_steps and rampdown_steps, so
      the planner can change the speed at a junction of two moves by writing
      end_steps of the one and start_steps of the other only, see dda_plan().
      It does so only while neither of them is live.

      After an endstop stop start_steps and end_steps are zero (no joining
      for moves with endstop checks) and total_steps is twice the steps to
      go, so this decelerates to a stop, too.
    */
    move_n = (int32_t)(dda->start_steps + move_step_no);
    if (dda->end_steps + move_step < (uint32_t)move_n)
      move_n = (int32_t)(dda->end_steps + move_step);
    move_c = dda_c_for_n(dda, (uint32_t)move_n);
    #ifdef LINEAR_ADVANCE
      // Advance = K * E steps per second = la_factor / c.
      la_adv = dda->la_factor / (float)move_c;
      la_adv_steps = (la_adv < 1000000.f) ? (int32_t)(la_adv + 0.5f) : 1000000;
    #endif

    ATOMIC_START();
      // Apply only if dda didn't change underneath us, e.g. because the
      // next move became live meanwhile.
      if (current_id == dda->id) {
        dda->c = move_c;
        dda->n = move_n;
        #ifdef LINEAR_ADVANCE
          la_set_advance(la_adv_steps);
        #endif
      }
    ATOMIC_END();
  #else
    recalc_speed = 0;
    if (move_step_no <= dda->rampup_steps) {
      move_n = move_step_no;
      recalc_speed = 1;
    }
    else if (move_step_no >= dda->rampdown_steps) {
      move_n = move_step;
      recalc_speed = 1;
    }
    if (recalc_speed) {
      if (move_n == 0)
        move_c = dda->c0;
      else
        // Explicit formula: c0 * (sqrt(n + 1) - sqrt(n)),
        // approximation here: c0 * (1 / (2 * sqrt(n))).
        // This >> 13 looks odd, but is verified with the explicit formula.
        #if __FPU_PRESENT
        move_c = (dda->c0 / (2 * int_f_sqrt(move_n)));
        #else
        move_c = (dda->c0 * int_inv_sqrt(move_n)) >> 13;
        #endif

      if (move_c < dda->c_min) {
        // We hit max speed not always exactly.
        move_c = dda->c_min;

        // This is a hack which deals with movements with an unknown number of
        // acceleration steps. dda_create() sets a very high number, then,
        // but we don't want to re-calculate all the time.
        dda->rampup_steps = move_step_no;
        dda->rampdown_steps = dda->total_steps - dda->rampup_steps;
      }

      // Write results.
      ATOMIC_START();
        /**
          Apply new n & c values only if dda didn't change underneath us. It
          is possible for dda to be modified since fetching values in the
          ATOMIC above, e.g. when a new dda becomes live.

          In case such a change happened, values in the new dda are more
          recent than our calculation here, anyways.
        */
        if (current_id == dda->id) {
          dda->c = move_c;
          dda->n = move_n;
        }
      ATOMIC_END();
    }
    else {
      ATOMIC_START();
        if (current_id == dda->id)
          // This happens only when !recalc_speed, meaning we are cruising, not
          // accelerating or decelerating. So it pegs our dda->c at c_min if it
          // never made it as far as c_min. 
          dda->c = dda->c_min;
      ATOMIC_END();
    }
  #endif /* LOOKAHEAD */
  #endif /* ACCELERATION_RAMPING */
}

/// update global current_position struct
void update_current_position(void) {
  DDA *dda = mb_tail_dda;
  enum axis_e i;

  if (dda != NULL) {
    uint32_t axis_um;
    axes_int32_t delta_um;

    for (i = X; i < AXIS_COUNT; i++) {
      axis_um = steps_to_um(move_state.steps[i], i);
      delta_um[i] = (int32_t)get_direction(dda, i) * axis_um;
    }

    delta_to_axes(delta_um);

    for (i = X; i < AXIS_COUNT; i++) {
      current_position.axis[i] = dda->endpoint.axis[i] - delta_um[i];
    }

    // delta_um counts motor steps, which include the Z correction:
    // motor Z = logical Z + correction. Back to logical Z here.
    current_position.axis[Z] += bed_level_offset(dda->endpoint.axis);
    current_position.axis[Z] -= bed_level_offset(current_position.axis);

    current_position.F = dda->endpoint.F;
  }
  else {
    memcpy(&current_position, &startpoint, sizeof(TARGET));
  }
}

uint8_t dda_endstop_result(uint32_t *steps_trigger, uint32_t *steps_total) {
  if ( ! move_state.endstop_stop)
    return 0;
  *steps_trigger = move_state.endstop_steps_trigger;
  *steps_total = move_state.endstop_steps_total;
  return 1;
}
