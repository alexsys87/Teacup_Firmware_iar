/** \file
  \brief Z probe (BLTouch), see probe.h.

  BLTouch commands by servo pulse (M280 P0 S<angle>): 10 deploy, 90 stow,
  160 alarm release / reset, 120 self test, 60 switch mode. The probe
  signal goes to Z_MIN_PIN (active high). When the pin touches the bed the
  BLTouch gives a short pulse and pulls the pin in by itself; the endstop
  edge interrupt (hal/endstops.c) catches that pulse, the move stops with
  deceleration and dda_endstop_result() tells the step count at the
  trigger. So the measurement is independent of the overshoot.

  Every point is measured twice (Z_PROBE_SAMPLES 2): fast to find the bed,
  up by Z_PROBE_RETRACT, slowly again; the slow one counts.

  All probing happens with the Z correction suspended, i.e. in motor
  coordinates. Mesh values are bed heights in the Z coordinate system of
  the last Z homing: bed_z = nozzle Z at the trigger + probe offset Z.
*/

#include "probe.h"

#ifdef Z_PROBE

#include "servo.h"
#include "pinio.h"
#include "dda.h"
#include "dda_queue.h"
#include "dda_maths.h"
#include "home.h"
#include "clock.h"
#include "serial.h"
#include "sersendf.h"
#include "settings.h"
#include "bed_leveling.h"
#include "delay.h"
#include <stdlib.h>
#include <math.h>

#define BLTOUCH_DEPLOY      10
#define BLTOUCH_STOW        90
#define BLTOUCH_RESET       160

int32_t probe_offset[3];

void probe_defaults(void) {
  probe_offset[X] = (int32_t)(Z_PROBE_OFFSET_X * 1000.);
  probe_offset[Y] = (int32_t)(Z_PROBE_OFFSET_Y * 1000.);
  probe_offset[Z] = (int32_t)(Z_PROBE_OFFSET_Z * 1000.);
}

/// Wait, keep the printer running (temperatures, host, watchdog).
static void probe_wait_ms(uint32_t ms) {
  uint32_t start = clock_millis();

  while (clock_millis() - start < ms)
    clock_poll();
}

static void bltouch_cmd(uint16_t angle) {
  servo_write_angle(angle);
  probe_wait_ms(BLTOUCH_DELAY);
}

/// Current probe signal (1 = triggered or alarm).
static uint8_t probe_triggered(void) {
  uint8_t v;

  endstops_on();                        // Pull-ups, if configured.
  delay_us(20);
  v = z_min();
  endstops_off();
  return v;
}

void probe_init(void) {
  servo_init();
  servo_write_angle(BLTOUCH_STOW);
}

uint8_t probe_deploy(void) {
  // Never while moving: after a trigger the nozzle is still below the
  // trigger point until the raise is done, the pin would hit the bed.
  queue_wait();
  if (probe_triggered())
    bltouch_cmd(BLTOUCH_RESET);         // Alarm (blinking) or pin down.
  bltouch_cmd(BLTOUCH_DEPLOY);
  if (probe_triggered()) {
    bltouch_cmd(BLTOUCH_RESET);
    bltouch_cmd(BLTOUCH_DEPLOY);
    if (probe_triggered()) {
      serial_writestr("Error:BLTouch deploy failed\n");
      bltouch_cmd(BLTOUCH_STOW);
      return 0;
    }
  }
  return 1;
}

uint8_t probe_stow(void) {
  queue_wait();
  bltouch_cmd(BLTOUCH_STOW);
  if (probe_triggered()) {
    serial_writestr("Error:BLTouch stow failed\n");
    return 0;
  }
  return 1;
}

/// Move the nozzle to X, Y, Z (um, current coordinates), E stays.
static void probe_goto(int32_t x, int32_t y, int32_t z, uint32_t feed) {
  TARGET t = startpoint;

  t.axis[X] = x;
  t.axis[Y] = y;
  t.axis[Z] = z;
  t.F = feed;
  t.f_multiplier = 256;
  while (queue_full())
    clock_poll();
  enqueue(&t);
}

/**
  Move Z down to z_um until the probe triggers.
  \return 1 if triggered, *trigger = motor position (steps) at the trigger.
*/
static uint8_t probe_move(int32_t z_um, uint32_t feed, int32_t *trigger) {
  TARGET t = startpoint;
  int32_t start = startpoint_steps.axis[Z];
  uint32_t st_trigger, st_total;

  t.axis[Z] = z_um;
  t.F = feed;
  t.f_multiplier = 256;
  enqueue_home(&t, Z_MIN_ENDSTOP, 1);
  queue_wait();
  endstops_off();
  if ( ! dda_endstop_result(&st_trigger, &st_total))
    return 0;                           // Went all the way down.

  // Stopped early: the motors are where the steps say, not at the target.
  startpoint_steps.axis[Z] = start - (int32_t)st_total;
  zcorr_sync_logical();
  *trigger = start - (int32_t)st_trigger;
  return 1;
}

/**
  Probe down from the current position, at most down to z_low (um).
  \return 1 on success, *trigger = motor steps at the trigger.
*/
static uint8_t probe_down(int32_t z_low, int32_t *trigger) {
  if ( ! probe_deploy())
    return 0;
  if ( ! probe_move(z_low, Z_PROBE_FEEDRATE_FAST, trigger)) {
    serial_writestr("Error:Probing failed, no trigger\n");
    probe_stow();
    return 0;
  }
  #if Z_PROBE_SAMPLES > 1
  {
    const int32_t retract = (int32_t)(Z_PROBE_RETRACT * 1000.);
    int32_t z = steps_to_um(*trigger, Z);

    probe_goto(startpoint.axis[X], startpoint.axis[Y], z + retract,
               settings.max_feedrate[Z]);
    if ( ! probe_deploy())
      return 0;
    if ( ! probe_move(z - retract, Z_PROBE_FEEDRATE_SLOW, trigger)) {
      serial_writestr("Error:Probing failed, no trigger on the 2nd sample\n");
      probe_stow();
      return 0;
    }
  }
  #endif
  return 1;
}

/// Nozzle limits (soft limits with home offsets), um.
static int32_t lim_lo(enum axis_e a) {
  return (int32_t)((a == X ? X_MIN : Y_MIN) * 1000.) + home_offset[a];
}
static int32_t lim_hi(enum axis_e a) {
  return (int32_t)((a == X ? X_MAX : Y_MAX) * 1000.) + home_offset[a];
}

/// Raise to the clearance height if below, up by 'retract' at least.
static void probe_raise(int32_t above) {
  int32_t z = (int32_t)(Z_PROBE_CLEARANCE * 1000.);

  if (above > z)
    z = above;
  if (startpoint.axis[Z] < z)
    probe_goto(startpoint.axis[X], startpoint.axis[Y], z,
               settings.max_feedrate[Z]);
}

uint8_t probe_home_z(void) {
  const int32_t clearance = (int32_t)(Z_PROBE_CLEARANCE * 1000.);
  int32_t x, y, trigger, z_home;

  if ((axes_homed & (HOMED_X | HOMED_Y)) != (HOMED_X | HOMED_Y)) {
    serial_writestr("echo:Home X and Y before Z (probe)\n");
    return 0;
  }
  queue_wait();
  zcorr_suspend();

  // Z is unknown: lift a bit relative to where it is, then go to the
  // homing position.
  probe_goto(startpoint.axis[X], startpoint.axis[Y],
             startpoint.axis[Z] + clearance, settings.max_feedrate[Z]);
  #ifdef Z_SAFE_HOMING_X
    x = (int32_t)(Z_SAFE_HOMING_X * 1000.);
  #else
    x = (lim_lo(X) + lim_hi(X)) / 2 - probe_offset[X];
  #endif
  #ifdef Z_SAFE_HOMING_Y
    y = (int32_t)(Z_SAFE_HOMING_Y * 1000.);
  #else
    y = (lim_lo(Y) + lim_hi(Y)) / 2 - probe_offset[Y];
  #endif
  if (x < lim_lo(X)) x = lim_lo(X);
  if (x > lim_hi(X)) x = lim_hi(X);
  if (y < lim_lo(Y)) y = lim_lo(Y);
  if (y > lim_hi(Y)) y = lim_hi(Y);
  probe_goto(x, y, startpoint.axis[Z], Z_PROBE_XY_FEEDRATE);

  if ( ! probe_down(startpoint.axis[Z] - MAX_DELTA_UM / 2, &trigger)) {
    zcorr_resume();
    return 0;
  }

  // Define the coordinate: at the trigger the nozzle is -offset Z above
  // the bed (plus Z_MIN and the M206 home offset).
  #ifdef Z_MIN
    z_home = (int32_t)(Z_MIN * 1000.);
  #else
    z_home = 0;
  #endif
  z_home += home_offset[Z] - probe_offset[Z];
  startpoint_steps.axis[Z] += um_to_steps(z_home, Z) - trigger;
  zcorr_sync_logical();
  axes_homed |= HOMED_Z;

  probe_raise(z_home + (int32_t)(Z_PROBE_RETRACT * 1000.));
  probe_stow();
  queue_wait();
  zcorr_resume();
  return 1;
}

/**
  Probe at probe position px, py. Correction suspended, XYZ homed.
  \return 1 on success, *bed_z in um.
*/
static uint8_t probe_point(int32_t px, int32_t py, int32_t *bed_z) {
  int32_t nx = px - probe_offset[X], ny = py - probe_offset[Y];
  int32_t trigger, z;

  if (nx < lim_lo(X) || nx > lim_hi(X) || ny < lim_lo(Y) || ny > lim_hi(Y)) {
    sersendf_P(("echo:Probe point X%lq Y%lq unreachable\n"), px, py);
    return 0;
  }
  probe_raise(0);
  probe_goto(nx, ny, startpoint.axis[Z], Z_PROBE_XY_FEEDRATE);
  if ( ! probe_down((int32_t)(Z_PROBE_LOW_POINT * 1000.), &trigger))
    return 0;
  z = steps_to_um(trigger, Z);
  *bed_z = z + probe_offset[Z];
  probe_raise(z + (int32_t)(Z_PROBE_RETRACT * 1000.));
  return 1;
}

/// X, Y and Z homed? Tells the host otherwise.
static uint8_t homed_xyz(void) {
  if ((axes_homed & HOMED_XYZ) != HOMED_XYZ) {
    serial_writestr("echo:Home X, Y and Z first (G28)\n");
    return 0;
  }
  return 1;
}

uint8_t probe_single(int32_t px, int32_t py, int32_t *bed_z) {
  uint8_t ok;

  if ( ! homed_xyz())
    return 0;
  queue_wait();
  zcorr_suspend();
  ok = probe_point(px, py, bed_z);
  if (ok)
    ok = probe_stow();
  queue_wait();
  zcorr_resume();
  if (ok)
    sersendf_P(("Bed X: %lq Y: %lq Z: %lq\n"), px, py, *bed_z);
  return ok;
}

#ifdef Z_STEPPER_ALIGN
/**
  Move Z (mask 1) or Z2 (mask 2) alone up by 'um'. The logical Z stays,
  G34 homes Z again afterwards.
*/
static void z_single_up(uint8_t mask, int32_t um) {
  TARGET t = startpoint;
  int32_t steps_before = startpoint_steps.axis[Z];

  queue_wait();
  z_step_mask = mask;
  t.axis[Z] = startpoint.axis[Z] + um;
  t.F = settings.max_feedrate[Z];
  t.f_multiplier = 256;
  enqueue(&t);
  queue_wait();
  z_step_mask = 3;
  startpoint.axis[Z] -= um;
  startpoint_steps.axis[Z] = steps_before;
}

uint8_t probe_align_z(uint8_t iterations, int32_t accuracy) {
  const int32_t px1 = (int32_t)(Z_STEPPER_ALIGN_X1 * 1000.);
  const int32_t px2 = (int32_t)(Z_STEPPER_ALIGN_X2 * 1000.);
  const int32_t py = (int32_t)(Z_STEPPER_ALIGN_Y * 1000.);
  const float sx1 = (float)(Z_STEPPER_X1 * 1000.);
  const float sx2 = (float)(Z_STEPPER_X2 * 1000.);
  const int32_t max_move = (int32_t)(Z_STEPPER_ALIGN_MAX * 1000.);
  int32_t z1, z2, last_diff = 0x7FFFFFFF;
  uint8_t i, ok = 0;

  if ( ! homed_xyz())
    return 0;
  queue_wait();
  zcorr_suspend();
  for (i = 0; i < iterations; i++) {
    float slope, zs1, zs2;
    int32_t diff, move;

    if ( ! probe_point(px1, py, &z1) || ! probe_point(px2, py, &z2))
      break;
    diff = z2 - z1;
    sersendf_P(("echo:G34 #%su: Z1 %lq Z2 %lq, difference %lq\n"),
               (uint8_t)(i + 1), z1, z2, diff);
    if (labs(diff) <= accuracy) {
      ok = 1;
      break;
    }
    if (labs(diff) >= labs(last_diff)) {
      serial_writestr("echo:G34: not getting better, check Z_STEPPER_X1/X2\n");
      break;
    }
    last_diff = diff;

    /**
      The bed looks higher where the gantry is lower. Heights at the
      lead screws (straight line through the probe points), raise the
      lower side of the gantry by the difference. Only up: no crash.
    */
    slope = (float)diff / (float)(px2 - px1);
    zs1 = (float)z1 + slope * (sx1 - (float)px1);
    zs2 = (float)z1 + slope * (sx2 - (float)px1);
    move = (int32_t)lrintf(zs1 - zs2);
    if (labs(move) > max_move) {
      serial_writestr("echo:G34: difference too large (Z_STEPPER_ALIGN_MAX)\n");
      break;
    }
    probe_raise(0);
    if (move > 0)
      z_single_up(1, move);
    else
      z_single_up(2, -move);
  }
  probe_stow();
  queue_wait();
  zcorr_resume();

  // Z and Z2 moved differently: find Z = 0 again.
  if ( ! probe_home_z())
    return 0;
  if (ok)
    serial_writestr("echo:G34: Z steppers aligned, run G29 again\n");
  else
    serial_writestr("echo:G34: not aligned\n");
  return ok;
}
#endif /* Z_STEPPER_ALIGN */

uint8_t probe_mesh(void) {
  #ifdef BED_LEVELING
    const int32_t inset = (int32_t)(MESH_INSET * 1000.);
    int32_t x0, x1, y0, y1, dx, dy, z;
    uint8_t ix, iy, k;

    if ( ! homed_xyz())
      return 0;

    // Probe positions reachable with the nozzle inside the soft limits.
    x0 = lim_lo(X) + inset;
    if (x0 < lim_lo(X) + probe_offset[X]) x0 = lim_lo(X) + probe_offset[X];
    x1 = lim_hi(X) - inset;
    if (x1 > lim_hi(X) + probe_offset[X]) x1 = lim_hi(X) + probe_offset[X];
    y0 = lim_lo(Y) + inset;
    if (y0 < lim_lo(Y) + probe_offset[Y]) y0 = lim_lo(Y) + probe_offset[Y];
    y1 = lim_hi(Y) - inset;
    if (y1 > lim_hi(Y) + probe_offset[Y]) y1 = lim_hi(Y) + probe_offset[Y];
    if (x1 - x0 < 10000 || y1 - y0 < 10000) {
      serial_writestr("echo:G29: probe area too small, check M851\n");
      return 0;
    }
    dx = (x1 - x0) / (GRID_POINTS_X - 1);
    dy = (y1 - y0) / (GRID_POINTS_Y - 1);

    queue_wait();
    zcorr_suspend();
    bed_level_new(x0, y0, dx, dy, GRID_POINTS_X, GRID_POINTS_Y);
    for (iy = 0; iy < GRID_POINTS_Y; iy++) {
      for (k = 0; k < GRID_POINTS_X; k++) {
        // Serpentine: every other row backwards.
        ix = (iy & 1) ? (uint8_t)(GRID_POINTS_X - 1 - k) : k;
        if ( ! probe_point(x0 + dx * ix, y0 + dy * iy, &z)) {
          bed_level_new(0, 0, 0, 0, 0, 0);      // No mesh.
          probe_stow();
          queue_wait();
          zcorr_resume();
          serial_writestr("echo:G29 aborted, leveling off\n");
          return 0;
        }
        bed_level_set_point(ix, iy, z);
      }
    }
    probe_stow();
    bed_level_set_active(1);
    queue_wait();
    zcorr_resume();
    bed_level_report();
    return 1;
  #else
    serial_writestr("echo:G29 needs BED_LEVELING\n");
    return 0;
  #endif
}

#endif /* Z_PROBE */
