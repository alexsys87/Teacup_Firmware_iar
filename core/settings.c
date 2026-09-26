/** \file
  \brief Runtime settings and their storage, Marlin compatible commands.

  Defaults come from the printer configuration (STEPS_PER_M_X etc.).
  M500 stores the current values in Flash (see hal/flash_store.c), they're
  loaded at startup. M502 restores the defaults, M503 reports.

  Stored: steps per mm, max. feedrates, max. accelerations, accelerations
  for printing, retracts and travel, linear advance K, input shaping and
  S-curve, XY skew (M852), Z backlash (M425), firmware retract (M207,
  M208), host lost timeout (M86), power loss recovery on/off (M413),
  jerk, home offsets, PID values of all heaters and the part fan
  feed-forward (M301 F), stepper idle timeout,
  Z offset (M290), probe offset (M851), filament runout (M412), bed
  leveling mesh and state (G29, M420, M421). The layout is the same with
  or without these features, so a stored record survives option changes.
  Records of the first version (without the new fields) are still read.
*/

#include "settings.h"

#include <string.h>
#include "config_wrapper.h"
#include "dda_maths.h"
#include "dda_kinematics.h"
#include "dda_queue.h"
#include "heater.h"
#include "home.h"
#include "pinio.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "flash_store.h"
#include "babystep.h"
#include "probe.h"
#include "filament.h"
#include "bed_leveling.h"
#include "gcode_parse.h"
#include "input_shaping.h"

/// Increment when the stored layout changes. Old records are ignored then,
/// except versions 1 to 5, which get converted.
#define SETTINGS_VERSION 6

#ifndef MAX_ACCELERATION_X
  #define MAX_ACCELERATION_X ACCELERATION
#endif
#ifndef MAX_ACCELERATION_Y
  #define MAX_ACCELERATION_Y ACCELERATION
#endif
#ifndef MAX_ACCELERATION_Z
  #define MAX_ACCELERATION_Z ACCELERATION
#endif
#ifndef MAX_ACCELERATION_E
  #define MAX_ACCELERATION_E ACCELERATION
#endif
#ifndef ACCELERATION_RETRACT
  #define ACCELERATION_RETRACT ACCELERATION
#endif
#ifndef ACCELERATION_TRAVEL
  #define ACCELERATION_TRAVEL ACCELERATION
#endif
#ifndef XY_SKEW_FACTOR
  #define XY_SKEW_FACTOR 0.0
#endif
#ifndef BACKLASH_Z
  #define BACKLASH_Z 0.0
#endif
#ifndef RETRACT_LENGTH
  #define RETRACT_LENGTH 3.0
#endif
#ifndef RETRACT_FEEDRATE
  #define RETRACT_FEEDRATE 45.0
#endif
#ifndef RETRACT_ZLIFT
  #define RETRACT_ZLIFT 0.0
#endif
#ifndef RETRACT_RECOVER_LENGTH
  #define RETRACT_RECOVER_LENGTH 0.0
#endif
#ifndef RETRACT_RECOVER_FEEDRATE
  #define RETRACT_RECOVER_FEEDRATE 8.0
#endif
#ifndef HOST_TIMEOUT
  #define HOST_TIMEOUT 0
#endif
#ifndef HOST_LOST_HOTEND_TEMP
  #define HOST_LOST_HOTEND_TEMP 0
#endif
#ifndef MAX_JERK_X
  #define MAX_JERK_X 0
  #define MAX_JERK_Y 0
  #define MAX_JERK_Z 0
  #define MAX_JERK_E 0
#endif

settings_t settings;

/// Motion settings as stored in version 1 and 2, don't change.
typedef struct {
  axes_uint32_t steps_per_m;
  axes_uint32_t max_feedrate;
  axes_uint32_t max_accel;
  uint32_t      acceleration;
  axes_uint32_t max_jerk;
} settings_motion_v1_t;

/// Everything stored in Flash, version 1. Word aligned, no pointers.
typedef struct {
  settings_motion_v1_t motion;
  int32_t    home_offset[3];
  uint32_t   stepper_idle_timeout;
  struct {
    int32_t p, i, d, i_limit;
  } pid[NUM_HEATERS];
} settings_store_v1_t;

/// Version 2: version 1 plus Z offset, probe, runout and the mesh.
typedef struct {
  settings_store_v1_t v1;
  int32_t    z_offset;          ///< M290, um.
  int32_t    probe_offset[3];   ///< M851, um.
  int32_t    runout_distance;   ///< M412 D, um.
  uint32_t   runout_enabled;    ///< M412 S.
  mesh_t     mesh;              ///< G29, M420, M421.
} settings_store_v2_t;

/// Version 3: version 2 plus retract and travel acceleration (M204 R T).
typedef struct {
  settings_store_v2_t v2;
  uint32_t   accel_retract;
  uint32_t   accel_travel;
} settings_store_v3_t;

/// Version 4: version 3 plus linear advance (M900 K).
typedef struct {
  settings_store_v3_t v3;
  uint32_t   la_k;
} settings_store_v4_t;

/// Version 5: version 4 plus input shaping and S-curve (M593).
typedef struct {
  settings_store_v4_t v4;
  uint32_t   is_freq[2];
  uint32_t   is_damp[2];
  uint32_t   is_type[2];
  uint32_t   s_curve_us;
} settings_store_v5_t;

/// Version 6: version 5 plus fan feed-forward, skew, backlash, retract,
/// host lost timeout, power loss recovery switch.
typedef struct {
  settings_store_v5_t v5;
  uint32_t   pid_fan_ff;        ///< M301 F, 1/100 PWM counts.
  int32_t    skew_xy;           ///< M852 I, millionths.
  uint32_t   backlash_z;        ///< M425 Z, um.
  uint32_t   backlash_f;        ///< M425 F, 1/1000.
  uint32_t   retract_length;    ///< M207 S, um.
  uint32_t   retract_feedrate;  ///< M207 F, mm/min.
  int32_t    retract_zlift;     ///< M207 Z, um.
  int32_t    recover_extra;     ///< M208 S, um.
  uint32_t   recover_feedrate;  ///< M208 F, mm/min.
  uint32_t   host_timeout;      ///< M86 S, s.
  uint32_t   host_lost_temp;    ///< M86 E, C.
  uint32_t   plr_enabled;       ///< M413 S.
} settings_store_t;

uint32_t settings_axis_accel(enum axis_e axis) {
  uint32_t a = settings.accel_travel;

  if (settings.max_accel[axis] < a)
    a = settings.max_accel[axis];
  return a ? a : 1;
}

void settings_defaults(void) {
  uint8_t i;

  settings.steps_per_m[X] = STEPS_PER_M_X;
  settings.steps_per_m[Y] = STEPS_PER_M_Y;
  settings.steps_per_m[Z] = STEPS_PER_M_Z;
  settings.steps_per_m[E] = STEPS_PER_M_E;
  settings.max_feedrate[X] = MAXIMUM_FEEDRATE_X;
  settings.max_feedrate[Y] = MAXIMUM_FEEDRATE_Y;
  settings.max_feedrate[Z] = MAXIMUM_FEEDRATE_Z;
  settings.max_feedrate[E] = MAXIMUM_FEEDRATE_E;
  settings.max_accel[X] = MAX_ACCELERATION_X;
  settings.max_accel[Y] = MAX_ACCELERATION_Y;
  settings.max_accel[Z] = MAX_ACCELERATION_Z;
  settings.max_accel[E] = MAX_ACCELERATION_E;
  settings.acceleration = ACCELERATION;
  settings.accel_retract = ACCELERATION_RETRACT;
  settings.accel_travel = ACCELERATION_TRAVEL;
  settings.la_k = (uint32_t)(LINEAR_ADVANCE_K * 10000. + 0.5);
  settings.is_freq[0] = (uint32_t)(INPUT_SHAPING_FREQ_X * 100. + 0.5);
  settings.is_freq[1] = (uint32_t)(INPUT_SHAPING_FREQ_Y * 100. + 0.5);
  settings.is_damp[0] = (uint32_t)(INPUT_SHAPING_DAMPING_X * 1000. + 0.5);
  settings.is_damp[1] = (uint32_t)(INPUT_SHAPING_DAMPING_Y * 1000. + 0.5);
  settings.is_type[0] = INPUT_SHAPING_TYPE_X;
  settings.is_type[1] = INPUT_SHAPING_TYPE_Y;
  settings.s_curve_us = (uint32_t)(S_CURVE_TIME * 1000. + 0.5);
  settings.skew_xy = (int32_t)(XY_SKEW_FACTOR * 1000000. +
                               (XY_SKEW_FACTOR < 0 ? -0.5 : 0.5));
  settings.backlash_z = (uint32_t)(BACKLASH_Z * 1000. + 0.5);
  settings.backlash_f = 1000;
  settings.retract_length = (uint32_t)(RETRACT_LENGTH * 1000. + 0.5);
  settings.retract_feedrate = (uint32_t)(RETRACT_FEEDRATE * 60. + 0.5);
  settings.retract_zlift = (int32_t)(RETRACT_ZLIFT * 1000. + 0.5);
  settings.recover_extra = (int32_t)(RETRACT_RECOVER_LENGTH * 1000. +
                                     (RETRACT_RECOVER_LENGTH < 0 ? -0.5 : 0.5));
  settings.recover_feedrate = (uint32_t)(RETRACT_RECOVER_FEEDRATE * 60. + 0.5);
  settings.host_timeout = HOST_TIMEOUT;
  settings.host_lost_temp = HOST_LOST_HOTEND_TEMP;
  settings.plr_enabled = 1;
  settings.max_jerk[X] = MAX_JERK_X;
  settings.max_jerk[Y] = MAX_JERK_Y;
  settings.max_jerk[Z] = MAX_JERK_Z;
  settings.max_jerk[E] = MAX_JERK_E;

  for (i = 0; i < 3; i++)
    home_offset[i] = 0;
  steppers_set_idle_timeout(STEPPER_IDLE_TIMEOUT);
  pid_init();
  #ifdef BABYSTEPPING
    babystep_set_offset(0);
  #endif
  #ifdef Z_PROBE
    probe_defaults();
  #endif
  #ifdef FILAMENT_RUNOUT_PIN
    filament_runout_set(1, (int32_t)(FILAMENT_RUNOUT_DISTANCE * 1000.));
  #endif
  #ifdef BED_LEVELING
    bed_level_defaults();
  #endif
}

void settings_apply(void) {
  dda_maths_update();
  kinematics_update();
  #ifdef INPUT_SHAPING
    shaper_configure();
  #endif
}

/// Everything of version 1 into the store.
static void store_v1(settings_store_v1_t *v1) {
  uint8_t i;

  memcpy(v1->motion.steps_per_m, settings.steps_per_m, sizeof(axes_uint32_t));
  memcpy(v1->motion.max_feedrate, settings.max_feedrate, sizeof(axes_uint32_t));
  memcpy(v1->motion.max_accel, settings.max_accel, sizeof(axes_uint32_t));
  v1->motion.acceleration = settings.acceleration;
  memcpy(v1->motion.max_jerk, settings.max_jerk, sizeof(axes_uint32_t));
  for (i = 0; i < 3; i++)
    v1->home_offset[i] = home_offset[i];
  v1->stepper_idle_timeout = steppers_get_idle_timeout();
  for (i = 0; i < NUM_HEATERS; i++)
    pid_get((heater_t)i, &v1->pid[i].p, &v1->pid[i].i, &v1->pid[i].d,
            &v1->pid[i].i_limit);
}

/// Version 1 part from the store.
static void load_v1(const settings_store_v1_t *v1) {
  uint8_t i;

  memcpy(settings.steps_per_m, v1->motion.steps_per_m, sizeof(axes_uint32_t));
  memcpy(settings.max_feedrate, v1->motion.max_feedrate, sizeof(axes_uint32_t));
  memcpy(settings.max_accel, v1->motion.max_accel, sizeof(axes_uint32_t));
  settings.acceleration = v1->motion.acceleration;
  memcpy(settings.max_jerk, v1->motion.max_jerk, sizeof(axes_uint32_t));
  for (i = 0; i < 3; i++)
    home_offset[i] = v1->home_offset[i];
  steppers_set_idle_timeout((uint16_t)v1->stepper_idle_timeout);
  for (i = 0; i < NUM_HEATERS; i++) {
    pid_set_p((heater_t)i, v1->pid[i].p);
    pid_set_i((heater_t)i, v1->pid[i].i);
    pid_set_d((heater_t)i, v1->pid[i].d);
    pid_set_i_limit((heater_t)i, v1->pid[i].i_limit);
  }
}

uint8_t settings_save(void) {
  static settings_store_t store;
  settings_store_v5_t *v5 = &store.v5;
  settings_store_v2_t *v2 = &store.v5.v4.v3.v2;
  uint8_t a;

  memset(&store, 0, sizeof(store));
  store_v1(&v2->v1);
  #ifdef BABYSTEPPING
    v2->z_offset = babystep_offset();
  #endif
  #ifdef Z_PROBE
    v2->probe_offset[X] = probe_offset[X];
    v2->probe_offset[Y] = probe_offset[Y];
    v2->probe_offset[Z] = probe_offset[Z];
  #endif
  #ifdef FILAMENT_RUNOUT_PIN
    v2->runout_distance = filament_runout_distance();
    v2->runout_enabled = filament_runout_enabled();
  #else
    v2->runout_distance = -1;
    v2->runout_enabled = 1;
  #endif
  #ifdef BED_LEVELING
    v2->mesh = mesh;
  #endif
  v5->v4.v3.accel_retract = settings.accel_retract;
  v5->v4.v3.accel_travel = settings.accel_travel;
  v5->v4.la_k = settings.la_k;
  for (a = 0; a < 2; a++) {
    v5->is_freq[a] = settings.is_freq[a];
    v5->is_damp[a] = settings.is_damp[a];
    v5->is_type[a] = settings.is_type[a];
  }
  v5->s_curve_us = settings.s_curve_us;
  store.pid_fan_ff = pid_get_fan_ff();
  store.skew_xy = settings.skew_xy;
  store.backlash_z = settings.backlash_z;
  store.backlash_f = settings.backlash_f;
  store.retract_length = settings.retract_length;
  store.retract_feedrate = settings.retract_feedrate;
  store.retract_zlift = settings.retract_zlift;
  store.recover_extra = settings.recover_extra;
  store.recover_feedrate = settings.recover_feedrate;
  store.host_timeout = settings.host_timeout;
  store.host_lost_temp = settings.host_lost_temp;
  store.plr_enabled = settings.plr_enabled;

  if ( ! flash_store_write(&store, sizeof(store), SETTINGS_VERSION))
    return 0;

  sersendf_P(("echo:Settings Stored (%u bytes; crc %lu)\n"),
             (uint16_t)sizeof(store), flash_store_crc32(&store, sizeof(store)));
  return 1;
}

/// Version 2 part from the store.
static void load_v2(const settings_store_v2_t *v2) {
  load_v1(&v2->v1);
  #ifdef BABYSTEPPING
    babystep_set_offset(v2->z_offset);
  #endif
  #ifdef Z_PROBE
    probe_offset[X] = v2->probe_offset[X];
    probe_offset[Y] = v2->probe_offset[Y];
    probe_offset[Z] = v2->probe_offset[Z];
  #endif
  #ifdef FILAMENT_RUNOUT_PIN
    if (v2->runout_distance >= 0)
      filament_runout_set((int8_t)(v2->runout_enabled ? 1 : 0),
                          v2->runout_distance);
  #endif
  #ifdef BED_LEVELING
    if (v2->mesh.nx <= 7 && v2->mesh.ny <= 7) {
      mesh = v2->mesh;
      zcorr_sync_logical();
    }
  #endif
}

/// Version 3 part from the store.
static void load_v3(const settings_store_v3_t *v3) {
  load_v2(&v3->v2);
  if (v3->accel_retract)
    settings.accel_retract = v3->accel_retract;
  if (v3->accel_travel)
    settings.accel_travel = v3->accel_travel;
}

/// Report a loaded record of an older layout.
static void report_old(const void *data, uint16_t length) {
  sersendf_P(("echo:Stored settings retrieved (%u bytes, old format; crc %lu)\n"),
             length, flash_store_crc32(data, length));
}

/// Version 4 part from the store.
static void load_v4(const settings_store_v4_t *v4) {
  load_v3(&v4->v3);
  if (v4->la_k <= 100000UL)
    settings.la_k = v4->la_k;
}

/// Version 5 part from the store.
static void load_v5(const settings_store_v5_t *v5) {
  uint8_t a;

  load_v4(&v5->v4);
  for (a = 0; a < 2; a++) {
    if (v5->is_freq[a] <= 50000UL)            // Up to 500 Hz.
      settings.is_freq[a] = v5->is_freq[a];
    if (v5->is_damp[a] < 1000UL)
      settings.is_damp[a] = v5->is_damp[a];
    if (v5->is_type[a] <= 1)
      settings.is_type[a] = v5->is_type[a];
  }
  if (v5->s_curve_us <= 100000UL)
    settings.s_curve_us = v5->s_curve_us;
}

uint8_t settings_load(void) {
  static settings_store_t store;
  settings_store_v5_t *v5 = &store.v5;
  settings_store_v4_t *v4 = &store.v5.v4;
  settings_store_v3_t *v3 = &store.v5.v4.v3;

  if (flash_store_read(&store, sizeof(store), SETTINGS_VERSION)) {
    load_v5(v5);
    if (store.pid_fan_ff <= 25500UL)
      pid_set_fan_ff(store.pid_fan_ff);
    if (store.skew_xy >= -100000L && store.skew_xy <= 100000L)
      settings.skew_xy = store.skew_xy;
    if (store.backlash_z <= 5000UL)
      settings.backlash_z = store.backlash_z;
    if (store.backlash_f <= 1000UL)
      settings.backlash_f = store.backlash_f;
    if (store.retract_length <= 100000UL)
      settings.retract_length = store.retract_length;
    if (store.retract_feedrate >= 60 && store.retract_feedrate <= 60000UL)
      settings.retract_feedrate = store.retract_feedrate;
    if (store.retract_zlift >= 0 && store.retract_zlift <= 10000L)
      settings.retract_zlift = store.retract_zlift;
    if (store.recover_extra >= -100000L && store.recover_extra <= 100000L)
      settings.recover_extra = store.recover_extra;
    if (store.recover_feedrate >= 60 && store.recover_feedrate <= 60000UL)
      settings.recover_feedrate = store.recover_feedrate;
    if (store.host_timeout <= 3600UL)
      settings.host_timeout = store.host_timeout;
    if (store.host_lost_temp <= 300UL)
      settings.host_lost_temp = store.host_lost_temp;
    if (store.plr_enabled <= 1)
      settings.plr_enabled = store.plr_enabled;
    sersendf_P(("echo:Stored settings retrieved (%u bytes; crc %lu)\n"),
               (uint16_t)sizeof(store), flash_store_crc32(&store, sizeof(store)));
    return 1;
  }

  // Older records without the new fields? Take what's there, the new
  // fields keep their values.
  if (flash_store_read(v5, sizeof(*v5), 5)) {
    load_v5(v5);
    report_old(v5, sizeof(*v5));
    return 1;
  }
  if (flash_store_read(v4, sizeof(*v4), 4)) {
    load_v4(v4);
    report_old(v4, sizeof(*v4));
    return 1;
  }
  if (flash_store_read(v3, sizeof(*v3), 3)) {
    load_v3(v3);
    report_old(v3, sizeof(*v3));
    return 1;
  }
  if (flash_store_read(&v3->v2, sizeof(v3->v2), 2)) {
    load_v2(&v3->v2);
    report_old(&v3->v2, sizeof(v3->v2));
    return 1;
  }
  if (flash_store_read(&v3->v2.v1, sizeof(v3->v2.v1), 1)) {
    load_v1(&v3->v2.v1);
    report_old(&v3->v2.v1, sizeof(v3->v2.v1));
    return 1;
  }
  return 0;
}

void settings_init(void) {
  if (flash_store_on_spi())
    serial_writestr("echo:Settings in SPI flash\n");
  settings_defaults();
  if ( ! settings_load())
    serial_writestr("echo:No stored settings, using defaults\n");
  settings_apply();
  // Startup position is logical 0 (not homed), motors wherever the Z
  // correction puts that.
  startpoint.axis[Z] = next_target.target.axis[Z] = 0;
  dda_new_startpoint();
}

/** Print thousandths with two decimals, e.g. 1234 -> "1.23". */
static void write_milli2(int32_t milli) {
  uint32_t v;

  if (milli < 0) {
    serial_writechar('-');
    milli = -milli;
  }
  v = ((uint32_t)milli + 5) / 10;             // hundredths, rounded
  serwrite_uint32(v / 100);
  serial_writechar('.');
  serial_writechar((char)('0' + (v / 10) % 10));
  serial_writechar((char)('0' + v % 10));
}

/// " X<a> Y<b> Z<c> E<d>" with thousandths values.
static void write_axes(const char *cmd, int32_t x, int32_t y, int32_t z,
                       int32_t e, uint8_t with_e) {
  serial_writestr("echo:  ");
  serial_writestr(cmd);
  serial_writestr(" X"); write_milli2(x);
  serial_writestr(" Y"); write_milli2(y);
  serial_writestr(" Z"); write_milli2(z);
  if (with_e) {
    serial_writestr(" E"); write_milli2(e);
  }
  serial_writechar('\n');
}

static void report_pid(const char *cmd, heater_t h) {
  int32_t p, i, d, lim;

  pid_get(h, &p, &i, &d, &lim);
  // Teacup -> Marlin units: Kp = P / 256, Ki = I / 64, Kd = D / 128.
  serial_writestr("echo:  ");
  serial_writestr(cmd);
  serial_writestr(" P"); write_milli2((int32_t)(((int64_t)p * 1000) / 256));
  serial_writestr(" I"); write_milli2((int32_t)(((int64_t)i * 1000) / 64));
  serial_writestr(" D"); write_milli2((int32_t)(((int64_t)d * 1000) / 128));
  #if defined HEATER_EXTRUDER && defined HEATER_FAN
    if (h == HEATER_EXTRUDER) {
      serial_writestr(" F");
      write_milli2((int32_t)pid_get_fan_ff() * 10);
    }
  #endif
  serial_writechar('\n');
}

void settings_report(void) {
  const settings_t *s = &settings;

  serial_writestr("echo:; Steps per unit:\n");
  write_axes("M92", (int32_t)s->steps_per_m[X], (int32_t)s->steps_per_m[Y],
             (int32_t)s->steps_per_m[Z], (int32_t)s->steps_per_m[E], 1);

  // mm/min -> mm/s: * 1000 / 60 for thousandths.
  serial_writestr("echo:; Maximum feedrates (units/s):\n");
  write_axes("M203", (int32_t)(s->max_feedrate[X] * 50 / 3),
             (int32_t)(s->max_feedrate[Y] * 50 / 3),
             (int32_t)(s->max_feedrate[Z] * 50 / 3),
             (int32_t)(s->max_feedrate[E] * 50 / 3), 1);

  serial_writestr("echo:; Maximum Acceleration (units/s2):\n");
  write_axes("M201", (int32_t)s->max_accel[X] * 1000,
             (int32_t)s->max_accel[Y] * 1000, (int32_t)s->max_accel[Z] * 1000,
             (int32_t)s->max_accel[E] * 1000, 1);

  serial_writestr("echo:; Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>\n");
  serial_writestr("echo:  M204 P"); write_milli2((int32_t)s->acceleration * 1000);
  serial_writestr(" R"); write_milli2((int32_t)s->accel_retract * 1000);
  serial_writestr(" T"); write_milli2((int32_t)s->accel_travel * 1000);
  serial_writechar('\n');

  #ifdef LINEAR_ADVANCE
    serial_writestr("echo:; Linear Advance:\n");
    serial_writestr("echo:  M900 K");
    serwrite_uint32(s->la_k / 10000);
    serial_writechar('.');
    serial_writechar((char)('0' + (s->la_k / 1000) % 10));
    serial_writechar((char)('0' + (s->la_k / 100) % 10));
    serial_writechar((char)('0' + (s->la_k / 10) % 10));
    serial_writechar((char)('0' + s->la_k % 10));
    serial_writechar('\n');
  #endif

  #ifdef INPUT_SHAPING
  {
    uint8_t a;

    serial_writestr("echo:; Input Shaping: F<Hz> D<zeta> T<0 = ZV, 1 = MZV>, S-curve S<ms>\n");
    for (a = 0; a < 2; a++) {
      serial_writestr(a ? "echo:  M593 Y F" : "echo:  M593 X F");
      write_milli2((int32_t)s->is_freq[a] * 10);
      serial_writestr(" D0.");
      serial_writechar((char)('0' + (s->is_damp[a] / 100) % 10));
      serial_writechar((char)('0' + (s->is_damp[a] / 10) % 10));
      serial_writechar((char)('0' + s->is_damp[a] % 10));
      serial_writestr(" T");
      serwrite_uint32(s->is_type[a]);
      serial_writechar('\n');
    }
    serial_writestr("echo:  M593 S");
    write_milli2((int32_t)s->s_curve_us);
    if (shaper_overflows) {
      serial_writestr(" ; history overflows: ");
      serwrite_uint32(shaper_overflows);
    }
    serial_writechar('\n');
  }
  #endif

  #ifdef SKEW_CORRECTION
    serial_writestr("echo:; Skew factor XY (tangent):\n");
    serial_writestr("echo:  M852 I");
    {
      int32_t k = s->skew_xy;
      uint32_t a;
      uint8_t n;

      if (k < 0) {
        serial_writechar('-');
        k = -k;
      }
      a = (uint32_t)k;
      serwrite_uint32(a / 1000000UL);
      serial_writechar('.');
      for (n = 0, a %= 1000000UL; n < 6; n++) {
        a *= 10;
        serial_writechar((char)('0' + a / 1000000UL));
        a %= 1000000UL;
      }
    }
    serial_writechar('\n');
  #endif
  #ifdef POWER_LOSS_RECOVERY
    serial_writestr("echo:; Power-Loss Recovery:\n");
    sersendf_P(("echo:  M413 S%lu\n"), s->plr_enabled);
  #endif
  #ifdef HOST_WATCH
    serial_writestr("echo:; Host lost: S<timeout s> E<hotend C>\n");
    sersendf_P(("echo:  M86 S%lu E%lu\n"), s->host_timeout, s->host_lost_temp);
  #endif
  #ifdef FIRMWARE_RETRACT
    serial_writestr("echo:; Retract: S<length> F<units/m> Z<lift>\n");
    serial_writestr("echo:  M207 S");
    write_milli2((int32_t)s->retract_length);
    serial_writestr(" F");
    write_milli2((int32_t)s->retract_feedrate * 1000);
    serial_writestr(" Z");
    write_milli2(s->retract_zlift);
    serial_writestr("\necho:; Recover: S<length> F<units/m>\n");
    serial_writestr("echo:  M208 S");
    write_milli2(s->recover_extra);
    serial_writestr(" F");
    write_milli2((int32_t)s->recover_feedrate * 1000);
    serial_writechar('\n');
  #endif
  #ifdef BACKLASH_COMPENSATION
    serial_writestr("echo:; Backlash compensation: F<fraction> Z<mm>\n");
    serial_writestr("echo:  M425 F");
    write_milli2((int32_t)s->backlash_f);
    serial_writestr(" Z");
    write_milli2((int32_t)s->backlash_z);
    serial_writechar('\n');
  #endif

  serial_writestr("echo:; Advanced: X<max_x_jerk> Y<max_y_jerk> Z<max_z_jerk> E<max_e_jerk>\n");
  write_axes("M205", (int32_t)(s->max_jerk[X] * 50 / 3),
             (int32_t)(s->max_jerk[Y] * 50 / 3), (int32_t)(s->max_jerk[Z] * 50 / 3),
             (int32_t)(s->max_jerk[E] * 50 / 3), 1);

  serial_writestr("echo:; Home offset:\n");
  write_axes("M206", home_offset[X], home_offset[Y], home_offset[Z], 0, 0);

  #ifdef HEATER_EXTRUDER
    serial_writestr("echo:; Hotend PID:\n");
    report_pid("M301", HEATER_EXTRUDER);
  #endif
  #ifdef HEATER_BED
    serial_writestr("echo:; Bed PID:\n");
    report_pid("M304", HEATER_BED);
  #endif

  serial_writestr("echo:; Stepper idle timeout:\n");
  sersendf_P(("echo:  M84 S%u\n"), steppers_get_idle_timeout());
  #ifdef BABYSTEPPING
    serial_writestr("echo:; Z offset (M290 total, mm): ");
    write_milli2(babystep_offset());
    serial_writechar('\n');
  #endif
  #ifdef Z_PROBE
    serial_writestr("echo:; Z-Probe Offset (mm):\n");
    write_axes("M851", probe_offset[X], probe_offset[Y], probe_offset[Z], 0, 0);
  #endif
  #ifdef FILAMENT_RUNOUT_PIN
    serial_writestr("echo:; Filament runout sensor:\n");
    sersendf_P(("echo:  M412 S%su D"), filament_runout_enabled());
    write_milli2(filament_runout_distance());
    serial_writechar('\n');
  #endif
  #ifdef BED_LEVELING
    serial_writestr("echo:; Bed leveling:\n");
    sersendf_P(("echo:  M420 S%su Z"), mesh.active);
    write_milli2(mesh.fade);
    serial_writechar('\n');
    if (mesh.nx) {
      uint8_t ix, iy;

      sersendf_P(("echo:; Mesh %sux%su, X%lq Y%lq, spacing X%lq Y%lq:\n"),
                 mesh.nx, mesh.ny, mesh.x0, mesh.y0, mesh.dx, mesh.dy);
      for (iy = 0; iy < mesh.ny; iy++)
        for (ix = 0; ix < mesh.nx; ix++)
          sersendf_P(("echo:  M421 I%su J%su Z%lq\n"), ix, iy,
                     (int32_t)mesh.z[iy][ix]);
    }
  #endif
}
