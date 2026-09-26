/** \file
  \brief Runtime settings and their storage, Marlin compatible commands.

  Defaults come from the printer configuration (STEPS_PER_M_X etc.).
  M500 stores the current values in Flash (see hal/flash_store.c), they're
  loaded at startup. M502 restores the defaults, M503 reports.

  Stored: steps per mm, max. feedrates, max. accelerations, accelerations
  for printing, retracts and travel,
  jerk, home offsets, PID values of all heaters, stepper idle timeout,
  Z offset (M290), probe offset (M851), filament runout (M412), bed
  leveling mesh and state (G29, M420, M421). The layout is the same with
  or without these features, so a stored record survives option changes.
  Records of the first version (without the new fields) are still read.
*/

#include "settings.h"

#include <string.h>
#include "config_wrapper.h"
#include "dda_maths.h"
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

/// Increment when the stored layout changes. Old records are ignored then,
/// except versions 1 and 2, which get converted.
#define SETTINGS_VERSION 3

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
  settings_store_v2_t *v2 = &store.v2;

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
  store.accel_retract = settings.accel_retract;
  store.accel_travel = settings.accel_travel;

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

uint8_t settings_load(void) {
  static settings_store_t store;

  if (flash_store_read(&store, sizeof(store), SETTINGS_VERSION)) {
    load_v2(&store.v2);
    if (store.accel_retract)
      settings.accel_retract = store.accel_retract;
    if (store.accel_travel)
      settings.accel_travel = store.accel_travel;
    sersendf_P(("echo:Stored settings retrieved (%u bytes; crc %lu)\n"),
               (uint16_t)sizeof(store), flash_store_crc32(&store, sizeof(store)));
    return 1;
  }

  // Older records without the new fields? Take what's there, the new
  // fields keep their values.
  if (flash_store_read(&store.v2, sizeof(store.v2), 2)) {
    load_v2(&store.v2);
    sersendf_P(("echo:Stored settings retrieved (%u bytes, old format; crc %lu)\n"),
               (uint16_t)sizeof(store.v2),
               flash_store_crc32(&store.v2, sizeof(store.v2)));
    return 1;
  }
  if (flash_store_read(&store.v2.v1, sizeof(store.v2.v1), 1)) {
    load_v1(&store.v2.v1);
    sersendf_P(("echo:Stored settings retrieved (%u bytes, old format; crc %lu)\n"),
               (uint16_t)sizeof(store.v2.v1),
               flash_store_crc32(&store.v2.v1, sizeof(store.v2.v1)));
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
