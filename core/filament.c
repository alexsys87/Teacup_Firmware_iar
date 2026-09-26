/** \file
  \brief Filament runout sensor (M412) and filament change (M600).

  Runout: the sensor is debounced every 10 ms. When it reports "no
  filament", the E steps fed from then on are counted (dda_start()); after
  FILAMENT_RUNOUT_DISTANCE mm (the filament between sensor and nozzle,
  M412 D) the main loop runs a filament change. Without extrusion there's
  no pause, an idle printer without filament stays quiet.

  Filament change, like Marlin's M600:
    1. wait for the queued moves, remember the position (X, Y, Z, E, F),
    2. retract, lift Z, park at X/Y (only if X and Y are homed),
    3. unload,
    4. wait for the user: "echo:busy: paused for user" every 2 s, host
       prompt (OctoPrint), continue with M108 or M876 S0. Both work over
       the emergency parser, also while the command queue is full. After
       FILAMENT_CHANGE_NOZZLE_TIMEOUT seconds the hotend heater goes off,
       it's heated up again before loading,
    5. load and purge, retract,
    6. back to X/Y, down to Z, prime (undo the retraction).
  All E moves are relative moves outside the G-code coordinate system, the
  logical position afterwards is exactly the one before, E included.
  Without a hot hotend (below FILAMENT_CHANGE_MIN_TEMP) E doesn't move.
*/

#include "filament.h"

#include "pinio.h"
#include "dda.h"
#include "dda_queue.h"
#include "dda_maths.h"
#include "clock.h"
#include "serial.h"
#include "sersendf.h"
#include "temp.h"
#include "heater.h"
#include "home.h"
#include "settings.h"
#include "gcode_queue.h"
#include "beeper.h"

static uint8_t change_active = 0;

uint8_t filament_change_active(void) {
  return change_active;
}

void filament_change_defaults(filament_change_t *p) {
  p->retract = (int32_t)(FILAMENT_CHANGE_RETRACT * 1000.);
  p->lift    = (int32_t)(FILAMENT_CHANGE_Z_LIFT * 1000.);
  p->park_x  = (int32_t)(FILAMENT_CHANGE_PARK_X * 1000.);
  p->park_y  = (int32_t)(FILAMENT_CHANGE_PARK_Y * 1000.);
  p->unload  = (int32_t)(FILAMENT_CHANGE_UNLOAD * 1000.);
  p->load    = (int32_t)(FILAMENT_CHANGE_LOAD * 1000.);
}

/// Queue a move, waiting for queue space.
static void fc_move(TARGET *t) {
  while (queue_full())
    clock_poll();
  enqueue(t);
}

/// Relative E move, 'de' um at 'feed' mm/min. X, Y, Z stay.
static void fc_e(TARGET *t, int32_t de, uint32_t feed) {
  t->axis[E] = de;
  t->F = feed;
  fc_move(t);
  t->axis[E] = 0;
}

/// Move to X, Y, Z (logical, um), no E.
static void fc_xyz(TARGET *t, int32_t x, int32_t y, int32_t z, uint32_t feed) {
  t->axis[X] = x;
  t->axis[Y] = y;
  t->axis[Z] = z;
  t->F = feed;
  fc_move(t);
}

/// Host prompt (OctoPrint "Action Command Prompt"), one "Continue" button.
static void prompt(const char *text) {
  serial_writestr("//action:prompt_end\n//action:prompt_begin ");
  serial_writestr(text);
  serial_writestr("\n//action:prompt_button Continue\n//action:prompt_show\n");
}

/// Wait for M108 / M876 S0. Switches the hotend off after the timeout.
/// \return 1 if the hotend heater was switched off.
static uint8_t wait_for_user(uint8_t may_time_out) {
  uint32_t since = clock_millis();
  uint8_t heater_off = 0;

  gcode_queue_set_paused(1);
  (void)temp_m108_seen();                     // Forget older ones.
  while ( ! temp_m108_seen()) {
    clock_poll();
    #if defined HEATER_EXTRUDER && FILAMENT_CHANGE_NOZZLE_TIMEOUT > 0
      if (may_time_out && ! heater_off &&
          clock_millis() - since >= FILAMENT_CHANGE_NOZZLE_TIMEOUT * 1000UL) {
        temp_set(TEMP_SENSOR_extruder, 0);
        serial_writestr("echo:Nozzle timed out, heater off\n");
        heater_off = 1;
      }
    #else
      (void)may_time_out;
      (void)since;
    #endif
  }
  gcode_queue_set_paused(0);
  serial_writestr("//action:prompt_end\n");
  return heater_off;
}

void filament_change(const filament_change_t *p) {
  TARGET saved, t;
  uint8_t can_e = 1, parked = 0;
  int32_t z_up;
  #ifdef HEATER_EXTRUDER
    uint16_t saved_temp;
  #endif

  if (change_active)
    return;
  change_active = 1;

  queue_wait();
  saved = startpoint;
  t = startpoint;
  t.e_relative = 1;
  t.axis[E] = 0;
  t.e_multiplier = 256;
  t.f_multiplier = 256;

  #ifdef HEATER_EXTRUDER
    saved_temp = temp_get_target(TEMP_SENSOR_extruder);
    can_e = temp_get(TEMP_SENSOR_extruder) >= FILAMENT_CHANGE_MIN_TEMP * 4;
    if ( ! can_e)
      serial_writestr("echo:Hotend too cold, filament not moved\n");
  #endif

  // 1. Retract, lift, park.
  if (can_e && p->retract > 0)
    fc_e(&t, -p->retract, FILAMENT_CHANGE_RETRACT_FEEDRATE);
  z_up = saved.axis[Z] + (p->lift > 0 ? p->lift : 0);
  #ifdef Z_MAX
    if ((axes_homed & HOMED_Z) &&
        z_up > (int32_t)(Z_MAX * 1000.) + home_offset[Z])
      z_up = (int32_t)(Z_MAX * 1000.) + home_offset[Z];
  #endif
  if (z_up > saved.axis[Z])
    fc_xyz(&t, saved.axis[X], saved.axis[Y], z_up, settings.max_feedrate[Z]);
  if ((axes_homed & (HOMED_X | HOMED_Y)) == (HOMED_X | HOMED_Y)) {
    fc_xyz(&t, p->park_x, p->park_y, z_up, FILAMENT_CHANGE_XY_FEEDRATE);
    parked = 1;
  }
  else {
    serial_writestr("echo:X/Y not homed, not parking\n");
  }

  // 2. Unload.
  if (can_e && p->unload > 0)
    fc_e(&t, -p->unload, FILAMENT_CHANGE_UNLOAD_FEEDRATE);
  queue_wait();

  // 3. Wait for the user.
  serial_writestr("echo:Insert filament and send M108\n");
  prompt("Insert filament");
  beeper_tone(2000, 100);
  #ifdef HEATER_EXTRUDER
    if (wait_for_user(can_e)) {
      serial_writestr("echo:Heating nozzle\n");
      temp_set(TEMP_SENSOR_extruder, saved_temp);
      temp_wait_sensor(TEMP_SENSOR_extruder, 1);
      can_e = temp_get(TEMP_SENSOR_extruder) >= FILAMENT_CHANGE_MIN_TEMP * 4;
    }
  #else
    wait_for_user(0);
  #endif

  // 4. Load, purge, retract against oozing on the way back.
  if (can_e && p->load > 0)
    fc_e(&t, p->load, FILAMENT_CHANGE_LOAD_FEEDRATE);
  if (can_e && p->retract > 0)
    fc_e(&t, -p->retract, FILAMENT_CHANGE_RETRACT_FEEDRATE);

  // 5. Back, down, prime.
  if (parked)
    fc_xyz(&t, saved.axis[X], saved.axis[Y], z_up, FILAMENT_CHANGE_XY_FEEDRATE);
  fc_xyz(&t, saved.axis[X], saved.axis[Y], saved.axis[Z],
         settings.max_feedrate[Z]);
  if (can_e && p->retract > 0)
    fc_e(&t, p->retract, FILAMENT_CHANGE_RETRACT_FEEDRATE);
  queue_wait();

  // Same logical position as before. X, Y, Z went back to the same
  // coordinates, so the motor positions (startpoint_steps) match, too; the
  // relative E moves didn't change the absolute E count.
  startpoint = saved;
  serial_writestr("echo:Resuming\n");
  change_active = 0;
}

#ifdef FILAMENT_RUNOUT_PIN

volatile uint32_t filament_e_steps = 0;

static uint8_t runout_enabled = 1;
static int32_t runout_distance = (int32_t)(FILAMENT_RUNOUT_DISTANCE * 1000.);
static uint8_t sensor_runout = 0;       ///< Debounced: 1 = no filament.
static uint8_t debounce = 0;
static uint8_t runout_seen = 0;         ///< Counting E since the trigger.
static uint32_t runout_start = 0;       ///< filament_e_steps at the trigger.
static uint8_t runout_pending = 0;

void filament_init(void) {
  SET_INPUT(FILAMENT_RUNOUT_PIN);
  #ifndef FILAMENT_RUNOUT_NO_PULLUP
    PULLUP_ON(FILAMENT_RUNOUT_PIN);
  #endif
}

uint8_t filament_sensor_runout(void) {
  return sensor_runout;
}

uint8_t filament_runout_enabled(void) {
  return runout_enabled;
}

int32_t filament_runout_distance(void) {
  return runout_distance;
}

void filament_runout_set(int8_t enable, int32_t distance_um) {
  if (enable >= 0)
    runout_enabled = enable ? 1 : 0;
  if (distance_um >= 0)
    runout_distance = distance_um;
  runout_seen = 0;
  if ( ! runout_enabled)
    runout_pending = 0;
}

void filament_runout_reset(void) {
  runout_seen = 0;
  runout_pending = 0;
}

void filament_runout_report(void) {
  serial_writestr(runout_enabled ? "echo:Filament runout ON" :
                                   "echo:Filament runout OFF");
  sersendf_P((", distance %lq mm, sensor: "), runout_distance);
  serial_writestr(sensor_runout ? "no filament\n" : "filament present\n");
}

void filament_tick(void) {
  uint8_t raw = (READ(FILAMENT_RUNOUT_PIN) == FILAMENT_RUNOUT_STATE);

  if (raw != sensor_runout) {
    if (++debounce >= FILAMENT_RUNOUT_DEBOUNCE) {
      sensor_runout = raw;
      debounce = 0;
    }
  }
  else {
    debounce = 0;
  }

  if ( ! sensor_runout) {
    runout_seen = 0;
    return;
  }
  if ( ! runout_enabled || change_active || runout_pending)
    return;

  if ( ! runout_seen) {
    runout_seen = 1;
    runout_start = filament_e_steps;
  }
  else {
    uint32_t need = (uint32_t)muldiv(runout_distance,
                                     settings.steps_per_m[E], 1000000UL);

    if (need < 1)
      need = 1;                   // Some extrusion at least: printing.
    if (filament_e_steps - runout_start >= need)
      runout_pending = 1;
  }
}

void filament_runout_service(void) {
  filament_change_t p;

  if ( ! runout_pending)
    return;
  runout_pending = 0;
  serial_writestr("echo:Filament runout\n//action:out_of_filament T0\n");
  filament_change_defaults(&p);
  filament_change(&p);
  runout_seen = 0;
}

#endif /* FILAMENT_RUNOUT_PIN */
