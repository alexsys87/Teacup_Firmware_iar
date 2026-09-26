/** \file
  \brief Reaction to a lost host, see host_watch.h.

  Printing from a host (OctoPrint, Pronterface, a slicer), the printer
  gets its moves line by line. If the host crashes, sleeps or the USB
  cable comes loose, the printer would stand with the hot nozzle in the
  print: it oozes, burns a hole into the part and the filament in the
  heat break cooks.

  Armed by a move with E from the host while the hotend has a target
  temperature. Lost when, while armed,

   - no line came for M86 S seconds while the printer waited for one
     (no command executing, command and movement queue empty; a long
     M109, G4 or M600 doesn't count), or
   - the USB port the lines came from was closed or unplugged, for 2 s.

  Then, after the last queued moves: retract, lift Z, park X/Y (if homed),
  hotend to M86 E (0 = off). The bed keeps its temperature. The print
  can't be continued; the thermal protection stays active as always.
*/

#include "host_watch.h"

#ifdef HOST_WATCH

#include "dda.h"
#include "dda_queue.h"
#include "clock.h"
#include "serial.h"
#include "sersendf.h"
#include "settings.h"
#include "temp.h"
#include "home.h"
#include "sd.h"
#include "gcode_parse.h"
#include "gcode_queue.h"
#include "filament.h"
#ifdef USB_CDC
  #include "usb_cdc.h"
#endif

#ifndef HOST_LOST_RETRACT
  #define HOST_LOST_RETRACT        2.0
#endif
#ifndef HOST_LOST_Z_LIFT
  #define HOST_LOST_Z_LIFT         10.0
#endif
#ifndef HOST_LOST_PARK_X
  #ifdef FILAMENT_CHANGE_PARK_X
    #define HOST_LOST_PARK_X       FILAMENT_CHANGE_PARK_X
    #define HOST_LOST_PARK_Y       FILAMENT_CHANGE_PARK_Y
  #else
    #define HOST_LOST_PARK_X       X_MIN
    #define HOST_LOST_PARK_Y       Y_MAX
  #endif
#endif
#ifndef HOST_LOST_XY_FEEDRATE
  #define HOST_LOST_XY_FEEDRATE    3000
#endif
#ifndef HOST_LOST_MIN_TEMP
  #define HOST_LOST_MIN_TEMP       180
#endif

static uint8_t armed;
static uint8_t host_port;
static uint32_t idle_since;               ///< clock_millis() of the last activity.
static uint32_t usb_gone_since;
static uint8_t usb_gone;

void host_watch_line(uint8_t port) {
  host_port = port;
  idle_since = clock_millis();
}

void host_watch_arm(void) {
  #ifdef HEATER_EXTRUDER
    if (temp_get_target(TEMP_SENSOR_extruder) == 0)
      return;
  #endif
  if ( ! armed)
    idle_since = clock_millis();
  armed = 1;
  usb_gone = 0;
}

/// Movement queue empty, last move done.
static uint8_t queue_empty(void) {
  return queue_free() == MOVEBUFFER_SIZE - 1 && mb_tail_dda == NULL;
}

/// Queue a move, waiting for queue space.
static void move(TARGET *t) {
  while (queue_full())
    clock_poll();
  enqueue(t);
}

static void host_lost(void) {
  TARGET saved = startpoint, t = startpoint;
  int32_t z;

  armed = 0;
  serial_writestr("echo:Host lost, parking\n");

  t.e_relative = 1;
  t.axis[E] = 0;
  t.e_multiplier = 256;
  t.f_multiplier = 256;

  #ifdef HEATER_EXTRUDER
    if (HOST_LOST_RETRACT > 0 &&
        temp_get(TEMP_SENSOR_extruder) >= HOST_LOST_MIN_TEMP * 4) {
      t.axis[E] = -(int32_t)(HOST_LOST_RETRACT * 1000.);
      t.F = settings.max_feedrate[E];
      move(&t);
      t.axis[E] = 0;
    }
    temp_set(TEMP_SENSOR_extruder, (uint16_t)(settings.host_lost_temp * 4));
  #endif

  z = saved.axis[Z] + (int32_t)(HOST_LOST_Z_LIFT * 1000.);
  #ifdef Z_MAX
    if ((axes_homed & HOMED_Z) && z > (int32_t)(Z_MAX * 1000.) + home_offset[Z])
      z = (int32_t)(Z_MAX * 1000.) + home_offset[Z];
  #endif
  if (z > saved.axis[Z]) {
    t.axis[Z] = z;
    t.F = settings.max_feedrate[Z];
    move(&t);
  }
  if ((axes_homed & (HOMED_X | HOMED_Y)) == (HOMED_X | HOMED_Y)) {
    t.axis[X] = (int32_t)(HOST_LOST_PARK_X * 1000.);
    t.axis[Y] = (int32_t)(HOST_LOST_PARK_Y * 1000.);
    t.F = HOST_LOST_XY_FEEDRATE;
    move(&t);
  }
  queue_wait();

  // Keep the absolute E of the G-code, the retract was relative. The
  // G-code target follows the parked position.
  startpoint.axis[E] = saved.axis[E];
  startpoint.e_relative = saved.e_relative;
  next_target.target.axis[X] = startpoint.axis[X];
  next_target.target.axis[Y] = startpoint.axis[Y];
  next_target.target.axis[Z] = startpoint.axis[Z];
  serial_writestr("echo:Parked, hotend temperature lowered\n");
}

void host_watch_poll(void) {
  uint32_t now = clock_millis();

  if ( ! armed)
    return;

  #ifdef HEATER_EXTRUDER
    // End of the print (M104 S0 in the end G-code), or printing from SD.
    if (temp_get_target(TEMP_SENSOR_extruder) == 0) {
      armed = 0;
      return;
    }
  #endif
  #ifdef SD
    if (gcode_sources & GCODE_SOURCE_SD) {
      armed = 0;
      return;
    }
  #endif

  // Waiting for a line only when nothing is queued any more.
  if ( ! gcode_queue_idle() || ! queue_empty() || filament_change_active())
    idle_since = now;

  #ifdef USB_CDC
    if (host_port == SERIAL_USB_PORT && ! usb_cdc_connected()) {
      if ( ! usb_gone) {
        usb_gone = 1;
        usb_gone_since = now;
      }
      else if (now - usb_gone_since >= 2000 && queue_empty() &&
               gcode_queue_idle()) {
        serial_writestr("echo:USB host disconnected\n");
        host_lost();
        return;
      }
    }
    else
      usb_gone = 0;
  #endif

  if (settings.host_timeout &&
      now - idle_since >= settings.host_timeout * 1000UL) {
    sersendf_P(("echo:No line from the host for %lu s\n"),
               settings.host_timeout);
    host_lost();
  }
}

#endif /* HOST_WATCH */
