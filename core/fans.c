/** \file
  \brief Hotend auto fan and controller fan, see fans.h.

  Hotend fan (Marlin's E0_AUTO_FAN): on while the hotend is at or above
  HOTEND_FAN_TEMP (50 C), also after the heater was switched off, until
  the hotend cooled down. Without the fan heat creeps up the heat break
  and the filament jams there.

  Controller fan (Marlin's CONTROLLER_FAN): on while the stepper drivers
  are enabled, the printer moves or a heater is on, and CONTROLLER_FAN_IDLE
  seconds (60) afterwards.

  Outputs: an MCU pin (HOTEND_FAN_PIN, CONTROLLER_FAN_PIN, active high,
  *_FAN_INVERT for active low) or a PCF8574 output (HOTEND_FAN_EXPANDER_BIT,
  CONTROLLER_FAN_EXPANDER_BIT, PCF8574_ADDRESS). A PCF8574 output is a weak
  pull-up when high: drive a MOSFET module with an extra pull-up (10 k to
  5 V) at its input, "high = on". After power up the PCF8574 outputs are
  high, both fans run until the first update (a few ms).
*/

#include "fans.h"

#include "temp.h"
#include "heater.h"
#include "pinio.h"
#include "clock.h"
#include "dda_queue.h"


#ifdef FANS

#ifndef HOTEND_FAN_TEMP
  #define HOTEND_FAN_TEMP       50
#endif
#ifndef CONTROLLER_FAN_IDLE
  #define CONTROLLER_FAN_IDLE   60
#endif

static uint8_t state;
static uint16_t controller_idle;          ///< 250 ms ticks since activity.

static void set_hotend_fan(uint8_t on) {
  #if defined HOTEND_FAN_PIN
    #ifdef HOTEND_FAN_INVERT
      WRITE(HOTEND_FAN_PIN, on ? 0 : 1);
    #else
      WRITE(HOTEND_FAN_PIN, on ? 1 : 0);
    #endif
  #elif defined HOTEND_FAN_EXPANDER_BIT
    expander_set(HOTEND_FAN_EXPANDER_BIT, on);
  #endif
  state = on ? (state | 1) : (state & ~1);
}

static void set_controller_fan(uint8_t on) {
  #if defined CONTROLLER_FAN_PIN
    #ifdef CONTROLLER_FAN_INVERT
      WRITE(CONTROLLER_FAN_PIN, on ? 0 : 1);
    #else
      WRITE(CONTROLLER_FAN_PIN, on ? 1 : 0);
    #endif
  #elif defined CONTROLLER_FAN_EXPANDER_BIT
    expander_set(CONTROLLER_FAN_EXPANDER_BIT, on);
  #endif
  state = on ? (state | 2) : (state & ~2);
}

void fans_init(void) {
  #ifdef HOTEND_FAN_PIN
    SET_OUTPUT(HOTEND_FAN_PIN);
  #endif
  #ifdef CONTROLLER_FAN_PIN
    SET_OUTPUT(CONTROLLER_FAN_PIN);
  #endif
  set_hotend_fan(0);
  set_controller_fan(0);
  controller_idle = 0xFFFF;
}

void fans_tick(void) {
  #if (defined HOTEND_FAN_PIN || defined HOTEND_FAN_EXPANDER_BIT) && \
      defined HEATER_EXTRUDER
    uint16_t t = temp_get(TEMP_SENSOR_extruder);

    // 1 C hysteresis, no flicker at the threshold.
    if (t >= HOTEND_FAN_TEMP * 4)
      set_hotend_fan(1);
    else if (t < (HOTEND_FAN_TEMP - 1) * 4)
      set_hotend_fan(0);
  #endif

  #if defined CONTROLLER_FAN_PIN || defined CONTROLLER_FAN_EXPANDER_BIT
    if (steppers_enabled() || mb_tail_dda != NULL || ! heaters_all_zero())
      controller_idle = 0;
    else if (controller_idle < 0xFFFF)
      controller_idle++;
    set_controller_fan(controller_idle <= CONTROLLER_FAN_IDLE * 4);
  #endif
}

void fans_emergency(void) {
  #ifdef HOTEND_FAN_PIN
    set_hotend_fan(1);
  #endif
}

uint8_t fans_state(void) {
  return state;
}

#endif /* FANS */
