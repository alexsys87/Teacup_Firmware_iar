/** \file
  \brief Thermal protection, modeled after Marlin.

  All temperatures are in Teacup's 14.2 fixed point (4 units per degree).
  thermal_protection_check() runs every 250 ms for each sensor with a heater.

  1. MAXTEMP: reading above HEATER_MAXTEMP / BED_MAXTEMP, always checked,
     also with the heater off. Catches shorted thermistors and stuck MOSFETs.

  2. MINTEMP: reading below HEATER_MINTEMP / BED_MINTEMP while the heater is
     on. Catches open (broken, unplugged) thermistors.

  3. Sensor timeout: no valid reading for TEMP_SENSOR_TIMEOUT seconds while
     the heater is on (e.g. MAX6675 with open thermocouple).

  4. Heating failed (Marlin's WATCH_TEMP): after setting a target, the
     temperature has to rise by WATCH_TEMP_INCREASE within WATCH_TEMP_PERIOD,
     again and again, until it is close to the target. Close to the target it
     has to reach target - hysteresis within THERMAL_PROTECTION_PERIOD.
     Catches a heater cartridge or thermistor fallen out of the block.

  5. Thermal runaway: once the target was reached, the temperature must not
     stay more than THERMAL_PROTECTION_HYSTERESIS below the target for longer
     than THERMAL_PROTECTION_PERIOD. Catches failures during printing.

  6. Heater stuck on (not in Marlin): while the heater output is off, the
     temperature must not rise by THERMAL_PROTECTION_OFF_RISE within
     THERMAL_PROTECTION_OFF_PERIOD seconds (_BED_ variants for the bed).
     The first period after switching off is not judged, the temperature
     still overshoots then. Catches a shorted MOSFET or a welded relay long
     before MAXTEMP, also when holding a target (the PID output drops to 0
     when the temperature climbs above the target). For the bed, MAXTEMP
     may never come: a bed at full power often stays below it. Only
     printer_kill() switching the power supply off (PS_ON_PIN) really stops
     such a heater, so use one.

  Three consecutive bad readings (TEMP_ERROR_READINGS) are required for
  MINTEMP/MAXTEMP, to ignore single noisy readings.
*/

#include "thermal_protection.h"
#include "config_wrapper.h"
#include "temp.h"
#include "kill.h"
#include "serial.h"

/// Ticks per second, thermal_protection_check() runs every 250 ms.
#define TP_TICKS_PER_S  4
/// Degree Celsius to 14.2 fixed point.
#define QC(c)           ((uint16_t)((c) * 4))

typedef struct {
  uint16_t period;        ///< Runaway period, ticks.
  uint16_t hysteresis;    ///< Runaway hysteresis, qC.
  uint16_t watch_period;  ///< Heating watch period, ticks.
  uint16_t watch_inc;     ///< Required increase per watch period, qC.
  uint16_t maxtemp;       ///< qC.
  uint16_t mintemp;       ///< qC.
  uint16_t max_target;    ///< Highest allowed target, qC.
  uint16_t off_period;    ///< Heater stuck on: period, ticks.
  uint16_t off_rise;      ///< Heater stuck on: allowed rise, qC. 0 = off.
} tp_param_t;

enum {
  TP_INACTIVE = 0,        ///< Heater off.
  TP_FIRST_HEATING,       ///< Target set, not reached yet.
  TP_STABLE               ///< Target reached, holding.
};

static struct {
  uint16_t target;        ///< Target the state machine refers to.
  uint16_t timer;         ///< Runaway / first heating countdown, ticks.
  uint16_t watch_target;  ///< Temperature to reach, 0 = not watching.
  uint16_t watch_timer;   ///< Watch countdown, ticks.
  uint8_t  state;
  uint8_t  max_count;     ///< Consecutive readings above MAXTEMP.
  uint8_t  min_count;     ///< Consecutive readings below MINTEMP.
  uint16_t off_ref;       ///< Heater off: temperature at period start.
  uint16_t off_timer;     ///< Heater off: period countdown, ticks.
  uint8_t  off_state;     ///< OFF_IDLE, OFF_SETTLING, OFF_WATCHING.
  uint8_t  off_count;     ///< Consecutive readings above the allowed rise.
} tp[NUM_TEMP_SENSORS];

enum {
  OFF_IDLE = 0,           ///< Heater on (or check disabled).
  OFF_SETTLING,           ///< First period after switching off, not judged.
  OFF_WATCHING            ///< Heater off for more than a period.
};

static void get_params(uint8_t sensor, tp_param_t *p) {
  #ifdef HEATER_BED
    if (sensor == (uint8_t)TEMP_SENSOR_bed) {
      p->period       = THERMAL_PROTECTION_BED_PERIOD * TP_TICKS_PER_S;
      p->hysteresis   = QC(THERMAL_PROTECTION_BED_HYSTERESIS);
      p->watch_period = WATCH_BED_TEMP_PERIOD * TP_TICKS_PER_S;
      p->watch_inc    = QC(WATCH_BED_TEMP_INCREASE);
      p->maxtemp      = QC(BED_MAXTEMP);
      p->mintemp      = QC(BED_MINTEMP);
      p->max_target   = QC(BED_MAXTEMP - BED_OVERSHOOT);
      p->off_period   = THERMAL_PROTECTION_BED_OFF_PERIOD * TP_TICKS_PER_S;
      p->off_rise     = QC(THERMAL_PROTECTION_BED_OFF_RISE);
      return;
    }
  #else
    (void)sensor;
  #endif
  p->period       = THERMAL_PROTECTION_PERIOD * TP_TICKS_PER_S;
  p->hysteresis   = QC(THERMAL_PROTECTION_HYSTERESIS);
  p->watch_period = WATCH_TEMP_PERIOD * TP_TICKS_PER_S;
  p->watch_inc    = QC(WATCH_TEMP_INCREASE);
  p->maxtemp      = QC(HEATER_MAXTEMP);
  p->mintemp      = QC(HEATER_MINTEMP);
  p->max_target   = QC(HEATER_MAXTEMP - HOTEND_OVERSHOOT);
  p->off_period   = THERMAL_PROTECTION_OFF_PERIOD * TP_TICKS_PER_S;
  p->off_rise     = QC(THERMAL_PROTECTION_OFF_RISE);
}

/**
  Heater stuck on: the heater is off, but the temperature rises. Compares
  the temperature at the end of each period with the start of the period.
*/
static void check_heater_off(uint8_t i, uint16_t current, uint8_t heater_on,
                             const tp_param_t *p) {
  if (heater_on || p->off_rise == 0 || p->off_period == 0) {
    tp[i].off_state = OFF_IDLE;
    return;
  }

  if (tp[i].off_state == OFF_IDLE) {
    // Just switched off.
    tp[i].off_state = OFF_SETTLING;
    tp[i].off_ref = current;
    tp[i].off_timer = p->off_period;
    tp[i].off_count = 0;
    return;
  }

  if (tp[i].off_timer > 1) {
    tp[i].off_timer--;
    return;
  }

  // Period over. Several readings in a row must agree, like MAXTEMP.
  if (tp[i].off_state == OFF_WATCHING &&
      (uint32_t)current > (uint32_t)tp[i].off_ref + p->off_rise) {
    if (++tp[i].off_count >= TEMP_ERROR_READINGS)
      printer_kill("Heater off but temperature rising, system stopped!", i);
    return;
  }

  tp[i].off_state = OFF_WATCHING;
  tp[i].off_ref = current;
  tp[i].off_timer = p->off_period;
  tp[i].off_count = 0;
}

/**
  (Re)start watching the heating progress, as long as we're far enough below
  the target. Close to the target the first heating timer takes over.
*/
static void start_watch(uint8_t i, uint16_t current, uint16_t target,
                        const tp_param_t *p) {
  if ((uint32_t)target > (uint32_t)current + p->watch_inc +
                         QC(TEMP_HYSTERESIS) + QC(1)) {
    tp[i].watch_target = current + p->watch_inc;
    tp[i].watch_timer = p->watch_period;
  }
  else {
    tp[i].watch_target = 0;
  }
}

void thermal_protection_check(uint8_t i, uint16_t current, uint16_t target,
                              uint8_t age, uint8_t heater_on) {
  tp_param_t p;

  if (i >= NUM_TEMP_SENSORS)
    return;

  get_params(i, &p);

  // Sensor delivers no readings while heating.
  if (target && age > TEMP_SENSOR_TIMEOUT * TP_TICKS_PER_S)
    printer_kill("Temperature sensor timeout, system stopped!", i);

  // No fresh reading, nothing to judge. Timers pause, the timeout above
  // takes care of a sensor which stopped delivering.
  if (age > 2)
    return;

  // MAXTEMP, checked always.
  if (current > p.maxtemp) {
    if (++tp[i].max_count >= TEMP_ERROR_READINGS)
      printer_kill("MAXTEMP triggered, system stopped!", i);
  }
  else {
    tp[i].max_count = 0;
  }

  #ifndef NO_THERMAL_PROTECTION
    check_heater_off(i, current, heater_on, &p);
  #else
    (void)heater_on;
  #endif

  if (target == 0) {
    tp[i].state = TP_INACTIVE;
    tp[i].target = 0;
    tp[i].watch_target = 0;
    tp[i].min_count = 0;
    return;
  }

  // MINTEMP, only while heating.
  if (current < p.mintemp) {
    if (++tp[i].min_count >= TEMP_ERROR_READINGS)
      printer_kill("MINTEMP triggered, system stopped!", i);
  }
  else {
    tp[i].min_count = 0;
  }

  #ifndef NO_THERMAL_PROTECTION
    // New target: start over.
    if (target != tp[i].target) {
      tp[i].target = target;
      tp[i].state = TP_FIRST_HEATING;
      tp[i].timer = p.period;
      start_watch(i, current, target, &p);
    }

    // Heating progress.
    if (tp[i].watch_target) {
      if (current >= tp[i].watch_target)
        start_watch(i, current, target, &p);
      else if (tp[i].watch_timer == 0 || --tp[i].watch_timer == 0)
        printer_kill("Heating failed, system stopped!", i);
    }

    switch (tp[i].state) {
      case TP_FIRST_HEATING:
        if ((uint32_t)current + p.hysteresis >= target) {
          tp[i].state = TP_STABLE;
          tp[i].timer = p.period;
        }
        else if ( ! tp[i].watch_target) {
          // Close to target, but not there yet.
          if (tp[i].timer == 0 || --tp[i].timer == 0)
            printer_kill("Heating failed, system stopped!", i);
        }
        break;

      case TP_STABLE:
        if ((uint32_t)current + p.hysteresis >= target)
          tp[i].timer = p.period;
        else if (tp[i].timer == 0 || --tp[i].timer == 0)
          printer_kill("Thermal Runaway, system stopped!", i);
        break;

      default:
        break;
    }
  #endif /* NO_THERMAL_PROTECTION */
}

uint16_t thermal_protection_limit_target(uint8_t i, uint16_t target) {
  tp_param_t p;

  get_params(i, &p);
  if (target > p.max_target) {
    serial_writestr("echo:Target temperature limited by MAXTEMP\n");
    return p.max_target;
  }
  return target;
}
