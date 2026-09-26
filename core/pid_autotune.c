/** \file
  \brief PID autotune (M303), a port of Marlin's PID_autotune().

  The heater is switched between bias + d and bias - d whenever the
  temperature crosses the target (at least 5 s per half period). After the
  first cycles bias and d are adjusted to make the oscillation symmetric.
  From the amplitude (max - min) and the period Tu:

    Ku = 4 * d / (pi * (max - min) / 2)
    Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8

  Results are reported in Marlin units (Kp, Ki, Kd per second, as used by
  M301/M304) and in Teacup units (DEFAULT_P etc. for the printer config).
  With U1 they're applied right away; M500 stores them.

  Thermal protection stays active: the target is set for the sensor, only
  the PID loop is bypassed. M108 aborts.
*/

#include "pid_autotune.h"

#include "config_wrapper.h"
#include "clock.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"

/// Abort if the temperature exceeds the target by this much.
#define AUTOTUNE_MAX_OVERSHOOT  30
/// Abort after this time without finishing, ms.
#define AUTOTUNE_TIMEOUT        (20UL * 60 * 1000)

/** Print a float with two decimals. */
static void write_float2(float f) {
  uint32_t v;

  if (f < 0) {
    serial_writechar('-');
    f = -f;
  }
  v = (uint32_t)(f * 100.0f + 0.5f);
  serwrite_uint32(v / 100);
  serial_writechar('.');
  serial_writechar((char)('0' + (v / 10) % 10));
  serial_writechar((char)('0' + v % 10));
}

static void finish(heater_t h, temp_sensor_t s) {
  heater_set(h, 0);
  temp_set_manual(s, 0);
  temp_set(s, 0);
}

void pid_autotune(heater_t h, temp_sensor_t s, uint16_t target,
                  uint8_t cycles_wanted, uint8_t apply) {
  float kp = 0, ki = 0, kd = 0;
  float max_t = 0, min_t = 10000;
  float target_f = (float)target;
  int32_t bias = 127, d = 127;
  uint32_t t1, t2, t_high = 0, t_low = 0, start, last_sample, last_report;
  uint8_t heating = 1, cycles = 0, have_result = 0;

  if (h >= NUM_HEATERS || s >= NUM_TEMP_SENSORS) {
    serial_writestr("echo:PID Autotune failed! Bad heater\n");
    return;
  }
  if (cycles_wanted < 3)
    cycles_wanted = 3;
  if (cycles_wanted > 20)
    cycles_wanted = 20;

  serial_writestr("PID Autotune start\n");

  // Target for thermal protection (clamped to MAXTEMP - overshoot), but no
  // PID: we drive the heater ourselves.
  temp_set(s, (uint16_t)(target * 4));
  temp_set_manual(s, 1);
  (void)temp_m108_seen();

  heater_set(h, 255);
  start = t1 = t2 = last_sample = last_report = clock_millis();

  for (;;) {
    uint32_t now;
    float cur;

    clock_poll();

    if (temp_m108_seen()) {
      serial_writestr("PID Autotune failed! (M108)\n");
      finish(h, s);
      return;
    }

    now = clock_millis();
    if (now - last_sample < 250)
      continue;                   // A new reading every 250 ms.
    last_sample = now;

    cur = (float)temp_get(s) / 4.0f;
    if (cur > max_t)
      max_t = cur;
    if (cur < min_t)
      min_t = cur;

    if (heating && cur > target_f) {
      if (now - t2 > 5000) {
        heating = 0;
        heater_set(h, (uint8_t)(bias - d));
        t1 = now;
        t_high = t1 - t2;
        max_t = target_f;
      }
    }

    if ( ! heating && cur < target_f) {
      if (now - t1 > 5000) {
        heating = 1;
        t2 = now;
        t_low = t2 - t1;
        if (cycles > 0) {
          bias += (d * ((int32_t)t_high - (int32_t)t_low)) /
                  (int32_t)(t_low + t_high);
          if (bias < 20)
            bias = 20;
          if (bias > 255 - 20)
            bias = 255 - 20;
          d = (bias > 127) ? 254 - bias : bias;

          sersendf_P((" bias: %ld d: %ld min: "), bias, d);
          write_float2(min_t);
          serial_writestr(" max: ");
          write_float2(max_t);

          if (cycles > 2) {
            float ku = (4.0f * (float)d) /
                       (3.14159265f * (max_t - min_t) * 0.5f);
            float tu = (float)(t_low + t_high) * 0.001f;

            kp = 0.6f * ku;
            ki = 2.0f * kp / tu;
            kd = kp * tu * 0.125f;
            have_result = 1;

            serial_writestr(" Ku: ");
            write_float2(ku);
            serial_writestr(" Tu: ");
            write_float2(tu);
            serial_writestr("\n Classic PID\n Kp: ");
            write_float2(kp);
            serial_writestr(" Ki: ");
            write_float2(ki);
            serial_writestr(" Kd: ");
            write_float2(kd);
          }
          serial_writechar('\n');
        }
        heater_set(h, (uint8_t)(bias + d));
        cycles++;
        min_t = target_f;
      }
    }

    if (cur > target_f + AUTOTUNE_MAX_OVERSHOOT) {
      serial_writestr("PID Autotune failed! Temperature too high\n");
      finish(h, s);
      return;
    }

    if (now - last_report >= 2000) {
      last_report = now;
      temp_print(TEMP_SENSOR_none);
    }

    if (now - start > AUTOTUNE_TIMEOUT) {
      serial_writestr("PID Autotune failed! timeout\n");
      finish(h, s);
      return;
    }

    if (cycles > cycles_wanted && have_result) {
      // Teacup units: P = Kp * 256, I = Ki * 64, D = Kd * 128.
      int32_t p = (int32_t)(kp * 256.0f + 0.5f);
      int32_t i = (int32_t)(ki * 64.0f + 0.5f);
      int32_t dd = (int32_t)(kd * 128.0f + 0.5f);
      int32_t ilim = i ? 255L * 1024 / i : 32767;
      const char *pre = "DEFAULT_";

      finish(h, s);

      serial_writestr("PID Autotune finished! Put the last Kp, Ki and Kd "
                      "constants from below into Configuration.h\n");
      #ifdef HEATER_BED
        if (h == HEATER_BED)
          pre = "DEFAULT_bed";
      #endif
      serial_writestr("#define "); serial_writestr(pre);
      serial_writestr("Kp "); write_float2(kp); serial_writechar('\n');
      serial_writestr("#define "); serial_writestr(pre);
      serial_writestr("Ki "); write_float2(ki); serial_writechar('\n');
      serial_writestr("#define "); serial_writestr(pre);
      serial_writestr("Kd "); write_float2(kd); serial_writechar('\n');
      sersendf_P(("echo:Teacup: DEFAULT_P %ld DEFAULT_I %ld DEFAULT_D %ld "
                  "DEFAULT_I_LIMIT %ld\n"), p, i, dd, ilim);

      if (apply) {
        pid_set_p(h, p);
        pid_set_i(h, i);
        pid_set_d(h, dd);
        pid_set_i_limit(h, ilim);
        serial_writestr("echo:PID values applied, M500 to store them\n");
      }
      return;
    }
  }
}
