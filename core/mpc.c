/** \file
  \brief Model predictive temperature control of the hotend, see mpc.h.

  Like Marlin's MPCTEMP: instead of reacting to the temperature error (PID),
  a model of the hotend says which power is needed.

  Model, two masses: the heater block (heat capacity C, heater power P,
  losses A to the ambient, with the part fan up to F at full speed) and the
  sensor, which follows the block with the responsiveness R:

    dT_block / dt  = (P * out - (T_block - T_ambient) * (A + (F - A) * fan)) / C
    dT_sensor / dt = (T_block - T_sensor) * R

  Every 100 ms the model runs one step with the output of the last step,
  then is pulled halfway towards the measured temperature (noise averages
  out, model errors can't grow). While the output isn't saturated or the
  model is at rest, the difference moves the modeled ambient temperature:
  it takes the part of an integral term, without windup.

  Power for the target:
    - bring the block to the target within 2 s: (target - T_block) * C / 2
    - the losses at the target: (target - T_ambient) * (A + (F - A) * fan)
    - heating the filament: E speed (mm/s) * H * (target - T_ambient)
  The fan and the extrusion act the moment they start, before the
  temperature drops.

  M306 T measures the model: cooling to ambient (fan on), heating at full
  power (fit of an exponential gives C, R and a first A), holding the
  target with the model (losses A, with the fan F). Only P has to be given
  (heater cartridge, e.g. 40 W).
*/

#include "mpc.h"

#ifdef HOTEND_MPC

#include <math.h>

#include "heater.h"
#include "temp.h"
#include "clock.h"
#include "settings.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "dda.h"
#include "dda_queue.h"

#ifndef MPC_HEATER_POWER
  #define MPC_HEATER_POWER 40.0
#endif
#ifndef MPC_BLOCK_HEAT_CAPACITY
  #define MPC_BLOCK_HEAT_CAPACITY 16.7
#endif
#ifndef MPC_SENSOR_RESPONSIVENESS
  #define MPC_SENSOR_RESPONSIVENESS 0.22
#endif
#ifndef MPC_AMBIENT_XFER_COEFF
  #define MPC_AMBIENT_XFER_COEFF 0.068
#endif
#ifndef MPC_AMBIENT_XFER_COEFF_FAN255
  #define MPC_AMBIENT_XFER_COEFF_FAN255 0.097
#endif
#ifndef MPC_FILAMENT_HEAT_CAPACITY_PERMM
  #define MPC_FILAMENT_HEAT_CAPACITY_PERMM 0.0056   // 1.75 mm PLA
#endif

/// Pull of the model towards the measurement per step (Marlin 0.5).
#define MPC_SMOOTHING      0.5f
/// Model at rest below this rate, K/s.
#define MPC_STEADYSTATE    0.5f
/// Min. change of the modeled ambient per s, K.
#define MPC_AMBIENT_MIN    1.0f

static struct {
  float block, sensor, ambient;   ///< Modeled temperatures, C.
  uint8_t last_out;               ///< Output of the last step.
  uint32_t last_ms;
  uint8_t running;
} mpc;

void mpc_defaults(void) {
  settings.mpc[MPC_P] = (float)MPC_HEATER_POWER;
  settings.mpc[MPC_C] = (float)MPC_BLOCK_HEAT_CAPACITY;
  settings.mpc[MPC_R] = (float)MPC_SENSOR_RESPONSIVENESS;
  settings.mpc[MPC_A] = (float)MPC_AMBIENT_XFER_COEFF;
  settings.mpc[MPC_F] = (float)MPC_AMBIENT_XFER_COEFF_FAN255;
  settings.mpc[MPC_H] = (float)MPC_FILAMENT_HEAT_CAPACITY_PERMM;
}

void mpc_reset(void) {
  mpc.running = 0;
}

/// Part fan speed 0..1.
static float fan_fraction(void) {
  #ifdef HEATER_FAN
    return (float)fan_get() / 255.f;
  #else
    return 0.f;
  #endif
}

/// Filament speed of the running move, mm/s (0 when retracting).
static float e_speed(void) {
  DDA *dda = mb_tail_dda;
  uint32_t c;

  if (dda == NULL || ! dda->live || ! dda->e_direction || ! dda->delta[E] ||
      ! dda->total_steps)
    return 0.f;
  c = dda->c;
  if (c == 0)
    return 0.f;
  // Fast axis steps per s * E share, in mm.
  return (float)F_CPU / (float)c * (float)dda->delta[E] /
         (float)dda->total_steps * 1000.f / (float)settings.steps_per_m[E];
}

uint8_t mpc_run(float temp, float target) {
  const float *m = settings.mpc;
  uint32_t now = clock_millis();
  float dt, xfer, block_delta, delta, power, out;

  if ( ! mpc.running) {
    mpc.block = mpc.sensor = temp;
    mpc.ambient = temp < 30.f ? temp : 30.f;
    mpc.last_out = 0;
    mpc.last_ms = now - 100;
    mpc.running = 1;
  }
  dt = (float)(now - mpc.last_ms) * 0.001f;
  if (dt < 0.01f) dt = 0.01f;
  if (dt > 1.0f) dt = 1.0f;
  mpc.last_ms = now;
  if (m[MPC_C] <= 0.f || m[MPC_P] <= 0.f)
    return 0;

  xfer = m[MPC_A] + (m[MPC_F] - m[MPC_A]) * fan_fraction();

  // Model step with the output of the last step.
  block_delta = ((float)mpc.last_out / 255.f * m[MPC_P] -
                 (mpc.block - mpc.ambient) * xfer) * dt / m[MPC_C];
  mpc.block += block_delta;
  mpc.sensor += (mpc.block - mpc.sensor) * m[MPC_R] * dt;

  // Towards the measurement.
  delta = (temp - mpc.sensor) * MPC_SMOOTHING;
  mpc.block += delta;
  mpc.sensor += delta;

  // Ambient correction while not saturated or at rest.
  if ((mpc.last_out > 0 && mpc.last_out < 255) ||
      fabsf(block_delta + delta) < MPC_STEADYSTATE * dt) {
    if (delta > 0.f)
      mpc.ambient += (delta > MPC_AMBIENT_MIN * dt) ? delta : MPC_AMBIENT_MIN * dt;
    else if (delta < 0.f)
      mpc.ambient += (delta < -MPC_AMBIENT_MIN * dt) ? delta : -MPC_AMBIENT_MIN * dt;
  }

  power = (target - mpc.block) * m[MPC_C] / 2.f +
          (target - mpc.ambient) * xfer +
          e_speed() * m[MPC_H] * (target - mpc.ambient);
  out = power * 255.f / m[MPC_P];
  if (out < 0.f) out = 0.f;
  if (out > 255.f) out = 255.f;
  mpc.last_out = (uint8_t)(out + 0.5f);
  return mpc.last_out;
}

/// A float with 'dec' decimals.
static void write_float(float f, uint8_t dec) {
  uint32_t scale = 1, v, n;
  uint8_t i;

  for (i = 0; i < dec; i++)
    scale *= 10;
  if (f < 0.f) {
    serial_writechar('-');
    f = -f;
  }
  v = (uint32_t)(f * (float)scale + 0.5f);
  serwrite_uint32(v / scale);
  if ( ! dec)
    return;
  serial_writechar('.');
  for (n = scale / 10, v %= scale; n; n /= 10) {
    serial_writechar((char)('0' + v / n));
    v %= n;
  }
}

void mpc_report(void) {
  static const char letter[MPC_PARAMS] = { 'P', 'C', 'R', 'A', 'F', 'H' };
  static const uint8_t dec[MPC_PARAMS] = { 2, 2, 4, 4, 4, 4 };
  uint8_t i;

  serial_writestr("echo:  M306");
  for (i = 0; i < MPC_PARAMS; i++) {
    serial_writechar(' ');
    serial_writechar(letter[i]);
    write_float(settings.mpc[i], dec[i]);
  }
  serial_writechar('\n');
}

/**
  Wait up to 'ms' milliseconds, keeping the clock running.
  \return 0 on M108.
*/
static uint8_t tune_wait(uint32_t ms) {
  uint32_t start = clock_millis();

  while (clock_millis() - start < ms) {
    clock_poll();
    if (temp_m108_seen())
      return 0;
  }
  return 1;
}

/// Temperature of the hotend, C.
static float hotend(void) {
  return (float)temp_get(TEMP_SENSOR_extruder) * 0.25f;
}

/**
  Hold 'target' with the model for 'settle' s, then average the heater
  power over 'measure' s. \return the average power in W, < 0 on M108.
*/
static float tune_hold(float target, uint32_t settle, uint32_t measure) {
  uint32_t t, n = 0;
  float sum = 0.f;

  mpc_reset();
  for (t = 0; t < (settle + measure) * 10; t++) {
    uint8_t out;

    if ( ! tune_wait(100))
      return -1.f;
    out = mpc_run(hotend(), target);
    heater_set(HEATER_EXTRUDER, out);
    if (t >= settle * 10) {
      sum += (float)out;
      n++;
    }
  }
  return sum / (float)n / 255.f * settings.mpc[MPC_P];
}

static void tune_end(uint8_t fan) {
  heater_set(HEATER_EXTRUDER, 0);
  temp_set_manual(TEMP_SENSOR_extruder, 0);
  temp_set(TEMP_SENSOR_extruder, 0);
  #ifdef HEATER_FAN
    fan_set(fan);
  #else
    (void)fan;
  #endif
  mpc_reset();
}

void mpc_autotune(uint16_t target) {
  #define N_SAMPLES 16
  float samples[N_SAMPLES], ambient, last, t1, t2, t3, asymp, k_block;
  float t1_time, dist, p_hold, c_new, r_new, a_new, f_new;
  uint32_t start, sample_ms = 1000, next;
  uint8_t n = 0, i, fan = 0, m;

  #ifdef HEATER_FAN
    fan = fan_get();
  #endif
  if (settings.mpc[MPC_P] <= 0.f) {
    serial_writestr("echo:MPC autotune: set the heater power first (M306 P)\n");
    return;
  }
  serial_writestr("echo:MPC autotune start\n");
  (void)temp_m108_seen();
  // No target while cooling: the thermal protection would see a heater
  // that doesn't heat.
  temp_set(TEMP_SENSOR_extruder, 0);
  heater_set(HEATER_EXTRUDER, 0);

  // 1. Cool down to ambient: until it drops less than 0.5 C in 10 s.
  serial_writestr("echo:MPC autotune: cooling to ambient\n");
  #ifdef HEATER_FAN
    fan_set(255);
  #endif
  last = hotend();
  for (i = 0; i < 120; i++) {
    if ( ! tune_wait(10000))
      goto aborted;
    if (last - hotend() < 0.5f)
      break;
    last = hotend();
  }
  ambient = hotend();
  #ifdef HEATER_FAN
    fan_set(0);
  #endif

  // 2. Full power up to the target, samples at 1, 2, 3 ... s; when the
  //    buffer is full, every other one is dropped and the distance doubles.
  sersendf_P(("echo:MPC autotune: heating to %u C\n"), target);
  // A target for the thermal protection, the heater is driven directly.
  temp_set(TEMP_SENSOR_extruder, (uint16_t)(target * 4));
  temp_set_manual(TEMP_SENSOR_extruder, 1);
  heater_set(HEATER_EXTRUDER, 255);
  start = clock_millis();
  next = sample_ms;
  for (;;) {
    if ( ! tune_wait(50))
      goto aborted;
    if (clock_millis() - start < next)
      continue;
    samples[n++] = hotend();
    if (samples[n - 1] >= (float)target)
      break;
    if (n == N_SAMPLES) {
      for (i = 0; i < N_SAMPLES / 2; i++)
        samples[i] = samples[2 * i];
      n = N_SAMPLES / 2;
      sample_ms *= 2;
      next = 1000 + (uint32_t)(n - 1) * sample_ms;   // Same grid, spaced wider.
    }
    next += sample_ms;
    if (clock_millis() - start > 20UL * 60 * 1000) {
      serial_writestr("echo:MPC autotune failed: target not reached\n");
      tune_end(fan);
      return;
    }
  }
  heater_set(HEATER_EXTRUDER, 0);

  // Three equally spaced samples: first, middle, last.
  m = (uint8_t)((n - 1) / 2);
  t1 = samples[0];
  t2 = samples[m];
  t3 = samples[2 * m];
  dist = (float)m * (float)sample_ms * 0.001f;
  t1_time = 1.f;
  asymp = (t2 * t2 - t1 * t3) / (2.f * t2 - t1 - t3);
  if (m < 2 || ! (asymp > t3) || ! ((t2 - asymp) / (t1 - asymp) > 0.f)) {
    serial_writestr("echo:MPC autotune failed: no exponential rise\n");
    tune_end(fan);
    return;
  }
  k_block = -logf((t2 - asymp) / (t1 - asymp)) / dist;
  a_new = settings.mpc[MPC_P] / (asymp - ambient);
  c_new = a_new / k_block;
  r_new = k_block / (1.f - (ambient - asymp) * expf(-k_block * t1_time) /
                           (t1 - asymp));
  settings.mpc[MPC_C] = c_new;
  settings.mpc[MPC_R] = r_new;
  settings.mpc[MPC_A] = settings.mpc[MPC_F] = a_new;

  // 3. Hold the target: losses without and with the fan.
  serial_writestr("echo:MPC autotune: measuring heat loss\n");
  p_hold = tune_hold((float)target, 20, 30);
  if (p_hold < 0.f)
    goto aborted;
  a_new = p_hold / (hotend() - ambient);
  settings.mpc[MPC_A] = settings.mpc[MPC_F] = a_new;
  settings.mpc[MPC_C] = a_new / k_block;
  #ifdef HEATER_FAN
    fan_set(255);
    p_hold = tune_hold((float)target, 20, 30);
    fan_set(0);
    if (p_hold < 0.f)
      goto aborted;
    f_new = p_hold / (hotend() - ambient);
    settings.mpc[MPC_F] = f_new > a_new ? f_new : a_new;
  #else
    (void)f_new;
  #endif

  tune_end(fan);
  serial_writestr("echo:MPC autotune finished, ambient ");
  write_float(ambient, 1);
  serial_writestr(" C, M500 stores:\n");
  mpc_report();
  return;

aborted:
  serial_writestr("echo:MPC autotune aborted (M108)\n");
  tune_end(fan);
}

#endif /* HOTEND_MPC */
