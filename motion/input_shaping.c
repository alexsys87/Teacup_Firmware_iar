/** \file
  \brief Input shaping (M593, ZV / MZV) and S-curve smoothing of X and Y.

  Input shaping cancels the ringing of an axis at its resonance frequency:
  the motion is split into two (ZV) or three (MZV) delayed copies of
  reduced size, timed so that the vibrations they excite cancel out.
  Shaped position, with nominal position x(t) from the moves:

    xs(t) = sum over copies k of  A[k] * x(t - T[k])

  S-curve smoothing averages that over a window of W:

    xss(t) = 1 / W * integral from t - W to t of xs(s) ds

  which turns the constant acceleration of the moves into one ramping up
  and down within W (jerk limited), across move boundaries, too. With
  W = 0 it's plain input shaping, without shaping one copy of weight 1.

  Both are done on the step level, positions stay exact:

   - dda_step() hands X and Y steps to shaper_step(), which stores their
     time (when the Bresenham algorithm wanted them) and direction in a
     history.
   - For each copy k two read pointers follow the history: a step enters
     the window of copy k at its time + T[k] and leaves it W later. Steps
     that left count fully (p[k]), steps inside the window with the time
     they spent in it (dsum[k] / W).
   - shaper_service() steps the motor towards the rounded shaped position,
     one step at a time, at least IS_STEP_GAP apart, setting the direction
     pin as needed, and asks to run again at the next event of a copy or
     when the smoothed position will have moved by one step. It's part of
     the auxiliary step generator in the step interrupt (hal/timer.c).

  Moves with endstop checks (homing, probing) aren't shaped: a delayed
  axis would stop later than the endstop saw it.

  Copies, with damped frequency fd = f * sqrt(1 - zeta^2), td = 1 / fd
  (as in Klipper):
    ZV:  K = exp(-zeta * pi / sqrt(1 - zeta^2))
         A = 1, K;  T = 0, td / 2
    MZV: K = exp(-0.75 * zeta * pi / sqrt(1 - zeta^2)), a = 1 - 1 / sqrt(2)
         A = a, (sqrt(2) - 1) * K, a * K^2;  T = 0, 0.375 td, 0.75 td
  normalized to a sum of 1.
*/

#include "input_shaping.h"

#ifdef INPUT_SHAPING

#include <string.h>
#include <math.h>
#include "pinio.h"
#include "timer.h"
#include "atomic.h"
#include "settings.h"

#define IS_HIST       INPUT_SHAPING_BUFFER
#if (IS_HIST & (IS_HIST - 1)) != 0
  #error INPUT_SHAPING_BUFFER must be a power of 2.
#endif
#define IS_MASK       (IS_HIST - 1)
#define IS_PI         3.14159265f
#define IS_COPIES     3

/// Minimum time between two steps of an axis, see LA_STEP_GAP.
#define IS_STEP_GAP   (3UL * STEP_PULSE_CYCLES)

/// Time from a change of the direction pin to the next step.
#define IS_DIR_SETUP  ((uint32_t)(2 US))

typedef struct {
  uint8_t   n;                  ///< Copies, 0 = axis not shaped.
  float     w[IS_COPIES];       ///< Weights, sum 1.
  uint32_t  d[IS_COPIES];       ///< Delays, CPU ticks, ascending, d[0] = 0.
  uint32_t  win;                ///< Smoothing window, CPU ticks, 0 = none.
  float     inv_win;            ///< 1 / win, 0 without window.
} shaper_cfg_t;

typedef struct {
  shaper_cfg_t cfg;
  uint32_t  ev[IS_HIST];        ///< Step times, bit 0: 1 = forward.
  uint32_t  head;               ///< Steps written (free running index).
  uint32_t  r_in[IS_COPIES];    ///< Steps entered the window of copy k.
  uint32_t  r_out[IS_COPIES];   ///< Steps left the window of copy k.
  int32_t   p[IS_COPIES];       ///< Net steps that left the window.
  int32_t   s1[IS_COPIES];      ///< Net steps inside the window.
  int64_t   dsum[IS_COPIES];    ///< Their time inside, at tref[k].
  uint32_t  tref[IS_COPIES];
  int32_t   nominal;            ///< Position from the Bresenham steps.
  int32_t   actual;             ///< Position of the motor.
  uint32_t  last_step, last_dir;
  uint8_t   dir;                ///< Direction pin: 1 fwd, 0 back, 2 unknown.
} shaper_axis_t;

static shaper_axis_t sa[2];
static shaper_cfg_t cfg_new[2];
static volatile uint8_t cfg_pending;

volatile uint8_t shaper_on[2];
volatile uint32_t shaper_overflows;

TEACUP_HOT
TEACUP_STEP_RAMFUNC void shaper_step(uint8_t axis, uint8_t forward) {
  shaper_axis_t *s = &sa[axis];
  int32_t sign = forward ? 1 : -1;
  uint8_t k;

  s->nominal += sign;
  if (s->head - s->r_out[s->cfg.n - 1] >= IS_HIST) {
    // History full: this step isn't shaped, all copies take it right away.
    for (k = 0; k < s->cfg.n; k++)
      s->p[k] += sign;
    shaper_overflows++;
  }
  else {
    // The step was due at the current compare time of the step timer.
    s->ev[s->head & IS_MASK] = (TIM5->CCR1 & ~1UL) | (forward ? 1UL : 0UL);
    s->head++;
  }
  aux_kicked = 1;
}

/// Let the copies take the steps due at 'now'.
TEACUP_HOT
TEACUP_STEP_RAMFUNC static void advance_copies(shaper_axis_t *s, uint32_t now) {
  uint32_t win = s->cfg.win;
  uint8_t k;

  for (k = 0; k < s->cfg.n; k++) {
    uint32_t dk = s->cfg.d[k];

    // Steps enter and leave in time order, leaving first at equal times.
    for (;;) {
      uint32_t t_in = 0, t_out = 0;
      uint8_t in_due = 0, out_due = 0, leave;
      int32_t sign;

      if (s->r_in[k] != s->head) {
        t_in = (s->ev[s->r_in[k] & IS_MASK] & ~1UL) + dk;
        in_due = (int32_t)(now - t_in) >= 0;
      }
      if (s->r_out[k] != s->r_in[k]) {
        t_out = (s->ev[s->r_out[k] & IS_MASK] & ~1UL) + dk + win;
        out_due = (int32_t)(now - t_out) >= 0;
      }
      if ( ! in_due && ! out_due)
        break;
      leave = out_due && ( ! in_due || (int32_t)(t_in - t_out) >= 0);

      if (leave) {
        sign = (s->ev[s->r_out[k] & IS_MASK] & 1) ? 1 : -1;
        s->dsum[k] += (int64_t)s->s1[k] * (int32_t)(t_out - s->tref[k]);
        s->tref[k] = t_out;
        s->s1[k] -= sign;
        s->p[k] += sign;
        s->dsum[k] -= (int64_t)sign * win;
        s->r_out[k]++;
      }
      else {
        sign = (s->ev[s->r_in[k] & IS_MASK] & 1) ? 1 : -1;
        s->dsum[k] += (int64_t)s->s1[k] * (int32_t)(t_in - s->tref[k]);
        s->tref[k] = t_in;
        s->s1[k] += sign;
        s->r_in[k]++;
      }
    }
    if (win) {
      s->dsum[k] += (int64_t)s->s1[k] * (int32_t)(now - s->tref[k]);
      s->tref[k] = now;
    }
  }
}

/// Shaped position minus motor position, in steps.
TEACUP_HOT
TEACUP_STEP_RAMFUNC static float offset(const shaper_axis_t *s) {
  float d = 0.f;
  uint8_t k;

  for (k = 0; k < s->cfg.n; k++) {
    float x = (float)(s->p[k] - s->actual);

    if (s->cfg.win)
      x += (float)s->dsum[k] * s->cfg.inv_win;
    d += s->cfg.w[k] * x;
  }
  return d;
}

/// One step pulse of X or Y. Out of line: tests hook it.
TEACUP_HOT TEACUP_NOINLINE
TEACUP_STEP_RAMFUNC static void shaper_pulse(uint8_t axis) {
  #ifdef STEP_TIMER_PULSES
    if (step_timer[axis]) {
      STEP_TRIGGER(step_timer[axis]);
      return;
    }
  #endif
  {
    step_set_t st;

    step_set_clear(&st);
    if (axis == 0)
      step_add_x(&st);
    else
      step_add_y(&st);
    step_output(&st);
    timer_step_pulse_end();
  }
}

TEACUP_HOT
TEACUP_STEP_RAMFUNC static uint32_t axis_service(uint8_t axis, uint32_t now) {
  shaper_axis_t *s = &sa[axis];
  uint32_t next = AUX_NONE, since;
  float d;
  uint8_t k, dir;

  if ( ! s->cfg.n)
    return AUX_NONE;

  advance_copies(s, now);
  d = offset(s);

  if (d > 0.5f || d < -0.5f) {
    dir = (d > 0.f) ? 1 : 0;
    if (dir != s->dir) {
      if (axis == 0)
        x_direction(dir);
      else
        y_direction(dir);
      s->dir = dir;
      s->last_dir = now;
      return IS_DIR_SETUP;
    }
    since = now - s->last_dir;
    if (since < IS_DIR_SETUP)
      return IS_DIR_SETUP - since;
    since = now - s->last_step;
    if (since < IS_STEP_GAP)
      return IS_STEP_GAP - since;

    shaper_pulse(axis);
    s->actual += dir ? 1 : -1;
    s->last_step = now;
    d -= dir ? 1.f : -1.f;
    if (d > 0.5f || d < -0.5f)
      return IS_STEP_GAP;
  }

  // Next step entering or leaving the window of a copy.
  for (k = 0; k < s->cfg.n; k++) {
    uint32_t t;
    int32_t dt;

    if (s->r_in[k] != s->head) {
      t = (s->ev[s->r_in[k] & IS_MASK] & ~1UL) + s->cfg.d[k];
      dt = (int32_t)(t - now);
      if (dt < 1)
        dt = 1;
      if ((uint32_t)dt < next)
        next = (uint32_t)dt;
    }
    if (s->r_out[k] != s->r_in[k]) {
      t = (s->ev[s->r_out[k] & IS_MASK] & ~1UL) + s->cfg.d[k] + s->cfg.win;
      dt = (int32_t)(t - now);
      if (dt < 1)
        dt = 1;
      if ((uint32_t)dt < next)
        next = (uint32_t)dt;
    }
  }

  // With smoothing the position moves between these events, too: when
  // will it be half a step away?
  if (s->cfg.win) {
    float slope = 0.f, dt = -1.f;

    for (k = 0; k < s->cfg.n; k++)
      slope += s->cfg.w[k] * (float)s->s1[k];
    slope *= s->cfg.inv_win;                  // Steps per CPU tick.
    if (slope > 0.f)
      dt = (0.5f - d) / slope;
    else if (slope < 0.f)
      dt = (-0.5f - d) / slope;
    if (dt >= 0.f) {
      uint32_t t = (dt < 2.0e9f) ? (uint32_t)dt + 1 : 2000000000UL;

      if (t < next)
        next = t;
    }
  }

  return next;
}

/// X or Y has nothing left to do.
static uint8_t axis_idle(const shaper_axis_t *s) {
  return (s->cfg.n == 0 || s->r_out[s->cfg.n - 1] == s->head) &&
         s->actual == s->nominal;
}

/// Take the new configuration, X and Y are idle. Interrupts locked.
static void apply_config(void) {
  uint32_t now = TIM5->CNT;
  uint8_t a, k;

  for (a = 0; a < 2; a++) {
    shaper_axis_t *s = &sa[a];

    s->cfg = cfg_new[a];
    for (k = 0; k < IS_COPIES; k++) {
      s->r_in[k] = s->r_out[k] = s->head;
      s->p[k] = s->nominal;
      s->s1[k] = 0;
      s->dsum[k] = 0;
      s->tref[k] = now;
    }
    s->dir = 2;
    shaper_on[a] = s->cfg.n ? 1 : 0;
  }
  cfg_pending = 0;
}

TEACUP_HOT
TEACUP_STEP_RAMFUNC uint32_t shaper_service(uint32_t now) {
  uint32_t next = axis_service(0, now);
  uint32_t d = axis_service(1, now);

  if (d < next)
    next = d;
  if (cfg_pending && axis_idle(&sa[0]) && axis_idle(&sa[1]))
    apply_config();
  return next;
}

void shaper_dir_unknown(void) {
  sa[0].dir = sa[1].dir = 2;
}

uint8_t shaper_busy(void) {
  uint8_t busy;

  ATOMIC_START();
    busy = ! axis_idle(&sa[0]) || ! axis_idle(&sa[1]);
    if ( ! busy && cfg_pending)
      apply_config();
  ATOMIC_END();
  return busy;
}

/// Configuration of one axis from frequency, damping, type and window.
static void make_config(shaper_cfg_t *c, uint32_t freq_centi,
                        uint32_t damp_milli, uint32_t type, uint32_t win_us) {
  float a[IS_COPIES], t[IS_COPIES], sum = 0.f;
  float f = (float)freq_centi * 0.01f;
  float z = (float)damp_milli * 0.001f;
  uint8_t k;

  memset(c, 0, sizeof(*c));
  if (f > 0.f && z < 1.f) {
    float df = sqrtf(1.f - z * z);
    float td = 1.f / (f * df);
    float kk;

    if (type == 0) {                          // ZV
      kk = expf(-z * IS_PI / df);
      a[0] = 1.f;       t[0] = 0.f;
      a[1] = kk;        t[1] = 0.5f * td;
      c->n = 2;
    }
    else {                                    // MZV
      float a1 = 1.f - 1.f / sqrtf(2.f);

      kk = expf(-0.75f * z * IS_PI / df);
      a[0] = a1;                        t[0] = 0.f;
      a[1] = (sqrtf(2.f) - 1.f) * kk;   t[1] = 0.375f * td;
      a[2] = a1 * kk * kk;              t[2] = 0.75f * td;
      c->n = 3;
    }
  }
  else if (win_us) {                          // Smoothing only.
    a[0] = 1.f;
    t[0] = 0.f;
    c->n = 1;
  }

  for (k = 0; k < c->n; k++)
    sum += a[k];
  for (k = 0; k < c->n; k++) {
    c->w[k] = a[k] / sum;
    c->d[k] = (uint32_t)(t[k] * (float)F_CPU + 0.5f);
  }
  if (c->n && win_us) {
    c->win = win_us * (F_CPU / 1000000UL);
    c->inv_win = 1.f / (float)c->win;
  }
}

void shaper_configure(void) {
  shaper_cfg_t c[2];
  uint8_t a;

  for (a = 0; a < 2; a++)
    make_config(&c[a], settings.is_freq[a], settings.is_damp[a],
                settings.is_type[a], settings.s_curve_us);

  ATOMIC_START();
    if (memcmp(c, cfg_new, sizeof(c)) != 0 ||
        memcmp(&c[0], &sa[0].cfg, sizeof(c[0])) != 0 ||
        memcmp(&c[1], &sa[1].cfg, sizeof(c[1])) != 0) {
      memcpy(cfg_new, c, sizeof(c));
      cfg_pending = 1;
      if (axis_idle(&sa[0]) && axis_idle(&sa[1]))
        apply_config();
    }
  ATOMIC_END();
}

#endif /* INPUT_SHAPING */
