/** \file
  \brief Settings changeable at runtime (M92, M201..M205, M206, M301, M304,
         M84 S) and their storage in Flash (M500..M503).
*/

#ifndef _SETTINGS_H
#define _SETTINGS_H

#include <stdint.h>
#include "dda.h"

typedef struct {
  axes_uint32_t steps_per_m;    ///< M92, steps per meter.
  axes_uint32_t max_feedrate;   ///< M203, mm/min.
  axes_uint32_t max_accel;      ///< M201, mm/s^2 per axis.
  uint32_t      acceleration;   ///< M204 P, printing moves, mm/s^2.
  axes_uint32_t max_jerk;       ///< M205, mm/min.
  uint32_t      accel_retract;  ///< M204 R, E-only moves, mm/s^2.
  uint32_t      accel_travel;   ///< M204 T, moves without E, mm/s^2.
  uint32_t      la_k;           ///< M900 K, linear advance, 1/10000 s.
  uint32_t      is_freq[2];     ///< M593 F, shaping of X, Y, 1/100 Hz, 0 = off.
  uint32_t      is_damp[2];     ///< M593 D, damping ratio, 1/1000.
  uint32_t      is_type[2];     ///< M593 T, 0 = ZV, 1 = MZV.
  uint32_t      s_curve_us;     ///< M593 S, S-curve smoothing of X/Y, us.
  int32_t       skew_xy;        ///< M852 I, XY skew factor, millionths.
  uint32_t      backlash_z;     ///< M425 Z, Z backlash, um.
  uint32_t      backlash_f;     ///< M425 F, fraction corrected, 1/1000.
} settings_t;

/// Motion settings, read directly by the motion code.
extern settings_t settings;

/**
  Acceleration of a travel move of 'axis' alone (e.g. homing): the lower
  of M204 T and the M201 value of that axis.
*/
uint32_t settings_axis_accel(enum axis_e axis);

/// Startup: load from Flash, or use the compiled-in defaults.
void settings_init(void);

/// Compiled-in defaults (M502).
void settings_defaults(void);

/// Recalculate everything derived from the motion settings.
void settings_apply(void);

/// Save to Flash (M500). \return 1 on success.
uint8_t settings_save(void);

/// Load from Flash (M501). \return 1 on success.
uint8_t settings_load(void);

/// Report all settings as G-code (M503).
void settings_report(void);

#endif /* _SETTINGS_H */
