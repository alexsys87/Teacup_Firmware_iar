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
  uint32_t      backlash_x;     ///< M425 X, X backlash, um.
  uint32_t      backlash_y;     ///< M425 Y, Y backlash, um.
  uint32_t      backlash_s;     ///< M425 S, smoothing distance, um.
  uint32_t      retract_length; ///< M207 S, firmware retract, um.
  uint32_t      retract_feedrate; ///< M207 F, mm/min.
  int32_t       retract_zlift;  ///< M207 Z, um.
  int32_t       recover_extra;  ///< M208 S, extra prime after G11, um.
  uint32_t      recover_feedrate; ///< M208 F, mm/min.
  uint32_t      host_timeout;   ///< M86 S, host lost after s, 0 = off.
  uint32_t      host_lost_temp; ///< M86 E, hotend then, C.
  uint32_t      plr_enabled;    ///< M413 S, power loss recovery on.
  int32_t       tool_offset[3]; ///< M218 T1 X Y Z, offset of tool 1, um.
  uint32_t      tmc_current[4]; ///< M906 X Y Z E, run current, mA.
  uint32_t      tmc_stealth;    ///< M569 S, stealthChop per axis, bits X..E.
  uint32_t      junction_dev;   ///< M205 J, junction deviation, um, 0 = jerk.
  uint32_t      min_segment_us; ///< M205 B, min. segment time (queue low), us.
  uint32_t      filament_dia;   ///< M200 D, filament diameter, um.
  uint32_t      vol_enabled;    ///< M200 S1: E values are mm^3.
  uint32_t      vol_limit;      ///< M200 L, max. volumetric speed, mm^3/s / 1000, 0 = off.
  uint32_t      tmc_hybrid[4];  ///< M913, stealthChop up to mm/s, 0 = always.
  float         mpc[6];         ///< M306 P C R A F H, see heater.h.
  int32_t       skew_xz;        ///< M852 J, XZ skew factor, millionths.
  int32_t       skew_yz;        ///< M852 K, YZ skew factor, millionths.
  int32_t       twist[7];       ///< M423, probe Z correction along X, um.
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
