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
