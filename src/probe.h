/** \file
  \brief Z probe (BLTouch or inductive sensor): homing Z with the probe,
  G30, G29, M401/M402.
*/

#ifndef _PROBE_H
#define _PROBE_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef Z_PROBE

/// Probe offset from the nozzle, um (M851). Probe = nozzle + offset.
/// Z: nozzle height above the bed when the probe triggers, negated.
extern int32_t probe_offset[3];

/// Configured defaults of probe_offset (Z_PROBE_OFFSET_*).
void probe_defaults(void);

/**
  X axis twist compensation (M423, Marlin X_AXIS_TWIST_COMPENSATION): a
  sagging or twisted X gantry tilts the probe against the nozzle, so the
  probe sees the bed higher or lower than the nozzle does, depending on X.
  settings.twist[i] (um) is added to probe measurements at TWIST_POINTS
  places from X_TWIST_START to X_TWIST_END (probe position), linear in
  between and beyond. Value at a point: Z of the nozzle touching (paper)
  minus the probe result (G30) there.
*/
#ifndef TWIST_POINTS
  #define TWIST_POINTS 3
#endif
#if TWIST_POINTS < 2 || TWIST_POINTS > 7
  #error TWIST_POINTS must be 2..7.
#endif
#ifndef X_TWIST_START
  #define X_TWIST_START (X_MIN + 15.)
#endif
#ifndef X_TWIST_END
  #define X_TWIST_END (X_MAX - 15.)
#endif

/// Probe X position of twist point i, um.
int32_t probe_twist_x(uint8_t i);

/// Twist correction for a probe measurement at probe X px, um.
int32_t probe_twist(int32_t px);

/// Servo and BLTouch start-up (stow); nothing for an inductive sensor.
void probe_init(void);

/**
  M401 / M402. BLTouch: pin down / up. Inductive sensor: deploy checks it
  isn't active yet, stow does nothing. \return 1 on success.
*/
uint8_t probe_deploy(void);
uint8_t probe_stow(void);

/**
  G28 Z: home Z with the probe at Z_SAFE_HOMING_X/Y (default: probe at the
  bed center). X and Y must be homed. \return 1 on success.
*/
uint8_t probe_home_z(void);

/**
  G30: probe the bed at the probe position px, py (um) and report it.
  \return 1 on success, *bed_z = bed height there (um).
*/
uint8_t probe_single(int32_t px, int32_t py, int32_t *bed_z);

/**
  G34: align Z and Z2 (Z_STEPPER_ALIGN): probe at Z_STEPPER_ALIGN_X1 / X2,
  raise the lower lead screw alone, repeat until the difference is at most
  'accuracy' um, then home Z again. \return 1 when aligned.
*/
uint8_t probe_align_z(uint8_t iterations, int32_t accuracy);

/// G29: probe the mesh (needs BED_LEVELING), leveling on afterwards.
uint8_t probe_mesh(void);

#endif /* Z_PROBE */

#endif /* _PROBE_H */
