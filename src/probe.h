/** \file
  \brief Z probe (BLTouch): homing Z with the probe, G30, G29, M401/M402.
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

/// Servo and BLTouch start-up (stow).
void probe_init(void);

/// M401 / M402. \return 1 on success.
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

/// G29: probe the mesh (needs BED_LEVELING), leveling on afterwards.
uint8_t probe_mesh(void);

#endif /* Z_PROBE */

#endif /* _PROBE_H */
