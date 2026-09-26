/** \file
  \brief Z correction: bed leveling mesh and Z offset (babystepping).

  The motors run at "physical" Z = logical Z + correction, where

    correction = mesh(X, Y) * fade(Z) + Z offset (M290)

  The mesh part needs BED_LEVELING, the Z offset BABYSTEPPING. Without
  them the correction is always 0 and all of this compiles to nothing.
*/

#ifndef _BED_LEVELING_H
#define _BED_LEVELING_H

#include <stdint.h>

#include "config_wrapper.h"
#include "dda.h"

/**
  Correction in um for a logical position, added to Z before the conversion
  to motor steps. 0 while suspended (homing, probing).
*/
#if defined BED_LEVELING || defined BABYSTEPPING
  int32_t bed_level_offset(const axes_int32_t axis);
#else
  TEACUP_INLINE int32_t bed_level_offset(const axes_int32_t axis) {
    (void)axis;
    return 0;
  }
#endif

/**
  Switch the correction off (nested) or on again. The motors don't move:
  the logical Z of startpoint changes instead, so it reads the physical
  position while suspended. Homing and probing work in physical Z.
*/
void zcorr_suspend(void);
void zcorr_resume(void);

/**
  The motors are at startpoint_steps, the logical position in startpoint
  is outdated for Z (the correction changed, homing): recalculate the
  logical Z from the motor position, also the G-code target of Z.
*/
void zcorr_sync_logical(void);

/// Mesh storage, part of the settings (M500).
typedef struct {
  uint8_t  nx, ny;          ///< Grid points (0 = no mesh).
  uint8_t  active;          ///< Leveling on (M420 S1).
  uint8_t  reserved;
  int32_t  x0, y0;          ///< Position of point [0][0], um.
  int32_t  dx, dy;          ///< Grid spacing, um.
  int32_t  fade;            ///< Fade height, um, 0 = no fade (M420 Z).
  int16_t  z[7][7];         ///< Bed height at the points, um, [y][x].
} mesh_t;

#ifdef BED_LEVELING

extern mesh_t mesh;

/// Defaults: no mesh, leveling off, configured fade height.
void bed_level_defaults(void);

/// Leveling on/off (M420 S), keeps the motor position. \return new state.
uint8_t bed_level_set_active(uint8_t on);

/// Whether leveling is on and a mesh exists.
uint8_t bed_level_active(void);

/// Set the fade height (M420 Z), um, 0 = off. Keeps the motor position.
void bed_level_set_fade(int32_t fade_um);

/**
  Start a new mesh (G29): grid geometry, all points 0, leveling off.
*/
void bed_level_new(int32_t x0, int32_t y0, int32_t dx, int32_t dy,
                   uint8_t nx, uint8_t ny);

/// Set one point (G29, M421), um. \return 0 if the index is outside.
uint8_t bed_level_set_point(uint8_t ix, uint8_t iy, int32_t z_um);

/// Report state, fade height and the mesh (M420 without S).
void bed_level_report(void);

/**
  Queue a move, split at the grid lines while leveling is on, so Z follows
  the bilinear surface. Waits for queue space (clock_poll()), stops early
  after an M410. Use instead of enqueue() for printing moves.
*/
void bed_level_enqueue(TARGET *t);

#else

TEACUP_INLINE uint8_t bed_level_active(void) {
  return 0;
}

#endif /* BED_LEVELING */

#if ! defined BED_LEVELING
  #include "dda_queue.h"
  /// Without a mesh there is nothing to split.
  #define bed_level_enqueue(t) enqueue(t)
#endif

#endif /* _BED_LEVELING_H */
