/** \file
  \brief Z correction: bed leveling mesh and Z offset, see bed_leveling.h.

  Model: motor Z = logical Z + correction(X, Y, Z). The planner converts
  logical targets to motor steps with the correction applied
  (axes_um_to_steps_*() in dda_kinematics.c), so every move follows the
  bed. startpoint_steps always holds the true motor position (end of the
  queue); whenever the correction changes without a move (leveling on/off,
  new mesh, fade height, homing, probing), the logical Z is recalculated
  from it instead (zcorr_sync_logical()), the motors never jump.

  Mesh: bilinear interpolation between the grid points, optionally of a
  subdivided grid (MESH_SUBDIVISIONS, Catmull-Rom), constant beyond the
  outer points (no extrapolation). Faded out linearly between Z = 0 and
  the fade height, above that the nozzle follows the logical Z exactly.
  Moves are split where they cross a grid line, so Z follows the surface
  between the points, too.
*/

#include "bed_leveling.h"

#include <string.h>
#include <math.h>

#include "dda_maths.h"
#include "dda_queue.h"
#include "gcode_parse.h"
#include "clock.h"
#include "serial.h"
#include "sersendf.h"
#include "emergency_parser.h"
#include "babystep.h"
#include "retract.h"

/// Nesting counter of zcorr_suspend().
static uint8_t suspended = 0;

#ifdef BED_LEVELING
mesh_t mesh;
#endif

#if defined BED_LEVELING || defined BABYSTEPPING || defined FIRMWARE_RETRACT

#ifdef BED_LEVELING
/**
  Mesh subdivision (MESH_SUBDIVISIONS, like Marlin's
  ABL_BILINEAR_SUBDIVISION): a finer virtual grid, MESH_SUBDIVISIONS points
  per cell, from a Catmull-Rom spline through the measured points (beyond
  the outer points extrapolated linearly for the spline). Z then follows a
  smooth surface instead of kinks at the grid lines. Between the virtual
  points bilinear, moves are split at the virtual grid lines. 1 = plain
  bilinear mesh.
*/
#define VGRID ((7 - 1) * MESH_SUBDIVISIONS + 1)

static int16_t vz[VGRID][VGRID];        ///< Virtual grid, um, [y][x].
static uint8_t vnx, vny;                ///< Virtual points, 0 = no mesh.
static float vdx, vdy;                  ///< Virtual spacing, um.

/// Measured point, extrapolated linearly one point beyond the mesh.
static float mesh_point(int i, int j) {
  if (i < 0)
    return 2.f * mesh_point(0, j) - mesh_point(1, j);
  if (i >= mesh.nx)
    return 2.f * mesh_point(mesh.nx - 1, j) - mesh_point(mesh.nx - 2, j);
  if (j < 0)
    return 2.f * mesh_point(i, 0) - mesh_point(i, 1);
  if (j >= mesh.ny)
    return 2.f * mesh_point(i, mesh.ny - 1) - mesh_point(i, mesh.ny - 2);
  return (float)mesh.z[j][i];
}

/// Catmull-Rom spline between p1 (t = 0) and p2 (t = 1).
static float catmull_rom(float p0, float p1, float p2, float p3, float t) {
  return 0.5f * (2.f * p1 + (p2 - p0) * t +
                 (2.f * p0 - 5.f * p1 + 4.f * p2 - p3) * t * t +
                 (3.f * (p1 - p2) + p3 - p0) * t * t * t);
}

/// Compute the virtual grid from the mesh.
static void mesh_subdivide(void) {
  uint8_t vx, vy;
  int k;

  if (mesh.nx < 2 || mesh.ny < 2) {
    vnx = vny = 0;
    return;
  }
  vnx = (uint8_t)((mesh.nx - 1) * MESH_SUBDIVISIONS + 1);
  vny = (uint8_t)((mesh.ny - 1) * MESH_SUBDIVISIONS + 1);
  vdx = (float)mesh.dx / (float)MESH_SUBDIVISIONS;
  vdy = (float)mesh.dy / (float)MESH_SUBDIVISIONS;
  for (vy = 0; vy < vny; vy++) {
    int iy = vy / MESH_SUBDIVISIONS;
    float ty = (float)(vy % MESH_SUBDIVISIONS) / (float)MESH_SUBDIVISIONS;

    if (iy > mesh.ny - 2) {
      iy = mesh.ny - 2;
      ty = 1.f;
    }
    for (vx = 0; vx < vnx; vx++) {
      int ix = vx / MESH_SUBDIVISIONS;
      float tx = (float)(vx % MESH_SUBDIVISIONS) / (float)MESH_SUBDIVISIONS;
      float row[4], z;

      if (ix > mesh.nx - 2) {
        ix = mesh.nx - 2;
        tx = 1.f;
      }
      for (k = 0; k < 4; k++)
        row[k] = catmull_rom(mesh_point(ix - 1, iy - 1 + k),
                             mesh_point(ix, iy - 1 + k),
                             mesh_point(ix + 1, iy - 1 + k),
                             mesh_point(ix + 2, iy - 1 + k), tx);
      z = catmull_rom(row[0], row[1], row[2], row[3], ty);
      if (z > 32767.f) z = 32767.f;
      if (z < -32767.f) z = -32767.f;
      vz[vy][vx] = (int16_t)floorf(z + 0.5f);
    }
  }
}

/// Mesh height at X, Y (um), bilinear between the virtual points,
/// constant beyond the outer points.
static float mesh_z(int32_t x, int32_t y) {
  float fx = (float)(x - mesh.x0) / vdx;
  float fy = (float)(y - mesh.y0) / vdy;
  float tx, ty, z0, z1;
  int ix, iy;

  if (fx < 0.0f) fx = 0.0f;
  if (fx > (float)(vnx - 1)) fx = (float)(vnx - 1);
  if (fy < 0.0f) fy = 0.0f;
  if (fy > (float)(vny - 1)) fy = (float)(vny - 1);
  ix = (int)fx;
  iy = (int)fy;
  if (ix > vnx - 2) ix = vnx - 2;
  if (iy > vny - 2) iy = vny - 2;
  tx = fx - (float)ix;
  ty = fy - (float)iy;

  z0 = (float)vz[iy][ix] * (1.0f - tx) + (float)vz[iy][ix + 1] * tx;
  z1 = (float)vz[iy + 1][ix] * (1.0f - tx) + (float)vz[iy + 1][ix + 1] * tx;
  return z0 * (1.0f - ty) + z1 * ty;
}

/// Fade factor for logical Z (um): 1 at the bed, 0 at the fade height.
static float fade_factor(int32_t z) {
  if (mesh.fade <= 0 || z <= 0)
    return 1.0f;
  if (z >= mesh.fade)
    return 0.0f;
  return 1.0f - (float)z / (float)mesh.fade;
}
#endif /* BED_LEVELING */

int32_t bed_level_offset(const axes_int32_t axis) {
  int32_t offset = 0;

  if (suspended)
    return 0;
  #ifdef BABYSTEPPING
    offset = babystep_offset();
  #endif
  #ifdef FIRMWARE_RETRACT
    offset += retract_hop_um;
  #endif
  #ifdef BED_LEVELING
    if (mesh.active && vnx) {
      float f = fade_factor(axis[Z]);

      if (f > 0.0f)
        offset += (int32_t)floorf(mesh_z(axis[X], axis[Y]) * f + 0.5f);
    }
  #else
    (void)axis;
  #endif
  return offset;
}

#endif /* BED_LEVELING || BABYSTEPPING || FIRMWARE_RETRACT */

void zcorr_sync_logical(void) {
  axes_int32_t p;
  int32_t motor;

  // Called after every change of the mesh.
  #ifdef BED_LEVELING
    mesh_subdivide();
  #endif

  if (steps_per_m_P[Z] == 0)
    return;                             // Startup, settings not applied yet.
  motor = steps_to_um(startpoint_steps.axis[Z], Z);

  memcpy(p, startpoint.axis, sizeof(p));
  // Two rounds: the fade factor depends on the result.
  p[Z] = motor;
  p[Z] = motor - bed_level_offset(p);
  p[Z] = motor - bed_level_offset(p);
  startpoint.axis[Z] = p[Z];
  if ( ! next_target.option_all_relative)
    next_target.target.axis[Z] = p[Z];
}

void zcorr_suspend(void) {
  if (suspended++ == 0)
    zcorr_sync_logical();
}

void zcorr_resume(void) {
  if (suspended && --suspended == 0)
    zcorr_sync_logical();
}

#ifdef BED_LEVELING

void bed_level_defaults(void) {
  memset(&mesh, 0, sizeof(mesh));
  mesh.fade = (int32_t)(LEVELING_FADE_HEIGHT * 1000.);
  zcorr_sync_logical();
}

uint8_t bed_level_active(void) {
  return mesh.active && mesh.nx;
}

uint8_t bed_level_set_active(uint8_t on) {
  mesh.active = (on && mesh.nx) ? 1 : 0;
  zcorr_sync_logical();
  return mesh.active;
}

void bed_level_set_fade(int32_t fade_um) {
  mesh.fade = fade_um > 0 ? fade_um : 0;
  zcorr_sync_logical();
}

void bed_level_new(int32_t x0, int32_t y0, int32_t dx, int32_t dy,
                   uint8_t nx, uint8_t ny) {
  int32_t fade = mesh.fade;

  memset(&mesh, 0, sizeof(mesh));
  mesh.fade = fade;
  if (nx < 2 || ny < 2 || nx > 7 || ny > 7 || dx <= 0 || dy <= 0) {
    zcorr_sync_logical();
    return;
  }
  mesh.nx = nx;
  mesh.ny = ny;
  mesh.x0 = x0;
  mesh.y0 = y0;
  mesh.dx = dx;
  mesh.dy = dy;
  zcorr_sync_logical();
}

uint8_t bed_level_set_point(uint8_t ix, uint8_t iy, int32_t z_um) {
  if (ix >= mesh.nx || iy >= mesh.ny)
    return 0;
  if (z_um > 32767)
    z_um = 32767;
  if (z_um < -32767)
    z_um = -32767;
  mesh.z[iy][ix] = (int16_t)z_um;
  zcorr_sync_logical();
  return 1;
}

void bed_level_report(void) {
  uint8_t ix, iy;

  serial_writestr(mesh.active ? "echo:Bed Leveling ON\n"
                               : "echo:Bed Leveling OFF\n");
  if (mesh.fade)
    sersendf_P(("echo:Fade Height %lq\n"), mesh.fade);
  else
    serial_writestr("echo:Fade Height off\n");
  if ( ! mesh.nx) {
    serial_writestr("echo:No mesh\n");
    return;
  }
  sersendf_P(("echo:Mesh %dx%d, X%lq..%lq Y%lq..%lq\n"), mesh.nx, mesh.ny,
             mesh.x0, mesh.x0 + mesh.dx * (mesh.nx - 1),
             mesh.y0, mesh.y0 + mesh.dy * (mesh.ny - 1));
  // Back row first, like looking at the bed from the front.
  for (iy = mesh.ny; iy-- > 0; ) {
    sersendf_P(("echo: %d"), iy);
    for (ix = 0; ix < mesh.nx; ix++) {
      int32_t z = mesh.z[iy][ix];

      serial_writestr(z < 0 ? " " : " +");
      sersendf_P(("%lq"), z);
    }
    serial_writechar('\n');
  }
}

/// Wait for queue space, then queue. \return 0 after a quickstop.
static uint8_t queue_segment(TARGET *t, uint8_t qs) {
  while (queue_full())
    clock_poll();
  if (emergency_quickstop_count() != qs)
    return 0;
  enqueue(t);
  return 1;
}

void bed_level_enqueue(TARGET *t) {
  float sx = (float)startpoint.axis[X], sy = (float)startpoint.axis[Y];
  float ex = (float)t->axis[X], ey = (float)t->axis[Y];
  float len = sqrtf((ex - sx) * (ex - sx) + (ey - sy) * (ey - sy));
  float tv[2 * VGRID], prev = 0.0f;
  uint8_t n = 0, i, k;
  uint8_t qs = emergency_quickstop_count();
  int32_t z0 = startpoint.axis[Z], dz = t->axis[Z] - z0;
  int32_t e0 = 0, de, e_done = 0;
  TARGET seg;

  if ( ! bed_level_active() || suspended || len < 1.0f) {
    queue_segment(t, qs);
    return;
  }

  // Crossings with the (virtual) grid lines, as a fraction of the move.
  for (i = 0; i < vnx; i++) {
    float g = (float)mesh.x0 + vdx * (float)i;

    if ((g > sx && g < ex) || (g < sx && g > ex))
      tv[n++] = (g - sx) / (ex - sx);
  }
  for (i = 0; i < vny; i++) {
    float g = (float)mesh.y0 + vdy * (float)i;

    if ((g > sy && g < ey) || (g < sy && g > ey))
      tv[n++] = (g - sy) / (ey - sy);
  }
  // Insertion sort, 2 * VGRID values at most.
  for (i = 1; i < n; i++) {
    float v = tv[i];

    for (k = i; k > 0 && tv[k - 1] > v; k--)
      tv[k] = tv[k - 1];
    tv[k] = v;
  }

  if (t->e_relative) {
    de = t->axis[E];
  }
  else {
    e0 = startpoint.axis[E];
    de = t->axis[E] - e0;
  }

  seg = *t;
  for (i = 0; i < n; i++) {
    float f = tv[i];
    int32_t e_here;

    // Skip pieces below 10 um, e.g. when crossing a grid point.
    if ((f - prev) * len < 10.0f || (1.0f - f) * len < 10.0f)
      continue;
    seg.axis[X] = (int32_t)floorf(sx + (ex - sx) * f + 0.5f);
    seg.axis[Y] = (int32_t)floorf(sy + (ey - sy) * f + 0.5f);
    seg.axis[Z] = z0 + (int32_t)floorf((float)dz * f + 0.5f);
    e_here = (int32_t)floorf((float)de * f + 0.5f);
    if (t->e_relative) {
      seg.axis[E] = e_here - e_done;
      e_done = e_here;
    }
    else {
      seg.axis[E] = e0 + e_here;
    }
    if ( ! queue_segment(&seg, qs))
      return;
    prev = f;
  }

  seg = *t;
  if (t->e_relative)
    seg.axis[E] = de - e_done;
  queue_segment(&seg, qs);
}

#endif /* BED_LEVELING */
