/** \file
  \brief Z correction: bed leveling mesh and Z offset, see bed_leveling.h.

  Model: motor Z = logical Z + correction(X, Y, Z). The planner converts
  logical targets to motor steps with the correction applied
  (axes_um_to_steps_*() in dda_kinematics.c), so every move follows the
  bed. startpoint_steps always holds the true motor position (end of the
  queue); whenever the correction changes without a move (leveling on/off,
  new mesh, fade height, homing, probing), the logical Z is recalculated
  from it instead (zcorr_sync_logical()), the motors never jump.

  Mesh: bilinear interpolation between the grid points, constant beyond
  the outer points (no extrapolation). Faded out linearly between Z = 0 and
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

/// Nesting counter of zcorr_suspend().
static uint8_t suspended = 0;

#ifdef BED_LEVELING
mesh_t mesh;
#endif

#if defined BED_LEVELING || defined BABYSTEPPING

#ifdef BED_LEVELING
/// Bilinear mesh height at X, Y (um), constant beyond the outer points.
static float mesh_z(int32_t x, int32_t y) {
  float fx = (float)(x - mesh.x0) / (float)mesh.dx;
  float fy = (float)(y - mesh.y0) / (float)mesh.dy;
  float tx, ty, z0, z1;
  int ix, iy;

  if (fx < 0.0f) fx = 0.0f;
  if (fx > (float)(mesh.nx - 1)) fx = (float)(mesh.nx - 1);
  if (fy < 0.0f) fy = 0.0f;
  if (fy > (float)(mesh.ny - 1)) fy = (float)(mesh.ny - 1);
  ix = (int)fx;
  iy = (int)fy;
  if (ix > mesh.nx - 2) ix = mesh.nx - 2;
  if (iy > mesh.ny - 2) iy = mesh.ny - 2;
  tx = fx - (float)ix;
  ty = fy - (float)iy;

  z0 = (float)mesh.z[iy][ix] * (1.0f - tx) + (float)mesh.z[iy][ix + 1] * tx;
  z1 = (float)mesh.z[iy + 1][ix] * (1.0f - tx) + (float)mesh.z[iy + 1][ix + 1] * tx;
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
  #ifdef BED_LEVELING
    if (mesh.active && mesh.nx) {
      float f = fade_factor(axis[Z]);

      if (f > 0.0f)
        offset += (int32_t)floorf(mesh_z(axis[X], axis[Y]) * f + 0.5f);
    }
  #else
    (void)axis;
  #endif
  return offset;
}

#endif /* BED_LEVELING || BABYSTEPPING */

void zcorr_sync_logical(void) {
  axes_int32_t p;
  int32_t motor;

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
  float tv[16], prev = 0.0f;
  uint8_t n = 0, i, k;
  uint8_t qs = emergency_quickstop_count();
  int32_t z0 = startpoint.axis[Z], dz = t->axis[Z] - z0;
  int32_t e0 = 0, de, e_done = 0;
  TARGET seg;

  if ( ! bed_level_active() || suspended || len < 1.0f) {
    queue_segment(t, qs);
    return;
  }

  // Crossings with the grid lines, as a fraction of the move.
  for (i = 0; i < mesh.nx; i++) {
    float g = (float)(mesh.x0 + mesh.dx * i);

    if ((g > sx && g < ex) || (g < sx && g > ex))
      tv[n++] = (g - sx) / (ex - sx);
  }
  for (i = 0; i < mesh.ny; i++) {
    float g = (float)(mesh.y0 + mesh.dy * i);

    if ((g > sy && g < ey) || (g < sy && g > ey))
      tv[n++] = (g - sy) / (ey - sy);
  }
  // Insertion sort, 14 values at most.
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
