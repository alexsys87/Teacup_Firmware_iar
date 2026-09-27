#include "dda_kinematics.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>
#include <math.h>

#include "dda_maths.h"
#include "bed_leveling.h"
#include "settings.h"

#ifdef SKEW_CORRECTION
/**
  Skew factors (M852 I J K), tangents of the angles by which the axes lean,
  like Marlin's SKEW_CORRECTION:
    motor Y = Y - Z * yz
    motor X = X - motor Y * xy - Z * xz
  Linear, so straight moves stay straight.
*/
float skew_xy_factor, skew_xz_factor, skew_yz_factor;

void kinematics_update(void) {
  skew_xy_factor = (float)settings.skew_xy * 1e-6f;
  skew_xz_factor = (float)settings.skew_xz * 1e-6f;
  skew_yz_factor = (float)settings.skew_yz * 1e-6f;
}

/// Y with the skew correction, um.
static int32_t skew_y(const axes_int32_t um) {
  if (skew_yz_factor == 0.f)
    return um[Y];
  return um[Y] - (int32_t)lrintf((float)um[Z] * skew_yz_factor);
}

/// X with the skew correction, um.
static int32_t skew_x(const axes_int32_t um) {
  if (skew_xy_factor == 0.f && skew_xz_factor == 0.f)
    return um[X];
  return um[X] - (int32_t)lrintf((float)skew_y(um) * skew_xy_factor +
                                 (float)um[Z] * skew_xz_factor);
}

/// Motor deltas of X and Y back to G-code deltas.
static void unskew(axes_int32_t delta) {
  int32_t y_motor = delta[Y];

  if (skew_yz_factor != 0.f)
    delta[Y] += (int32_t)lrintf((float)delta[Z] * skew_yz_factor);
  if (skew_xy_factor != 0.f || skew_xz_factor != 0.f)
    delta[X] += (int32_t)lrintf((float)y_motor * skew_xy_factor +
                                (float)delta[Z] * skew_xz_factor);
}
#else
  #define skew_x(um) ((um)[X])
  #define skew_y(um) ((um)[Y])

void kinematics_update(void) {
}
#endif

void
carthesian_to_carthesian(const TARGET *startpoint, const TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  delta_um[X] = (uint32_t)labs(target->axis[X] - startpoint->axis[X]);
  delta_um[Y] = (uint32_t)labs(target->axis[Y] - startpoint->axis[Y]);
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);

  axes_um_to_steps_cartesian(target->axis, steps);
}

void
carthesian_to_corexy(const TARGET *startpoint, const TARGET *target,
                     axes_uint32_t delta_um, axes_int32_t steps) {

  delta_um[X] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) +
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Y] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) -
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);
  axes_um_to_steps_corexy(target->axis, steps);
}

void axes_um_to_steps_cartesian(const axes_int32_t um, axes_int32_t steps) {
  steps[X] = um_to_steps(skew_x(um), X);
  steps[Y] = um_to_steps(skew_y(um), Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps) {
  int32_t x = skew_x(um), y = skew_y(um);

  steps[X] = um_to_steps(x + y, X);
  steps[Y] = um_to_steps(x - y, Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void delta_to_axes_cartesian(axes_int32_t delta) {
  #ifdef SKEW_CORRECTION
    unskew(delta);
  #else
    (void)delta;
  #endif
}

void delta_to_axes_corexy(axes_int32_t delta) {
  // recalculate only dedicated axes
  int32_t x_axis, y_axis;
  x_axis = (delta[X] + delta[Y]) / 2;
  y_axis = (delta[X] - delta[Y]) / 2;
  delta[X] = x_axis;
  delta[Y] = y_axis;
  #ifdef SKEW_CORRECTION
    unskew(delta);
  #endif
}
