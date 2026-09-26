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
  XY skew factor (M852 I), tangent of the angle by which the Y axis leans
  towards X. Motor X = X - Y * factor, like Marlin's SKEW_CORRECTION.
*/
float skew_xy_factor;

void kinematics_update(void) {
  skew_xy_factor = (float)settings.skew_xy * 1e-6f;
}

/// X with the skew correction, um.
static int32_t skew_x(const axes_int32_t um) {
  if (skew_xy_factor == 0.f)
    return um[X];
  return um[X] - (int32_t)lrintf((float)um[Y] * skew_xy_factor);
}
#else
  #define skew_x(um) ((um)[X])

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
  steps[Y] = um_to_steps(um[Y], Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps) {
  int32_t x = skew_x(um);

  steps[X] = um_to_steps(x + um[Y], X);
  steps[Y] = um_to_steps(x - um[Y], Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void delta_to_axes_cartesian(axes_int32_t delta) {
  #ifdef SKEW_CORRECTION
    // Motor X delta back to X: X = motor X + Y * factor.
    if (skew_xy_factor != 0.f)
      delta[X] += (int32_t)lrintf((float)delta[Y] * skew_xy_factor);
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
    if (skew_xy_factor != 0.f)
      delta[X] += (int32_t)lrintf((float)delta[Y] * skew_xy_factor);
  #endif
}
