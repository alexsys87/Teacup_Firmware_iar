
/** \file
  \brief Mathematic algorithms for the digital differential analyser (DDA).
*/

#include "dda_maths.h"
#include "settings.h"

#include <stdlib.h>
#include <stdint.h>

/// Constants for um <=> steps conversions, see mul_div_k().
muldiv_k_t um_to_steps_k[AXIS_COUNT];
muldiv_k_t steps_to_um_k[AXIS_COUNT];
axes_uint32_t steps_per_m_P;

/*!
  Multiply-divide with 64 bit intermediate: multiplicand * multiplier /
  divisor, rounded like the old AVR bit loop (a remainder above divisor / 2
  rounds up, magnitude rounding for negative values). The product can't
  overflow, the result must fit into 32 bits.

  Cortex-M4: UMULL for the product, a hardware UDIV (2..12 cycles) when the
  product fits into 32 bits, else the library's 64 / 32 division. The AVR
  era algorithm looped over the bits of the multiplicand, several hundred
  cycles per call.
*/
TEACUP_HOT
static int32_t muldiv_u64(int32_t multiplicand, uint64_t multiplier,
                          uint32_t divisor) {
  uint32_t a = (multiplicand < 0) ? 0U - (uint32_t)multiplicand
                                  : (uint32_t)multiplicand;
  uint64_t p = (uint64_t)a * multiplier;
  uint32_t q, r;

  if ((p >> 32) == 0) {
    q = (uint32_t)p / divisor;
    r = (uint32_t)p - q * divisor;
  }
  else {
    uint64_t q64 = p / divisor;

    r = (uint32_t)(p - q64 * divisor);
    q = (uint32_t)q64;
  }
  if (r > divisor / 2)
    q++;

  return (multiplicand < 0) ? -(int32_t)q : (int32_t)q;
}

TEACUP_HOT
int32_t muldiv(int32_t multiplicand, uint32_t multiplier, uint32_t divisor) {
  return muldiv_u64(multiplicand, multiplier, divisor);
}

/*!
  Exact 2D distance, single precision FPU (VMUL, VMLA, VSQRT, about 30
  cycles). Replaces the integer approximation of the AVR era, which was off
  by -3 % .. +4 % depending on the direction, and so was the speed.

  Micrometers up to 4 km: float resolution is below 1 um up to 16 m.
*/
TEACUP_HOT
uint32_t distance_2d(uint32_t dx, uint32_t dy) {
  float x = (float)dx, y = (float)dy;

  return (uint32_t)(teacup_sqrtf(x * x + y * y) + 0.5f);
}

/// Exact 3D distance, see distance_2d().
TEACUP_HOT
uint32_t distance_3d(uint32_t dx, uint32_t dy, uint32_t dz) {
  float x = (float)dx, y = (float)dy, z = (float)dz;

  return (uint32_t)(teacup_sqrtf(x * x + y * y + z * z) + 0.5f);
}

/// Exact 3D distance as float, see distance_2d().
TEACUP_HOT
float distance_3d_f(uint32_t dx, uint32_t dy, uint32_t dz) {
  float x = (float)dx, y = (float)dy, z = (float)dz;

  return teacup_sqrtf(x * x + y * y + z * z);
}

/// k of mul_div_k(): floor(mult * 2^32 / div).
static void muldiv_k_set(muldiv_k_t *c, uint32_t mult, uint32_t div) {
  c->k = ((uint64_t)mult << 32) / div;
  c->mult = mult;
  c->div = div;
}

/*!
  Recalculate the per axis constants from the runtime settings (M92).
  Called by settings_apply().
*/
void dda_maths_update(void) {
  enum axis_e i;

  for (i = X; i < AXIS_COUNT; i++) {
    uint32_t spm = settings.steps_per_m[i];

    muldiv_k_set(&um_to_steps_k[i], spm, UM_PER_METER);
    muldiv_k_set(&steps_to_um_k[i], UM_PER_METER, spm);
    steps_per_m_P[i] = spm;
  }
}
