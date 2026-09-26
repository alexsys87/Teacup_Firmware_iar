
/** \file
  \brief Mathematic algorithms for the digital differential analyser (DDA).
*/

#include "dda_maths.h"
#include "settings.h"

#include <stdlib.h>
#include <stdint.h>

/*!
  Pre-calculated constant values for axis um <=> steps conversions.

  These should be calculated at run-time once in dda_init() if the
  STEPS_PER_M_* constants are replaced with run-time options (M92).
*/
axes_uint32_t axis_qn_P;
axes_uint32_t axis_qr_P;
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

/*!
  multiplicand * (qn + rn / divisor), rounded. The same as
  muldiv(multiplicand, qn * divisor + rn, divisor), for precalculated
  quotients and remainders (um <=> steps, bed leveling).
*/
TEACUP_HOT
int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                 uint32_t divisor) {
  return muldiv_u64(multiplicand, (uint64_t)qn * divisor + rn, divisor);
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

#if __FPU_PRESENT
/// Square root with the FPU (one VSQRT.F32 instruction).
TEACUP_HOT
uint_fast16_t int_f_sqrt(uint32_t a) {
  return (uint_fast16_t)teacup_sqrtf((float)a);
}
#endif /* __FPU_PRESENT */
/*!
  integer square root algorithm
  \param a find square root of this number
  \return sqrt(a - 1) < returnvalue <= sqrt(a)

  This is a binary search but it uses only the minimum required bits for
  each step.
*/
TEACUP_HOT
uint16_t int_sqrt(uint32_t a) {
  uint16_t b = a >> 16;
  uint8_t c = b >> 8;
  uint16_t x = 0;
  uint8_t z = 0;
  uint16_t i;
  uint8_t j;

  for (j = 0x8; j; j >>= 1) {
    uint8_t y2;

    z |= j;
    y2 = z * z;
    if (y2 > c)
      z ^= j;
  }

  x = z << 4;
  for(i = 0x8; i; i >>= 1) {
    uint16_t y2;

    x |= i;
    y2 = x * x;
    if (y2 > b)
      x ^= i;
  }

  x <<= 8;
  for(i = 0x80; i; i >>= 1) {
    uint32_t y2;

    x |= i;
    y2 = (uint32_t)x * x;
    if (y2 > a)
      x ^= i;
  }

  return x;
}

/*!
  integer inverse square root algorithm
  \param a find the inverse of the square root of this number
  \return 0x1000 / sqrt(a) - 1 < returnvalue <= 0x1000 / sqrt(a)

  This is a binary search but it uses only the minimum required bits for each step.
*/
TEACUP_HOT
uint16_t int_inv_sqrt(uint16_t a) {
  /// 16bits inverse (much faster than doing a full 32bits inverse)
  /// the 0xFFFFU instead of 0x10000UL hack allows using 16bits and 8bits
  /// variable for the first 8 steps without overflowing and it seems to
  /// give better results for the ramping equation too :)
  uint8_t z = 0, i;
  uint16_t x, j;
  uint32_t q = ((uint32_t)(0xFFFFU / a)) << 8;

  for (i = 0x80; i; i >>= 1) {
    uint16_t y;

    z |= i;
    y = (uint16_t)z * z;
    if (y > (q >> 8))
      z ^= i;
  }

  x = z << 4;
  for (j = 0x8; j; j >>= 1) {
    uint32_t y;

    x |= j;
    y = (uint32_t)x * x;
    if (y > q)
      x ^= j;
  }

  return x;
}

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
/*! crude logarithm algorithm
  \param v value to find \f$log_2\f$ of
  \return floor(log(v) / log(2))
*/
TEACUP_HOT
uint8_t msbloc (uint32_t v) {
  uint8_t i;
  uint32_t c;
  for (i = 31, c = 0x80000000; i; i--) {
    if (v & c)
      return i;
    c >>= 1;
  }
  return 0;
}

/*!
  Recalculate the per axis constants from the runtime settings (M92).
  Called by settings_apply().
*/
void dda_maths_update(void) {
  enum axis_e i;

  for (i = X; i < AXIS_COUNT; i++) {
    uint32_t spm = settings.steps_per_m[i];

    axis_qn_P[i] = spm / UM_PER_METER;
    axis_qr_P[i] = spm % UM_PER_METER;
    steps_per_m_P[i] = spm;
  }
}
