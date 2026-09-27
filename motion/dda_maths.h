#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config_wrapper.h"
#include "dda.h"

/// Rounded multiplicand * multiplier / divisor, 64 bit intermediate.
int32_t muldiv(int32_t multiplicand, uint32_t multiplier, uint32_t divisor);

/*!
  Micrometer distance <=> motor step distance conversions.
*/

#define UM_PER_METER (1000000UL)

/**
  Precalculated for m * mult / div without a division, see mul_div_k():
  k = floor(mult * 2^32 / div).
*/
typedef struct {
  uint64_t k;
  uint32_t mult, div;
} muldiv_k_t;

extern muldiv_k_t um_to_steps_k[AXIS_COUNT];
extern muldiv_k_t steps_to_um_k[AXIS_COUNT];

/**
  Rounded m * mult / div, exact (the same as muldiv()), without a division:
  q = floor(m * k / 2^32) is floor(m * mult / div) or one less, one
  correction step with the exact remainder settles it. Four UMULL, some 20
  cycles, where a 64 / 32 division is a library call of 100 and more.
  A remainder above div / 2 rounds up, magnitude rounding for negative
  values, like muldiv().
*/
TEACUP_INLINE int32_t mul_div_k(int32_t m, const muldiv_k_t *c) {
  uint32_t a = (m < 0) ? 0U - (uint32_t)m : (uint32_t)m;
  uint64_t q = (uint64_t)a * (uint32_t)(c->k >> 32) +
               (((uint64_t)a * (uint32_t)c->k) >> 32);
  uint64_t r = (uint64_t)a * c->mult - q * c->div;

  if (r >= c->div) {
    q++;
    r -= c->div;
  }
  if (r > c->div / 2)
    q++;
  return (m < 0) ? -(int32_t)q : (int32_t)q;
}

TEACUP_INLINE int32_t um_to_steps(int32_t distance, enum axis_e a) {
  return mul_div_k(distance, &um_to_steps_k[a]);
}

extern axes_uint32_t steps_per_m_P;

TEACUP_INLINE int32_t steps_to_um(int32_t steps, enum axis_e a) {
  return mul_div_k(steps, &steps_to_um_k[a]);
}

/// Exact 2D distance (FPU).
uint32_t distance_2d(uint32_t dx, uint32_t dy);

/// Exact 3D distance (FPU).
uint32_t distance_3d(uint32_t dx, uint32_t dy, uint32_t dz);

/// Exact 3D distance as float (FPU), for the planner.
float distance_3d_f(uint32_t dx, uint32_t dy, uint32_t dz);

/// Recalculate constants from the runtime settings.
void dda_maths_update(void);

#endif	/* _DDA_MATHS_H */
