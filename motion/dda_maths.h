#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config_wrapper.h"
#include "dda.h"

/// Rounded multiplicand * (qn + rn / divisor), quotient and remainder of
/// the multiplier precalculated elsewhere. 64 bit intermediate.
int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                 uint32_t divisor);

/// Rounded multiplicand * multiplier / divisor, 64 bit intermediate.
int32_t muldiv(int32_t multiplicand, uint32_t multiplier, uint32_t divisor);

/*!
  Micrometer distance <=> motor step distance conversions.
*/

#define UM_PER_METER (1000000UL)

extern axes_uint32_t  axis_qn_P;
extern axes_uint32_t  axis_qr_P;

TEACUP_INLINE int32_t um_to_steps(int32_t distance, enum axis_e a) {
  return muldivQR(distance, (axis_qn_P[a]),
                  (axis_qr_P[a]), UM_PER_METER);
}

extern axes_uint32_t steps_per_m_P;

TEACUP_INLINE int32_t steps_to_um(int32_t steps, enum axis_e a) {
  return muldiv(steps, UM_PER_METER, (steps_per_m_P[a]));
}

/// Exact 2D distance (FPU).
uint32_t distance_2d(uint32_t dx, uint32_t dy);

/// Exact 3D distance (FPU).
uint32_t distance_3d(uint32_t dx, uint32_t dy, uint32_t dz);

// integer square root algorithm
uint16_t int_sqrt(uint32_t a);
#if __FPU_PRESENT
uint_fast16_t int_f_sqrt(uint32_t a);
#endif

// integer inverse square root, 12bits precision
uint16_t int_inv_sqrt(uint16_t a);

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
uint8_t msbloc (uint32_t v);

/// Recalculate constants from the runtime settings.
void dda_maths_update(void);

#endif	/* _DDA_MATHS_H */
