/** \file
  \brief Input shaping (M593, ZV / MZV) and S-curve smoothing of X and Y.
*/

#ifndef _INPUT_SHAPING_H
#define _INPUT_SHAPING_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef INPUT_SHAPING

#include "arch.h"

/**
  Axis 0 (X) or 1 (Y) is shaped: dda_step() hands its steps to
  shaper_step() instead of stepping it directly. Written by the shaper
  while the axis is idle.
*/
extern volatile uint8_t shaper_on[2];

/// Steps which found the history full and weren't shaped (M593 reports).
extern volatile uint32_t shaper_overflows;

/**
  One step of X (axis 0) or Y (axis 1) from the Bresenham algorithm, in
  the step interrupt. The shaper emits it later, see shaper_service().
*/
void shaper_step(uint8_t axis, uint8_t forward);

/**
  Shaper part of the auxiliary step generator (hal/timer.c).
  \param now step timer count.
  \return CPU ticks until it wants to run again, or AUX_NONE.
*/
uint32_t shaper_service(uint32_t now);

/**
  A move not shaped (endstop checks) sets the direction pins itself: the
  shaper has to set them again before its next step.
*/
void shaper_dir_unknown(void);

/// Whether X or Y still have steps to emit (queue_wait()).
uint8_t shaper_busy(void);

/**
  Take the shaping settings (settings.is_*, settings.s_curve_us). They
  apply right away while X and Y are idle, else as soon as they are.
*/
void shaper_configure(void);

#endif /* INPUT_SHAPING */
#endif /* _INPUT_SHAPING_H */
