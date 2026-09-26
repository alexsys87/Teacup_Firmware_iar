#ifndef DDA_LOOKAHEAD_H_
#define DDA_LOOKAHEAD_H_

#include <stdint.h>
#include "config_wrapper.h"
#include "dda.h"
#include "debug.h"

#ifdef LOOKAHEAD

// Sanity: make sure the defines are in place
#if ! defined MAX_JERK_X || ! defined MAX_JERK_Y || \
    ! defined MAX_JERK_Z || ! defined MAX_JERK_E
#error LOOKAHEAD needs MAX_JERK_X, MAX_JERK_Y, MAX_JERK_Z and MAX_JERK_E.
#endif

// Move joining needs the same steps per mm on X and Y. This is checked at
// runtime (M92), moves aren't joined otherwise.
#if STEPS_PER_M_X != STEPS_PER_M_Y
  #warning Look-ahead is inactive with different steps per mm on X and Y.
#endif

#define MAX(a,b)  (((a)>(b))?(a):(b))
#define MIN(a,b)  (((a)<(b))?(a):(b))

void dda_find_crossing_speed(DDA *prev, DDA *current);
void dda_join_moves(DDA *prev, DDA *current);

#endif /* LOOKAHEAD */
#endif /* DDA_LOOKAHEAD_H_ */
