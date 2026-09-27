#ifndef _HOME_H
#define _HOME_H

#include <stdint.h>
#include "dda.h"

void home(void);

/// Axes homed since the steppers were last enabled: HOMED_X | HOMED_Y | HOMED_Z.
extern uint8_t axes_homed;
#define HOMED_X   0x01
#define HOMED_Y   0x02
#define HOMED_Z   0x04
#define HOMED_XYZ (HOMED_X | HOMED_Y | HOMED_Z)

/// Home offsets (M206), micrometers, X/Y/Z. Added to the home position.
extern int32_t home_offset[3];

/**
  Set the home offset of an axis (M206, M428). The current position is
  shifted accordingly, like Marlin does. Waits for the queue to empty.
*/
void home_set_offset(enum axis_e n, int32_t offset_um);

/**
  Offset of the active tool (M218, 0 for T0), micrometers, X/Y/Z: the
  nozzle of T1 sits this far from the nozzle of T0. Coordinates are
  those of the active nozzle, the home position and the soft limits move
  along.
*/
extern int32_t tool_shift[3];

/**
  Set the offset of the active tool (T0 / T1, M218). The current position
  is shifted, the next move puts the new nozzle at the programmed place.
  Waits for the queue to empty.
*/
void home_set_tool_shift(enum axis_e n, int32_t offset_um);

enum axis_endstop_e {
  X_MIN_ENDSTOP = 0x01,
  X_MAX_ENDSTOP = 0x02,
  Y_MIN_ENDSTOP = 0x04,
  Y_MAX_ENDSTOP = 0x08,
  Z_MIN_ENDSTOP = 0x10,
  Z_MAX_ENDSTOP = 0x20,
};

/**
  Z_HOMING_HEIGHT: lift Z to this height (mm) before homing X or Y, so
  the nozzle or a BLTouch pin doesn't drag over the bed or clips. Never
  lowers Z.
*/
void home_lift_z(void);

void home_none(void);
void home_x_negative(void);
void home_x_positive(void);
void home_y_negative(void);
void home_y_positive(void);
void home_z_negative(void);
void home_z_positive(void);

#endif /* _HOME_H */
