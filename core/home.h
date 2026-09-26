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

enum axis_endstop_e {
  X_MIN_ENDSTOP = 0x01,
  X_MAX_ENDSTOP = 0x02,
  Y_MIN_ENDSTOP = 0x04,
  Y_MAX_ENDSTOP = 0x08,
  Z_MIN_ENDSTOP = 0x10,
  Z_MAX_ENDSTOP = 0x20,
};

void home_none(void);
void home_x_negative(void);
void home_x_positive(void);
void home_y_negative(void);
void home_y_positive(void);
void home_z_negative(void);
void home_z_positive(void);

#endif /* _HOME_H */
