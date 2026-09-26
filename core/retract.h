/** \file
  \brief Firmware retract (G10 / G11, M207 / M208), like Marlin's
  FWRETRACT.
*/

#ifndef _RETRACT_H
#define _RETRACT_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef FIRMWARE_RETRACT

/**
  Z lift of the retract (M207 Z) while retracted, um. Part of the Z
  correction (bed_level_offset()): moves keep the lifted height until G11.
*/
extern int32_t retract_hop_um;

/// G10: retract E by M207 S at M207 F, then lift Z by M207 Z.
void retract_do(void);

/// G11: lower Z, prime E by M207 S + M208 S at M208 F.
void recover_do(void);

/// Whether retracted (G10 without G11 yet).
uint8_t retract_active(void);

/// Forget the retract state (homing, new print). The motors don't move.
void retract_reset(void);

#endif /* FIRMWARE_RETRACT */

#endif /* _RETRACT_H */
