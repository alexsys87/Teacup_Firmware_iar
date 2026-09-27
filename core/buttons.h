/** \file
  \brief Buttons on the PCF8574 I/O expander: debounced, with auto-repeat.
*/

#ifndef _BUTTONS_H
#define _BUTTONS_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef BUTTONS

/// Button events. BUTTON_FAST is added to repeats after a long hold.
enum {
  BUTTON_NONE = 0,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_OK,
  BUTTON_BACK,
  BUTTON_FAST = 0x80
};

/// Every 10 ms, after expander_tick().
void buttons_tick(void);

/// Next event, BUTTON_NONE if there is none.
uint8_t buttons_get(void);

/// Buttons currently held (bit 0 up, 1 down, 2 OK, 3 back), for M119.
uint8_t buttons_held(void);

#endif /* BUTTONS */

#endif /* _BUTTONS_H */
