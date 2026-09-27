/** \file
  \brief PCF8574 I/O expander on the I2C bus: outputs (fans) and inputs
  (buttons).
*/

#ifndef _EXPANDER_H
#define _EXPANDER_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef PCF8574_ADDRESS

/// Start the I2C bus.
void expander_init(void);

/// Set an output of the PCF8574 (sent within 10 ms).
void expander_set(uint8_t bit, uint8_t on);

/// Every 10 ms: send pending output changes, read the inputs.
void expander_tick(void);

/**
  Pin states of the last read (inputs are the pins written high), 0xFF
  before the first read and without an expander (not acknowledged).
*/
uint8_t expander_inputs(void);

#endif /* PCF8574_ADDRESS */

#endif /* _EXPANDER_H */
