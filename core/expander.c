/** \file
  \brief PCF8574 I/O expander, see expander.h.

  The PCF8574 has quasi-bidirectional pins: a pin written high is a weak
  pull-up (about 100 uA) and reads as input, e.g. a button to GND; a pin
  written low sinks current. Outputs (fans) and inputs (buttons) share the
  8 pins, inputs stay high in the output byte.

  Writes and reads go through the I2C queue together with the display.
  Neither may start while the display has a transmission open.
*/

#include "expander.h"

#ifdef PCF8574_ADDRESS

#include "i2c.h"

/// Output byte: unused outputs stay high (inputs for buttons, encoder).
static uint8_t exp_out = 0xFF;
static uint8_t exp_dirty = 1;
static uint8_t exp_in = 0xFF;

void expander_init(void) {
  i2c_init(0);
}

void expander_set(uint8_t bit, uint8_t on) {
  uint8_t v = on ? (uint8_t)(exp_out | (1U << bit))
                 : (uint8_t)(exp_out & ~(1U << bit));

  if (v != exp_out) {
    exp_out = v;
    exp_dirty = 1;
  }
}

void expander_tick(void) {
  #ifdef EXPANDER_INPUTS
    uint8_t v, r;

    r = i2c_read_result(&v);
    if (r == I2C_READ_DONE || r == I2C_READ_FAILED)
      exp_in = v;                     // 0xFF after a failed read.
  #endif

  // Wait while another device (display) has a transmission open.
  if (i2c_tx_open())
    return;
  if (exp_dirty) {
    i2c_write_to((uint8_t)(PCF8574_ADDRESS << 1), exp_out, 1);
    exp_dirty = 0;
  }
  #ifdef EXPANDER_INPUTS
    // The next read, its result is taken at the next tick.
    if (r != I2C_READ_PENDING)
      i2c_read_from((uint8_t)(PCF8574_ADDRESS << 1));
  #endif
}

uint8_t expander_inputs(void) {
  return exp_in;
}

#endif /* PCF8574_ADDRESS */
