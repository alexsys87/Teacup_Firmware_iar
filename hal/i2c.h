/** \file
  \brief I2C master, interrupt driven with a send queue.

  Used for displays and the I/O expander. Bytes are queued with
  i2c_write(); a byte written with last_byte = 1 ends the current
  transmission (STOP). The next byte after that starts a new transmission
  to the same address. i2c_read_from() queues a read of one byte (buttons
  on a PCF8574), i2c_read_result() picks it up.
*/

#ifndef _I2C_H
#define _I2C_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef I2C

/// Initialise the bus. address: 8 bit form, e.g. 0x78 for SSD1306, 0 = none.
/// Calling it again only sets the address.
void i2c_init(uint8_t address);

/// Whether a transmission is ongoing.
uint8_t i2c_busy(void);

/// Queue a byte for the address of i2c_init(). Waits while the queue is full.
void i2c_write(uint8_t data, uint8_t last_byte);

/// Queue a byte for another device (8 bit address form).
void i2c_write_to(uint8_t address, uint8_t data, uint8_t last_byte);

/**
  Whether a transmission was started with i2c_write() / i2c_write_to()
  and not ended yet (last_byte). Another device must wait for its turn.
*/
uint8_t i2c_tx_open(void);

/**
  Queue a read of one byte from a device (8 bit address form). Only one
  read at a time, and not while a transmission is open (i2c_tx_open()).
  \return 1 if queued.
*/
uint8_t i2c_read_from(uint8_t address);

/// Result of i2c_read_from().
enum {
  I2C_READ_NONE = 0,    ///< No read queued (or the result was taken).
  I2C_READ_PENDING,     ///< Not done yet.
  I2C_READ_DONE,        ///< Done, *value set.
  I2C_READ_FAILED       ///< Not acknowledged (no device) or bus error.
};

/// Take the result of i2c_read_from(); DONE and FAILED only once.
uint8_t i2c_read_result(uint8_t *value);

#endif /* I2C */

#endif /* _I2C_H */
