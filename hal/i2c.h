/** \file
  \brief I2C master, transmit only, interrupt driven with a send queue.

  Used for displays. Bytes are queued with i2c_write(); a byte written with
  last_byte = 1 ends the current transmission (STOP). The next byte after
  that starts a new transmission to the same address.
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

#endif /* I2C */

#endif /* _I2C_H */
