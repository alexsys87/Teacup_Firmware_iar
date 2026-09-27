/** \file
  \brief Single wire UART to TMC2208 / TMC2209 stepper drivers (TMC_UART).

  Half duplex USART (HDSEL) on one pin, TMC_UART_TX_PIN, open drain with
  pull-up; connect it to PDN_UART of the drivers (TMC2209: all drivers on
  one wire, addresses by MS1 / MS2). Polled, used from the main loop only.
*/

#ifndef _TMC_UART_H
#define _TMC_UART_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef TMC_UART

/// Clock, pin and USART setup.
void tmc_uart_init(void);

/// Drop all received characters (echo of our own transmission).
void tmc_uart_flush_rx(void);

/// Send n characters, return when the last one left the shift register.
void tmc_uart_send(const uint8_t *data, uint8_t n);

/// Received character or -1.
int16_t tmc_uart_getc(void);

#endif /* TMC_UART */

#endif /* _TMC_UART_H */
