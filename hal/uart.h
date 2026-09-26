/** \file
  \brief Host UART (USART1, USART2 or USART6), DMA or interrupt driven.

  Low level driver of the UART host port. Firmware code doesn't call it
  directly, it uses serial.h, which sends to and receives from all host
  ports (UART and USB CDC).
*/

#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include "config_wrapper.h"

/// Initialise the UART, pins and DMA.
void uart_init(void);

/// Number of characters waiting in the receive buffer.
uint16_t uart_rxchars(void);

/// Read one character. Returns 0 if nothing is available.
uint8_t uart_popchar(void);

/// Send one character. Waits if the transmit buffer is full.
void uart_writechar(uint8_t data);

/// Wait until everything is sent. Works with interrupts disabled.
void uart_flush(void);

/// Polled receive for use with interrupts disabled. -1 = no character.
int16_t uart_rx_poll(void);

#endif /* _UART_H */
