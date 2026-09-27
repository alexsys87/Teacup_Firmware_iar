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

#ifdef NO_SERIAL_UART

/*
  NO_SERIAL_UART: no host UART, USB CDC only. Frees the USART and its
  pins (e.g. for TMC_UART).
*/
#ifndef USB_CDC
  #error NO_SERIAL_UART needs USB_CDC as host connection.
#endif
static inline void uart_init(void) { }
static inline uint16_t uart_rxchars(void) { return 0; }
static inline uint8_t uart_popchar(void) { return 0; }
static inline void uart_writechar(uint8_t data) { (void)data; }
static inline void uart_flush(void) { }
static inline int16_t uart_rx_poll(void) { return -1; }

#else

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

#endif /* NO_SERIAL_UART */

#endif /* _UART_H */
