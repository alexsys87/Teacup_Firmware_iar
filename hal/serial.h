/** \file
  \brief Serial subsystem: host communication over all host ports.

  Host ports:
    SERIAL_UART_PORT  the UART (SERIAL_UART, always present)
    SERIAL_USB_PORT   USB CDC ACM virtual COM port (USB_CDC, optional)

  Receiving is done per port (serial_port_rxchars(), serial_port_popchar()),
  so lines from different ports never get mixed. Sending goes to all ports
  selected by serial_set_output(). By default that's all ports; while a
  host command executes, gcode_queue.c selects the port the command came
  from, so the answer goes back to the host which sent the command (like
  Marlin's MULTI_SERIAL). Messages outside of commands (start, errors,
  kill, SD printing) go to all ports.
*/

#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdint.h>
#include "config_wrapper.h"

/// Port numbers, used as index.
#define SERIAL_UART_PORT      0
#ifdef USB_CDC
  #define SERIAL_USB_PORT     1
  #define SERIAL_NUM_PORTS    2
#else
  #define SERIAL_NUM_PORTS    1
#endif

/// Output selection masks for serial_set_output().
#define SERIAL_MASK(port)     ((uint8_t)(1U << (port)))
#define SERIAL_MASK_ALL       ((uint8_t)((1U << SERIAL_NUM_PORTS) - 1U))

/// Initialise all host ports.
void serial_init(void);

/// Number of characters waiting in the receive buffer of a port.
uint16_t serial_port_rxchars(uint8_t port);

/// Read one character from a port. Returns 0 if nothing is available.
uint8_t serial_port_popchar(uint8_t port);

/**
  Select the ports serial_writechar() sends to, SERIAL_MASK(port) or
  SERIAL_MASK_ALL. Returns the previous selection, for restoring it.
*/
uint8_t serial_set_output(uint8_t mask);

/// Send one character to the selected ports.
void serial_writechar(uint8_t data);

/// Send a zero terminated string to the selected ports.
void serial_writestr(char const *data);

/// Legacy name from AVR times (strings in flash), same as serial_writestr().
#define serial_writestr_P(s) serial_writestr(s)

/**
  Wait until everything is sent on all ports. Works with interrupts
  disabled. USB gives up after a timeout if the host doesn't read.
*/
void serial_flush(void);

/**
  Polled receive from all ports, for use with interrupts disabled (halted
  state after printer_kill()). Returns the character or -1 if there's none,
  *port tells where it came from.
*/
int16_t serial_rx_poll(uint8_t *port);

#endif /* _SERIAL_H */
