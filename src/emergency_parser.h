/** \file
  \brief Emergency parser: M112, M108 and M410 act immediately.
*/

#ifndef _EMERGENCY_PARSER_H
#define _EMERGENCY_PARSER_H

#include <stdint.h>

/**
  Feed every received character, called from the receive interrupts of the
  host ports (UART, USB). 'port' is SERIAL_UART_PORT or SERIAL_USB_PORT,
  every port has its own parser state.
  These commands take effect right away, even when the command queue is
  full or the firmware waits for something:

    M112  emergency stop, printer_kill()
    M108  cancel waiting for temperatures
    M410  quickstop: stop all moves now, keep the position
    M876 S<n>  answer to a host prompt: continue after M600, like M108
*/
void emergency_parser_char(uint8_t port, uint8_t c);

/// Finish a quickstop in main loop context. Called from clock_poll().
void emergency_poll(void);

/// Quickstop like M410 (menu "Stop print"), finished by emergency_poll().
void emergency_quickstop(void);

/// Counts finished quickstops (wraps). Long operations compare it before
/// and after waiting to notice an M410 in between.
uint8_t emergency_quickstop_count(void);

#endif /* _EMERGENCY_PARSER_H */
