/** \file
  \brief Command queue: buffered, validated G-code lines from the host.
*/

#ifndef _GCODE_QUEUE_H
#define _GCODE_QUEUE_H

#include <stdint.h>

/**
  Read characters from the serial receive buffer, assemble lines, check
  line numbers and checksums and queue valid lines. Answers bad lines with
  "Error:..." and "Resend: N". Called from clock_poll(), so lines are
  received and checked also while a command blocks (G4, M116, G28, ...).
*/
void gcode_queue_read_serial(void);

/**
  Execute the oldest queued line and acknowledge it with "ok". Call only
  when the movement queue has room.
  \return 1 if a line was executed, 0 if the queue was empty.
*/
uint8_t gcode_queue_execute(void);

/// Number of free command slots.
uint8_t gcode_queue_free(void);

/**
  Host keepalive, called every second: while a command takes long, send
  "echo:busy: processing" every keepalive interval.
*/
void gcode_queue_keepalive(void);

/**
  Paused for the user (M600): send "echo:busy: paused for user" instead of
  "processing", also while no command executes (runout pause).
*/
void gcode_queue_set_paused(uint8_t on);

/// Set the keepalive interval in seconds, 0 = off (M113).
void gcode_queue_set_keepalive(uint8_t seconds);

/// Current keepalive interval in seconds.
uint8_t gcode_queue_get_keepalive(void);

/**
  Set by a command which sent its own "ok" (M105 answers "ok T:..."),
  so gcode_queue_execute() doesn't send another one.
*/
extern uint8_t gcode_ok_sent;

#endif /* _GCODE_QUEUE_H */
