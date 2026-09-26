/** \file
  \brief Reaction to a lost host (M86): park and lower the hotend
  temperature when a print from the host stops receiving lines.
*/

#ifndef _HOST_WATCH_H
#define _HOST_WATCH_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef HOST_WATCH

/// A line from host port 'port' starts executing (gcode_queue.c).
void host_watch_line(uint8_t port);

/// A move with E from the host was queued: a print is running.
void host_watch_arm(void);

/**
  Main loop, between two commands: park if the host is gone. Doesn't
  replace the thermal protection.
*/
void host_watch_poll(void);

#endif /* HOST_WATCH */

#endif /* _HOST_WATCH_H */
