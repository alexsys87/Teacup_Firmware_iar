/** \file
  \brief Printer status for host and display: message (M117), progress (M73).
*/

#ifndef _STATUS_H
#define _STATUS_H

#include <stdint.h>

/// Maximum message length, including the terminating zero.
#define STATUS_MSG_LEN 32

/// Set the status message (M117). NULL or "" clears it.
void status_set_message(const char *msg);

/// Current status message, never NULL.
const char *status_get_message(void);

/// Set print progress (M73). percent 0..100, 255 = unknown.
/// remaining in minutes, -1 = unknown.
void status_set_progress(uint8_t percent, int32_t remaining);

uint8_t status_get_progress(void);
int32_t status_get_remaining(void);

#endif /* _STATUS_H */
