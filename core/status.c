/** \file
  \brief Printer status: message (M117) and print progress (M73).

  Kept independent of a display, so the host can query it (M73 without
  parameters) and a display, if any, shows it.
*/

#include "status.h"

static char status_msg[STATUS_MSG_LEN];
static uint8_t status_percent = 255;
static int32_t status_remaining = -1;

void status_set_message(const char *msg) {
  uint8_t i = 0;

  if (msg) {
    while (msg[i] && i < STATUS_MSG_LEN - 1) {
      status_msg[i] = msg[i];
      i++;
    }
  }
  status_msg[i] = '\0';
}

const char *status_get_message(void) {
  return status_msg;
}

void status_set_progress(uint8_t percent, int32_t remaining) {
  status_percent = (percent > 100 && percent != 255) ? 100 : percent;
  status_remaining = remaining;
}

uint8_t status_get_progress(void) {
  return status_percent;
}

int32_t status_get_remaining(void) {
  return status_remaining;
}
