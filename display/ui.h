/** \file

  \brief User interface on the display: status screen, and with
  DISPLAY_MENU a menu operated with buttons on the PCF8574 expander.
*/

#ifndef _UI_H
#define _UI_H

#include <stdint.h>
#include "display.h"

#ifdef DISPLAY

/// After display_init().
void ui_init(void);

/// Every 10 ms (clock.c): buttons, menu, status screen.
void ui_tick(void);

/**
  Main loop, between commands: execute a command the menu queued (G-code
  lines, like from the host, but without "ok").
  \return 1 if one was executed.
*/
uint8_t ui_execute(void);

#ifdef DISPLAY_MENU
  /// Whether the menu is open (tests, M-code).
  uint8_t ui_menu_open(void);
#endif

#endif /* DISPLAY */

#endif /* _UI_H */
