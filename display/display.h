
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display.
*/

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <stdint.h>
#include "config_wrapper.h"
#include "displaybus.h"

#ifdef DISPLAY_BUS

  #if defined DISPLAY_TYPE_SSD1306

    /**
      SSD1306 / SH1106 OLED, 128x64 (8 lines) or 128x32 (4 lines,
      DISPLAY_HEIGHT 32), 21 characters of 6x8 pixels per line. I2C
      address 7 bit, 0x3C (0x3D with the address jumper moved).
    */
    #ifndef DISPLAY_I2C_ADDRESS
      #define DISPLAY_I2C_ADDRESS       0x3C
    #endif
    #ifndef DISPLAY_HEIGHT
      #define DISPLAY_HEIGHT            64
    #endif
    #define DISPLAY_LINES               (DISPLAY_HEIGHT / 8)
    #define DISPLAY_COLS                21

    #define DISPLAY

  #elif defined DISPLAY_TYPE_HD44780

    /**
      HD44780 character LCD, 20x4 by default (DISPLAY_COLS, DISPLAY_LINES
      for 16x2, 20x2, 16x4). On the 4 bit parallel bus, or over I2C with a
      PCF8574 backpack (DISPLAY_BUS_I2C, 7 bit address, 0x27 or 0x3F).
    */
    #ifndef DISPLAY_I2C_ADDRESS
      #define DISPLAY_I2C_ADDRESS       0x27
    #endif
    #ifndef DISPLAY_LINES
      #define DISPLAY_LINES             4
    #endif
    #ifndef DISPLAY_COLS
      #define DISPLAY_COLS              20
    #endif

    #define DISPLAY

  #else

    #error Display type not yet supported.

  #endif /* DISPLAY_TYPE_... */

  #define DISPLAY_SYMBOLS_PER_LINE      DISPLAY_COLS

#endif /* DISPLAY_BUS */


/// Character queue, filled by display_writechar(), drained by display_tick().
/// Size must be a power of 2.
#define DISPLAY_BUFFER_SIZE 128

#ifdef DISPLAY
  // Used from the main loop only, so no 'volatile' needed.
  extern uint8_t displayhead;
  extern uint8_t displaytail;
  extern uint8_t displaybuf[DISPLAY_BUFFER_SIZE];
#endif

/**
  Printable ASCII characters and our embedded fonts start at 0x20, so we can
  use 0x00..0x1F for storing control commands in the character queue. That's
  what genuine ASCII does, too, we just use our own code set.

  Queueing up actions together with actual characters not only postpones
  these actions to idle time, it's also necessary to keep them in the right
  order. Without it, writing a few characters, moving the cursor elsewhere and
  writing even more characters would result in all characters being written to
  the second position, because characters would wait in the queue while cursor
  movements were executed immediately.
*/
enum display_low_code {
  low_code_clear = 0x01,
  low_code_set_cursor
};

void display_init(void);
void display_tick(void);
void display_clear(void);

void display_greeting(void);
/// Column in characters on both display types.
void display_set_cursor(uint8_t line, uint8_t column);
void display_writechar(uint8_t data);

void display_writestr_P(const char *data_P);

/**
  Text screen: the status screen and the menu write into a buffer of
  DISPLAY_LINES x DISPLAY_COLS characters, display_update() sends the
  changed parts to the display, a bit at a time (no flicker, no waiting).
*/
void display_text_clear(void);
/// Write text at line / column, clipped at the end of the line.
void display_text(uint8_t line, uint8_t column, const char *text);
/// Fill a line with text, the rest with spaces.
void display_text_line(uint8_t line, const char *text);
/// Called often (clock_poll()): queue changed characters for the display.
void display_update(void);
/// Current text of a line (tests, M-code), DISPLAY_COLS characters.
const char *display_text_get(uint8_t line);
/// Screen and display both blank (after display_clear() at init).
void display_text_reset(void);

#endif /* _DISPLAY_H */
