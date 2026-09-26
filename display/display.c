/** \file

  \brief Display broker.

  Functions common to all displays. The display specific parts are in
  display_ssd1306.c and display_hd44780.c, only the one selected with
  DISPLAY_TYPE_xxx compiles to actual code.
*/

#include "display.h"

#ifdef DISPLAY

#include "delay.h"

uint8_t displayhead = 0;
uint8_t displaytail = 0;
uint8_t displaybuf[DISPLAY_BUFFER_SIZE];

#define BUFSIZE DISPLAY_BUFFER_SIZE
#include "ringbuffer.h"

/**
  Queue up a clear screen command. Very cheap operation on some displays, like
  the HD44780, rather expensive on others, like the SSD1306.
*/
void display_clear(void) {
  display_writechar((uint8_t)low_code_clear);
}

/**
  Sets the cursor to the given position.

  \param line   The vertical cursor position to set, in lines. First line is
                zero.

  \param column The horizontal cursor position to set. In characters on
                character based displays, in pixels on pixel based displays.
                First column is zero.
*/
void display_set_cursor(uint8_t line, uint8_t column) {
  display_writechar((uint8_t)low_code_set_cursor);
  display_writechar((uint8_t)line);
  display_writechar((uint8_t)column);
}

/**
  Prints a character at the current cursor position.

  In case the buffer is full already it waits for a millisecond to allow
  data to be sent to the display, then it tries again. If it still fails then,
  it drops the character. This way we're fairly protected against data loss,
  still we guarantee to not hang forever.
*/
void display_writechar(uint8_t data) {

  if ( ! buf_canwrite(display)) {
    display_tick();
    delay_ms(1);
  }
  if (buf_canwrite(display)) {
    buf_push(display, data);
  }
}

void display_writestr_P(const char *data_P) {
  uint8_t r;

  while ((r = (uint8_t)*data_P++) != 0)
    display_writechar(r);
}

#endif /* DISPLAY */
