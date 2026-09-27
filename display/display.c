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

/// Wanted screen content and what was sent to the display already.
static char screen[DISPLAY_LINES][DISPLAY_COLS + 1];
static char shown[DISPLAY_LINES][DISPLAY_COLS];
static uint8_t update_line;

void display_text_clear(void) {
  uint8_t l;

  for (l = 0; l < DISPLAY_LINES; l++)
    display_text_line(l, "");
}

void display_text(uint8_t line, uint8_t column, const char *text) {
  if (line >= DISPLAY_LINES)
    return;
  while (*text && column < DISPLAY_COLS)
    screen[line][column++] = *text++;
}

void display_text_line(uint8_t line, const char *text) {
  uint8_t c;

  if (line >= DISPLAY_LINES)
    return;
  for (c = 0; c < DISPLAY_COLS; c++)
    screen[line][c] = *text ? *text++ : ' ';
}

const char *display_text_get(uint8_t line) {
  screen[line][DISPLAY_COLS] = '\0';
  return screen[line];
}

/**
  Queue the changed part of one line, if the display queue has room for
  it. Lines take turns, so a busy line doesn't starve the others.
*/
void display_update(void) {
  uint8_t n, l, first, last, c;

  for (n = 0; n < DISPLAY_LINES; n++) {
    l = update_line;
    update_line = (uint8_t)((update_line + 1) % DISPLAY_LINES);
    for (first = 0; first < DISPLAY_COLS; first++)
      if (screen[l][first] != shown[l][first])
        break;
    if (first == DISPLAY_COLS)
      continue;
    for (last = DISPLAY_COLS - 1; last > first; last--)
      if (screen[l][last] != shown[l][last])
        break;
    // Cursor command (3 bytes) plus the characters.
    if (((displaytail - displayhead - 1) & (DISPLAY_BUFFER_SIZE - 1)) <
        (unsigned)(last - first + 4))
      return;
    display_set_cursor(l, first);
    for (c = first; c <= last; c++) {
      uint8_t ch = (uint8_t)screen[l][c];

      display_writechar(ch >= 0x20 && ch < 0x7F ? ch : '?');
      shown[l][c] = screen[l][c];
    }
    return;
  }
}

/// After a display clear: the display shows spaces, the screen too.
void display_text_reset(void) {
  uint8_t l, c;

  for (l = 0; l < DISPLAY_LINES; l++)
    for (c = 0; c < DISPLAY_COLS; c++)
      screen[l][c] = shown[l][c] = ' ';
}

#endif /* DISPLAY */
