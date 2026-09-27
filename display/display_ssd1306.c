/** \file

  \brief Code specific to the SSD1306 display (and SH1106).

  OLED with 128x64 pixels (DISPLAY_HEIGHT 64, the 0.96" modules) or 128x32
  (DISPLAY_HEIGHT 32, 0.91"), on I2C. Text in a grid of 6x8 pixel cells
  (font_6x8.c): 21 characters, 8 or 4 lines, one line per display page.

  SH1106 (the 1.3" modules): the same commands in page addressing mode, but
  132 columns RAM with the visible 128 in the middle, DISPLAY_SH1106 shifts
  by 2. DISPLAY_ROTATE_180 turns the picture for modules mounted upside
  down.

  Data path: display_writechar() queues characters and cursor commands
  (128 bytes), display_tick() renders one of them when the I2C bus is idle
  and puts the bytes into the I2C queue (drained by its interrupt).
*/

#include "display.h"

#if defined DISPLAY && defined DISPLAY_TYPE_SSD1306

#include "displaybus.h"

#define BUFSIZE DISPLAY_BUFFER_SIZE
#include "ringbuffer.h"
#include "font.h"
#include "delay.h"

#ifdef DISPLAY_SH1106
  #define COLUMN_OFFSET 2
#else
  #define COLUMN_OFFSET 0
#endif
/// Text starts at this pixel column: 21 cells of 6 = 126, centred.
#define TEXT_OFFSET   (COLUMN_OFFSET + 1)

static const uint8_t init_sequence[] = {
  0x00,             // Command marker.
  0xAE,             // Display off.
  0xD5, 0x80,       // Display clock divider (reset).
  0xA8, DISPLAY_HEIGHT - 1,   // Multiplex ratio: 1/64 or 1/32 duty.
  0xD3, 0x00,       // No display offset.
  0x40 | 0x00,      // Start line 0.
  0x20, 0x02,       // Page addressing mode (reset).
  #ifdef DISPLAY_ROTATE_180
    0xA0,           // No segment remap.
    0xC0,           // COM scan up.
  #else
    0xA1,           // Segment remap: column 127 is SEG0 (usual mounting).
    0xC8,           // COM scan down.
  #endif
  #if DISPLAY_HEIGHT == 64
    0xDA, 0x12,     // Alternative COM pins (128x64).
  #else
    0xDA, 0x02,     // Sequential COM pins (128x32).
  #endif
  0x81, 0x7F,       // Contrast (reset).
  0xD9, 0xF1,       // Precharge period.
  0xDB, 0x20,       // Vcomh (reset).
  0x8D, 0x14,       // Charge pump on.
  0xA6,             // Positive display.
  0xA4,             // Show RAM content.
  0xAF              // Display on.
};

/// Set the RAM position: page (text line) and pixel column.
static void set_position(uint8_t page, uint8_t column) {
  displaybus_write(0x00, 0);                    // Command marker.
  displaybus_write(0xB0 | (page & 0x07), 0);
  displaybus_write(0x00 | (column & 0x0F), 0);
  displaybus_write(0x10 | ((column >> 4) & 0x0F), 1);
}

/**
  Initializes the display's controller configuring the way of
  displaying data.
*/
void display_init(void) {
  uint8_t i;

  displaybus_init(DISPLAY_I2C_ADDRESS << 1);

  for (i = 0; i < sizeof(init_sequence); i++) {
    // Send last byte with 'last_byte' set.
    displaybus_write(init_sequence[i], (i == sizeof(init_sequence) - 1));
  }
  display_clear();
  while (buf_canread(display))
    display_tick();
  display_text_reset();
}

/**
  Show the name until the status screen takes over.
*/
void display_greeting(void) {
  display_text_clear();
  display_text(DISPLAY_LINES / 2 - 1, 7, "Teacup");
  display_text(DISPLAY_LINES / 2, 5, "STM32F4x1");
}

/**
  Forwards a character or a control command from the display queue to the I2C
  queue.
*/
void display_tick(void) {
  uint16_t i;
  uint8_t data, line, column;

  if (displaybus_busy()) {
    return;
  }

  /**
    Possible strategy for error recovery: after a failed, aborted I2C
    transmisson, 'i2c_state & I2C_INTERRUPTED' in i2c.c evaluates to true.

    Having a getter like displaybus_failed() would allow to test this condition
    here, so we could resend the previous data again, instead of grabbing a
    new byte from the buffer.
  */

  if (buf_canread(display)) {
    buf_pop(display, data);
    switch (data) {
      case low_code_clear:
        /**
          Clear the screen. The display has no 'clear' command, so write
          zeros, page by page (works on the SH1106 as well, which has no
          horizontal addressing mode).
        */
        for (line = 0; line < DISPLAY_LINES; line++) {
          set_position(line, 0);
          displaybus_write(0x40, 0);            // Data marker.
          for (i = 0; i < 128 + 2 * COLUMN_OFFSET; i++)
            displaybus_write(0x00, (i == 128 + 2 * COLUMN_OFFSET - 1));
        }
        break;

      case low_code_set_cursor:
        /**
          Set the cursor to the given position.

          This is a three-byte control command, so we fetch additional bytes
          from the queue and cross fingers they're actually there.
        */
        buf_pop(display, line);
        buf_pop(display, column);
        set_position(line, (uint8_t)(TEXT_OFFSET +
                     column * (FONT_COLUMNS + FONT_SYMBOL_SPACE)));
        break;

      default:
        // Should be a printable character.
        if (data < 0x20 || data > 0x7E)
          data = '?';

        // Write pixels command.
        displaybus_write(0x40, 0);

        // Send the character bitmap and the space after it.
        for (i = 0; i < FONT_COLUMNS; i++)
          displaybus_write(font[data - 0x20].data[i], 0);
        for (i = 0; i < FONT_SYMBOL_SPACE; i++)
          displaybus_write(0x00, (i == FONT_SYMBOL_SPACE - 1));
        break;
    }
  }
}

#endif /* DISPLAY && DISPLAY_TYPE_SSD1306 */
