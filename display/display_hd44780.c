/** \file

  \brief Code specific to the HD44780 display.

  Character LCD, 20x4 by default (DISPLAY_COLS, DISPLAY_LINES), on one of
  two buses:

   - DISPLAY_BUS_4BIT: RS, RW, E, D4..D7 on MCU pins (parallel-4bit.c).

   - DISPLAY_BUS_I2C: the usual PCF8574 backpack ("LCD1602/2004 I2C"),
     7 bit address DISPLAY_I2C_ADDRESS (0x27, 0x3F with a PCF8574A). Its
     pins: P0 RS, P1 RW, P2 E, P3 backlight, P4..P7 D4..D7. Each byte
     goes as two nibbles, each nibble as two I2C bytes (E high, E low), so
     four I2C bytes in one transmission, about 110 us at 400 kHz, longer
     than the 37 us the LCD needs per character. Clear (1.5 ms) is only
     used at startup.
*/

#include "display.h"

#if defined DISPLAY && defined DISPLAY_TYPE_HD44780

#include "displaybus.h"

#define BUFSIZE DISPLAY_BUFFER_SIZE
#include "ringbuffer.h"
#include "delay.h"

#ifdef DISPLAY_BUS_I2C

  #include "i2c.h"

  #define LCD_ADDRESS  (DISPLAY_I2C_ADDRESS << 1)
  #define LCD_RS       0x01
  #define LCD_E        0x04
  #define LCD_BL       0x08

  /// Send a byte, rs = 1 for data (character), 0 for an instruction.
  static void lcd_write(uint8_t data, uint8_t rs) {
    uint8_t hi = (uint8_t)((data & 0xF0) | LCD_BL | (rs ? LCD_RS : 0));
    uint8_t lo = (uint8_t)((data << 4) | LCD_BL | (rs ? LCD_RS : 0));

    i2c_write_to(LCD_ADDRESS, hi | LCD_E, 0);
    i2c_write_to(LCD_ADDRESS, hi, 0);
    i2c_write_to(LCD_ADDRESS, lo | LCD_E, 0);
    i2c_write_to(LCD_ADDRESS, lo, 1);
  }

  /// One nibble, for the 8 bit phase of the initialisation.
  static void lcd_nibble(uint8_t n) {
    i2c_write_to(LCD_ADDRESS, (uint8_t)((n << 4) | LCD_BL | LCD_E), 0);
    i2c_write_to(LCD_ADDRESS, (uint8_t)((n << 4) | LCD_BL), 1);
  }

  static uint8_t lcd_busy(void) {
    return i2c_busy();
  }

  /// Wait until the bytes are out, then give the LCD 'ms' to execute.
  static void lcd_wait(uint8_t ms) {
    while (i2c_busy())
      ;
    delay_ms(ms);
  }

  /**
    Initialisation by instruction (HD44780 datasheet, figure 24): the LCD
    may be in 8 or 4 bit mode after power up, three times "8 bit" gets it
    into 8 bit mode for sure, then "4 bit".
  */
  static void lcd_init_bus(void) {
    i2c_init(0);
    delay_ms(50);
    lcd_nibble(0x3);
    lcd_wait(5);
    lcd_nibble(0x3);
    lcd_wait(1);
    lcd_nibble(0x3);
    lcd_wait(1);
    lcd_nibble(0x2);
    lcd_wait(1);
    lcd_write(0x28, 0);       // 4 bit, 2 lines (also for 4 line LCDs), 5x8.
    lcd_write(0x08, 0);       // Display off.
    lcd_write(0x01, 0);       // Clear.
    lcd_wait(3);
  }

#else /* DISPLAY_BUS_4BIT */

  static void lcd_write(uint8_t data, uint8_t rs) {
    displaybus_write(data, rs ? parallel_4bit_data : parallel_4bit_instruction);
  }

  static uint8_t lcd_busy(void) {
    return displaybus_busy();
  }

  static void lcd_init_bus(void) {
    // Minimum initialisation time after power up.
    delay_ms(15);
    displaybus_init(0);
    lcd_write(0x28, 0);       // 4 bit, 2 lines, 5x8.
    lcd_write(0x01, 0);       // Clear.
    delay_ms(3);
  }

#endif /* DISPLAY_BUS_I2C */

/// DDRAM address of each line's start.
static const uint8_t line_address[4] = {
  0x00, 0x40, DISPLAY_COLS, 0x40 + DISPLAY_COLS
};

/**
 * Initializes the display's controller configuring the way of
 * displaying data.
 */
void display_init(void) {

  lcd_init_bus();

  // Write left to right, no display shifting.
  lcd_write(0x06, 0);
  // Display ON, cursor off, not blinking.
  lcd_write(0x0C, 0);

  display_text_reset();
}

/**
  Show the name until the status screen takes over.
*/
void display_greeting(void) {
  display_text_clear();
  display_text(0, (DISPLAY_COLS - 6) / 2, "Teacup");
  display_text(1, (DISPLAY_COLS - 9) / 2, "STM32F4x1");
}

/**
  Forwards a character or a control command from the display queue to display
  bus. As this is a character based display it's easy.
*/
void display_tick(void) {
  uint8_t data, line;

  if (lcd_busy()) {
    return;
  }

  if (buf_canread(display)) {
    buf_pop(display, data);
    switch (data) {
      case low_code_clear:
        // Takes 1.5 ms, only used at startup.
        lcd_write(0x01, 0);
        delay_ms(2);
        break;

      case low_code_set_cursor:
        /**
          Set the cursor to the given position.

          This is a three-byte control command, so we fetch additional bytes
          from the queue and cross fingers they're actually there.
        */
        buf_pop(display, line);
        buf_pop(display, data);
        // "Set DDRAM Address" command.
        lcd_write((uint8_t)(0x80 | (line_address[line & 3] + data)), 0);
        break;

      default:
        // Should be a printable character.
        lcd_write(data, 1);
        break;
    }
  }
}

#endif /* DISPLAY && DISPLAY_TYPE_HD44780 */
