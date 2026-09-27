
#ifndef _FONT_H
#define _FONT_H

#include <stdint.h>

/**
  Fixed width font for the pixel displays (SSD1306): 5x7 pixels in a 6x8
  cell, so 21 characters per line on 128 pixels, text in a grid like on a
  character LCD. Characters 0x20..0x7E, one byte per column, bit 0 at the
  top.
*/
#define FONT_ROWS             8
#define FONT_COLUMNS          5
#define FONT_SYMBOL_SPACE     1

typedef struct {
  uint8_t data[FONT_COLUMNS];
} symbol_t;

extern const symbol_t font[];

#endif /* _FONT_H */
