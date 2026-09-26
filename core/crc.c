/** \file
  \brief crc16 routine
*/

#include "crc.h"

/// CRC-16 (polynomial 0xA001), equivalent to avr-libc's _crc16_update().
static uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

/** block-at-once CRC16 calculator
  \param *data data to find crc16 for
  \param len length of data
  \return uint16 crc16 of passed data
*/
uint16_t crc_block(void *data, uint16_t len) {
  uint16_t crc = 0xfeed;
  uint8_t *ptr = (uint8_t *)data;

  while (len--)
    crc = crc16_update(crc, *ptr++);
  return crc;
}
