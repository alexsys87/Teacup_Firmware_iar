/** \file
  \brief Persistent storage of a settings record in one Flash sector.
*/

#ifndef _FLASH_STORE_H
#define _FLASH_STORE_H

#include <stdint.h>

/**
  Record version of the print statistics (M78). They share the sector
  with the settings: reading the settings takes the newest record of
  another version, the statistics the newest of this one; erasing the
  full sector keeps the newest record of the other kind.
*/
#define FLASH_STORE_STATS 0x5354

/**
  Read the newest valid record (of the statistics, or of the settings).
  \return 1 if a record with this version and length was found and copied.
*/
uint8_t flash_store_read(void *data, uint16_t length, uint16_t version);

/**
  Append a record, erasing the sector first when it's full.
  \return 1 on success.

  Erasing takes 1..2 s and stalls the CPU. Interrupts are disabled during
  that time, so call it only with an empty movement queue.
*/
uint8_t flash_store_write(const void *data, uint16_t length, uint16_t version);

/// Whether the records are kept in SPI flash (SPI_FLASH_SETTINGS).
uint8_t flash_store_on_spi(void);

/// CRC-32 (IEEE 802.3) of a buffer.
uint32_t flash_store_crc32(const void *data, uint32_t length);

#endif /* _FLASH_STORE_H */
