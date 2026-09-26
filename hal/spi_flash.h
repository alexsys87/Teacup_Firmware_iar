/** \file
  \brief SPI NOR flash (W25Qxx and compatibles) on the Black Pill footprint.
*/

#ifndef _SPI_FLASH_H
#define _SPI_FLASH_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef SPI_FLASH

/// Sector (smallest erase unit) and page (largest program unit) size.
#define SPI_FLASH_SECTOR  4096UL
#define SPI_FLASH_PAGE    256UL

/**
  Detect the chip (JEDEC ID) and report it. Call after spi_init().
  \return 1 if a chip was found.
*/
uint8_t spi_flash_init(void);

/// Whether a chip was found.
uint8_t spi_flash_present(void);

/// Chip size in bytes (0 if none).
uint32_t spi_flash_size(void);

/// Read any number of bytes.
void spi_flash_read(uint32_t addr, void *buf, uint32_t len);

/**
  Program any number of bytes (page boundaries are handled). Bits can only
  go from 1 to 0, erase first.
*/
void spi_flash_program(uint32_t addr, const void *buf, uint32_t len);

/// Erase the 4 kB sector containing addr.
void spi_flash_erase_sector(uint32_t addr);

#endif /* SPI_FLASH */

#endif /* _SPI_FLASH_H */
