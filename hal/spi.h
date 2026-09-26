/** \file
  \brief SPI subsystem for SD card, MAX6675 and MCP3008.

  Other than serial, SPI has to deal with multiple devices. Device selection
  happens before reading and writing, data exchange itself is the same for
  each device.
*/

#ifndef _SPI_H
#define _SPI_H

#include "config_wrapper.h"
#include "pinio.h"

#ifdef SPI

/// Initialise SPI subsystem (master, mode 0, slow clock).
void spi_init(void);

/// Exchange one byte.
uint8_t spi_rw(uint8_t byte);

/// Clock below 400 kHz, required for SD card initialisation.
void spi_speed_100_400(void);

/// Clock about 10 MHz.
void spi_speed_max(void);

#ifdef SD_CARD_SELECT_PIN
TEACUP_INLINE void spi_select_sd(void) {
  WRITE(SD_CARD_SELECT_PIN, 0);
}

TEACUP_INLINE void spi_deselect_sd(void) {
  WRITE(SD_CARD_SELECT_PIN, 1);
}
#endif /* SD_CARD_SELECT_PIN */

#ifdef TEMP_MCP3008
  #ifndef MCP3008_SELECT_PIN
    #error TEMP_MCP3008 needs MCP3008_SELECT_PIN.
  #endif
TEACUP_INLINE void spi_select_mcp3008(void) {
  WRITE(MCP3008_SELECT_PIN, 0);
}

TEACUP_INLINE void spi_deselect_mcp3008(void) {
  WRITE(MCP3008_SELECT_PIN, 1);
}
#endif /* TEMP_MCP3008 */

/// Select a device by its PIN_ID() chip select (e.g. MAX6675).
TEACUP_INLINE void spi_select_id(uint8_t id) {
  pin_id_write(id, 0);
}

TEACUP_INLINE void spi_deselect_id(uint8_t id) {
  pin_id_write(id, 1);
}

#endif /* SPI */

#endif /* _SPI_H */
