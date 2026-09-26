/** \file
  \brief Power loss recovery for prints from SD card or SPI flash
  (M413, M1000), like Marlin's POWER_LOSS_RECOVERY.
*/

#ifndef _POWER_LOSS_H
#define _POWER_LOSS_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef POWER_LOSS_RECOVERY

#include "spi_flash.h"

/// Records at the end of the SPI flash, two sectors used as a ring.
#define PLR_FLASH_SIZE (2 * SPI_FLASH_SECTOR)

/**
  Startup, after the settings and the SD / flash files: look for a record
  of an interrupted print and tell the host about it.
*/
void plr_init(void);

/// Main loop, before a line is read from the SD card / flash file.
void plr_line_begin(void);

/**
  Main loop, every round: store the print state at layer changes (and
  every PLR_INTERVAL seconds) while a file prints.
*/
void plr_tick(void);

/// The file is read to the end; the print ends when the queue is empty.
void plr_file_done(void);

/// The print ended or was cancelled: no resume.
void plr_clear(void);

/**
  M1000: resume the interrupted print. Heats up, lifts Z, homes X and Y,
  primes, goes back and continues the file. \return 1 on success.
*/
uint8_t plr_resume(void);

/// Whether a record of an interrupted print exists.
uint8_t plr_pending(void);

/// M413 without parameters, M1000 without a record.
void plr_report(void);

#endif /* POWER_LOSS_RECOVERY */

#endif /* _POWER_LOSS_H */
