
/** \file Coordinating reading and writing of SD cards.
*/

#ifndef _SD_H
#define _SD_H

#include "config_wrapper.h"

/*
  SD      : M20..M27 available, either on an SD card or in SPI flash.
  SD_CARD : a real SD card (Petit FatFs, sd.c).
  SD_FLASH: files in SPI flash instead (flash_files.c, SPI_FLASH_FILES).
*/
#ifdef SD_CARD_SELECT_PIN
  #define SD
  #define SD_CARD
  #include "pff.h"
#elif defined SPI_FLASH_FILES
  #define SD
  #define SD_FLASH
#endif

#ifdef SD


void sd_init(void);

void sd_mount(void);

void sd_unmount(void);

void sd_list(const char* path);

void sd_open(const char* filename);

uint8_t sd_read_gcode_line(void);

/// Report the SD print position (M27).
void sd_report_status(void);

/// Set the read position in the open file (M26). \return 1 on success.
uint8_t sd_seek(uint32_t position);

/// Read position in the open file: start of the next line, bytes.
uint32_t sd_position(void);

/// Name of the file opened last (M23), "" if none.
const char *sd_file_name(void);

#ifdef SD_FLASH
  /// Upload in progress (M28 until M29).
  uint8_t sd_writing(void);
  /// M28: create a file, the following lines go into it.
  void sd_start_write(const char *filename);
  /// Store one line of the upload.
  void sd_write_line(const char *line);
  /// M29: finish the upload.
  void sd_end_write(void);
  /// M30: delete a file.
  void sd_delete(const char *filename);
  /// M9002: delete all files.
  void sd_format(void);
#endif

#endif /* SD */

#endif /* _SD_H */
