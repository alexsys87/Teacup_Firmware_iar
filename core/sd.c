
/** \file Coordinating reading and writing of SD cards.
*/

#include "sd.h"

#ifdef SD_CARD

#include "delay.h"
#include "pinio.h"
#include "serial.h"
#include "sersendf.h"
#include "sermsg.h"
#include "gcode_parse.h"
#include "pff.h" 

static FATFS sdfile;
static FRESULT result;

/// Name of the open file (8.3 plus path, as given to M23).
static char open_name[32];

/** Initialize SPI for SD card reading.
*/
void sd_init(void) {
  WRITE(SD_CARD_SELECT_PIN, 1);
  SET_OUTPUT(SD_CARD_SELECT_PIN);
}

/** Mount the SD card (M21, and once at startup).

  Messages as Marlin's, hosts look for them.
*/
void sd_mount(void) {
  gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
  open_name[0] = '\0';
  result = pf_mount(&sdfile);
  if (result == FR_OK)
    serial_writestr("echo:SD card ok\n");
  else
    sersendf_P(("echo:SD init fail (%su)\n"), result);
}

/** Unmount the SD card.

  This makes just sure subsequent reads to the card do nothing, instead of
  trying and failing. Not mandatory, just removing the card is fine, as well
  as inserting and mounting another one without previous unmounting.
*/
void sd_unmount(void) {
  gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
  open_name[0] = '\0';
  pf_unmount(&sdfile);
}

/** List a directory, with the subdirectories.

  \param path  Path of the directory, "" for the top level.
  \param depth Levels of subdirectories still to list.

  Files as "NAME.GCO size", in subdirectories "DIR/NAME.GCO size", like
  Marlin's M20: hosts (OctoPrint, Pronterface) take the name up to the
  space. Hidden and system entries (e.g. "System Volume Information") are
  left out.
*/
static void list_dir(char *path, uint8_t depth) {
  FILINFO fno;
  DIR dir;
  uint8_t len, i;

  if (pf_opendir(&dir, path[0] ? path : "/") != FR_OK)
    return;
  for (;;) {
    if (pf_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0)
      break;
    if (fno.fattrib & (AM_HID | AM_SYS))
      continue;
    if (fno.fattrib & AM_DIR) {
      // Append "/NAME" to the path (the caller's buffer holds 2 levels).
      if (depth == 0 || fno.fname[0] == '.')
        continue;
      for (len = 0; path[len]; len++) ;
      if (len)
        path[len++] = '/';
      for (i = 0; fno.fname[i]; i++)
        path[len + i] = fno.fname[i];
      path[len + i] = '\0';
      list_dir(path, depth - 1);
      path[len ? len - 1 : 0] = '\0';
      continue;
    }
    if (path[0]) {
      serial_writestr(path);
      serial_writechar('/');
    }
    serial_writestr(fno.fname);
    serial_writechar(' ');
    serwrite_uint32(fno.fsize);
    serial_writechar('\n');
    delay_ms(2); // Time for sending the characters.
  }
}

/** List the card (M20).

  \param path The path to list. Toplevel path is "/".
*/
void sd_list(const char* path) {
  char p[40];
  DIR dir;

  result = pf_opendir(&dir, path);
  if (result != FR_OK) {
    sersendf_P(("echo:Failed to open dir. (%su)\n"), result);
    return;
  }
  // Markers expected by Pronterface, OctoPrint & co.
  serial_writestr("Begin file list\n");
  p[0] = '\0';
  list_dir(p, 2);
  serial_writestr("End file list\n");
}

/** Open a file for reading.

  \param filename Name of the file to open and to read G-code from.

  Before too long this will cause the printer to read G-code from this file
  until done or until stopped by G-code coming in over the serial line.
*/
void sd_open(const char* filename) {
  uint8_t i;

  open_name[0] = '\0';
  result = pf_open(filename);
  if (result != FR_OK) {
    // Marlin's message, hosts look for it.
    serial_writestr("echo:open failed, File: ");
    serial_writestr(filename);
    serial_writestr(".\n");
    return;
  }
  for (i = 0; filename[i] && i < sizeof(open_name) - 1; i++)
    open_name[i] = filename[i];
  open_name[i] = '\0';
  serial_writestr("File opened: ");
  serial_writestr(filename);
  serial_writestr(" Size: ");
  serwrite_uint32(sdfile.fsize);
  serial_writestr("\nFile selected\n");
}

uint32_t sd_position(void) {
  return sdfile.fptr;
}

uint32_t sd_file_size(void) {
  return open_name[0] ? sdfile.fsize : 0;
}

/// Whether an 8.3 name has an extension starting with 'G'.
static uint8_t is_gcode(const char *name) {
  while (*name && *name != '.')
    name++;
  return name[0] == '.' && name[1] == 'G';
}

uint8_t sd_dir_entry(const char *path, uint16_t index, char *name) {
  FILINFO fno;
  DIR dir;
  uint8_t i;

  if (pf_opendir(&dir, path[0] ? path : "/") != FR_OK)
    return SD_ENTRY_ERROR;
  for (;;) {
    if (pf_readdir(&dir, &fno) != FR_OK)
      return SD_ENTRY_ERROR;
    if (fno.fname[0] == 0)
      return SD_ENTRY_NONE;
    if ((fno.fattrib & (AM_HID | AM_SYS)) || fno.fname[0] == '.')
      continue;
    if ( ! (fno.fattrib & AM_DIR) && ! is_gcode(fno.fname))
      continue;
    if (index-- == 0)
      break;
  }
  for (i = 0; fno.fname[i] && i < 12; i++)
    name[i] = fno.fname[i];
  name[i] = '\0';
  return (fno.fattrib & AM_DIR) ? SD_ENTRY_DIR : SD_ENTRY_FILE;
}

const char *sd_file_name(void) {
  return open_name;
}

/** Read a line of G-code from a file.

  \param A pointer to the parser function. This function should accept an
         uint8_t with the character to parse and return an uint8_t whether
         end of line (EOL) was reached.

  \return Whether end of line (EOF) was reached or an error happened.

  Juggling with a buffer smaller than 512 bytes means that the underlying
  SD card handling code reads a full sector (512 bytes) in each operation,
  but throws away everything not fitting into this buffer. Next read operation
  reads the very same sector, but keeps a different part. That's ineffective.

  Much better is to parse straight as it comes from the card. This way there
  is no need for a buffer at all. Sectors are still read multiple times, but
  at least one line is read in one chunk (unless it crosses a sector boundary).
*/
uint8_t sd_read_gcode_line(void) {

  result = pf_parse_line(&gcode_parse_char);
  if (result == FR_END_OF_FILE) {
    return 1;
  }
  else if (result != FR_OK) {
    sersendf_P(("echo:Failed to read file. (%su)\n"), result);
    return 1;
  }

  return 0;
}

void sd_report_status(void) {
  if (gcode_sources & GCODE_SOURCE_SD) {
    serial_writestr("SD printing byte ");
    serwrite_uint32(sdfile.fptr);
    serial_writechar('/');
    serwrite_uint32(sdfile.fsize);
    serial_writechar('\n');
  }
  else {
    serial_writestr("Not SD printing\n");
  }
}

uint8_t sd_seek(uint32_t position) {
  result = pf_lseek(position);
  if (result != FR_OK) {
    sersendf_P(("echo:SD seek failed. (%su)\n"), result);
    return 0;
  }
  return 1;
}

#endif /* SD_CARD */
