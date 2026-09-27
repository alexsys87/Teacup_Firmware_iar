/** \file
  \brief G-code files in SPI flash, used like an SD card (M20..M30).

  Active with SPI_FLASH_FILES (default with SPI_FLASH and no SD card).
  Implements the sd_*() interface of sd.c, so M20, M23, M24, M25, M26, M27
  work unchanged. Additionally: M28/M29 upload a file (Pronterface "Upload
  to SD"), M30 deletes a file, M9002 deletes all.

  Layout (addresses in the SPI flash):
    0x0000  4 kB  settings (M500), see flash_store.c
    0x1000  4 kB  directory, 128 entries of 32 bytes
    0x2000  ...   file data, each file starts at a 4 kB sector
    end - 8 kB    power loss records (POWER_LOSS_RECOVERY), power_loss.c

  Directory entry: magic, start, size, flags, name (16). Entries are
  appended; size stays 0xFFFFFFFF until the upload is complete (M29), flags
  go to 0 when deleted. Programming only clears bits, so no erase is needed
  for these updates. A data sector is erased when writing enters it.

  New files go behind the last one. When the flash is full and no files
  are left (M30), the directory is erased and everything starts over;
  M9002 does that right away. There's no compaction: with files left,
  delete them all to reclaim the space.
*/

#include "sd.h"

#ifdef SD_FLASH

#include <string.h>
#include "spi_flash.h"
#include "serial.h"
#include "sermsg.h"
#include "gcode_parse.h"
#include "power_loss.h"

#define DIR_ADDR      0x1000UL
#define DATA_ADDR     0x2000UL
#define DIR_ENTRIES   128
#define ENTRY_MAGIC   0x4C494654UL          // "TFIL"
#define ERASED        0xFFFFFFFFUL

typedef struct {
  uint32_t magic;
  uint32_t start;
  uint32_t size;
  uint32_t flags;                           // ERASED = valid, 0 = deleted
  char     name[16];
} dir_entry_t;

//static uint8_t mounted;

// File being read (M23/M24).
static uint32_t rd_start, rd_size, rd_pos;
static uint8_t rd_open;
static char rd_name[17];

// File being written (M28/M29).
static uint8_t wr_active;
static int16_t wr_entry;
static uint32_t wr_start, wr_pos;
static uint8_t wr_page[SPI_FLASH_PAGE];
static uint16_t wr_fill;

/// End of the file area: the power loss records sit behind it.
static uint32_t files_end(void) {
  #ifdef POWER_LOSS_RECOVERY
    return spi_flash_size() - PLR_FLASH_SIZE;
  #else
    return spi_flash_size();
  #endif
}

static void read_entry(uint16_t i, dir_entry_t *e) {
  spi_flash_read(DIR_ADDR + (uint32_t)i * sizeof(dir_entry_t), e, sizeof(*e));
}

/// Entry in use and complete, not deleted.
static uint8_t entry_valid(const dir_entry_t *e) {
  return e->magic == ENTRY_MAGIC && e->size != ERASED && e->flags == ERASED;
}

/// First data address behind all files (4 kB aligned).
static uint32_t data_end(void) {
  uint32_t end = DATA_ADDR;
  uint16_t i;
  dir_entry_t e;

  for (i = 0; i < DIR_ENTRIES; i++) {
    uint32_t last;

    read_entry(i, &e);
    if (e.magic == ERASED)
      break;
    if (e.magic != ENTRY_MAGIC)
      continue;
    if (e.size == ERASED) {
      // Interrupted upload: find the first erased page behind its start.
      uint32_t a = e.start, w;

      for (;;) {
        spi_flash_read(a, &w, 4);
        if (w == ERASED || a + SPI_FLASH_PAGE >= files_end())
          break;
        a += SPI_FLASH_PAGE;
      }
      last = a;
    }
    else {
      last = e.start + e.size;
    }
    last = (last + SPI_FLASH_SECTOR - 1) & ~(SPI_FLASH_SECTOR - 1);
    if (last > end)
      end = last;
  }
  return end;
}

/// Index of the first free directory entry, -1 if full.
static int16_t free_entry(void) {
  uint16_t i;
  dir_entry_t e;

  for (i = 0; i < DIR_ENTRIES; i++) {
    read_entry(i, &e);
    if (e.magic == ERASED)
      return (int16_t)i;
  }
  return -1;
}

/// Index of a valid file with this name, -1 if none.
static int16_t find_file(const char *name) {
  uint16_t i;
  dir_entry_t e;

  for (i = 0; i < DIR_ENTRIES; i++) {
    read_entry(i, &e);
    if (e.magic == ERASED)
      break;
    if (entry_valid(&e) && strncmp(e.name, name, sizeof(e.name)) == 0)
      return (int16_t)i;
  }
  return -1;
}

static uint8_t any_valid(void) {
  uint16_t i;
  dir_entry_t e;

  for (i = 0; i < DIR_ENTRIES; i++) {
    read_entry(i, &e);
    if (e.magic == ERASED)
      break;
    if (entry_valid(&e))
      return 1;
  }
  return 0;
}

static void mark_deleted(int16_t i) {
  uint32_t zero = 0;

  spi_flash_program(DIR_ADDR + (uint32_t)i * sizeof(dir_entry_t) +
                    offsetof(dir_entry_t, flags), &zero, 4);
}

void sd_init(void) {
  //mounted = 0;
  rd_open = 0;
  wr_active = 0;
}

void sd_mount(void) {
  if ( ! spi_flash_present()) {
    serial_writestr("echo:SPI flash not found\n");
    //mounted = 0;
    return;
  }
  //mounted = 1;
}

void sd_unmount(void) {
  //mounted = 0;
  rd_open = 0;
  gcode_sources &= (uint8_t)~GCODE_SOURCE_SD;
}

void sd_list(const char *path) {
  uint16_t i;
  dir_entry_t e;

  (void)path;
  if ( ! spi_flash_present())
    return;
  serial_writestr("Begin file list\n");
  for (i = 0; i < DIR_ENTRIES; i++) {
    read_entry(i, &e);
    if (e.magic == ERASED)
      break;
    if (entry_valid(&e)) {
      e.name[sizeof(e.name) - 1] = '\0';
      serial_writestr(e.name);
      serial_writechar(' ');
      serwrite_uint32(e.size);
      serial_writechar('\n');
    }
  }
  serial_writestr("End file list\n");
}

void sd_open(const char *filename) {
  int16_t i;
  dir_entry_t e;

  rd_open = 0;
  rd_name[0] = '\0';
  if ( ! spi_flash_present() || (i = find_file(filename)) < 0) {
    serial_writestr("echo:open failed, File: ");
    serial_writestr(filename);
    serial_writestr(".\n");
    return;
  }
  read_entry((uint16_t)i, &e);
  rd_start = e.start;
  rd_size = e.size;
  rd_pos = 0;
  rd_open = 1;
  memcpy(rd_name, e.name, sizeof(e.name));
  rd_name[sizeof(e.name)] = '\0';
  serial_writestr("File opened: ");
  serial_writestr(filename);
  serial_writestr(" Size: ");
  serwrite_uint32(rd_size);
  serial_writestr("\nFile selected\n");
}

uint8_t sd_read_gcode_line(void) {
  uint8_t buf[32];

  if ( ! rd_open)
    return 1;

  while (rd_pos < rd_size) {
    uint32_t n = rd_size - rd_pos;
    uint32_t k;

    if (n > sizeof(buf))
      n = sizeof(buf);
    spi_flash_read(rd_start + rd_pos, buf, n);
    for (k = 0; k < n; k++) {
      rd_pos++;
      if (gcode_parse_char(buf[k]) || buf[k] == '\n')
        return 0;                           // One line done.
    }
  }
  gcode_parse_char('\n');                   // Last line without EOL.
  rd_open = 0;
  return 1;
}

void sd_report_status(void) {
  if (rd_open && (gcode_sources & GCODE_SOURCE_SD)) {
    serial_writestr("SD printing byte ");
    serwrite_uint32(rd_pos);
    serial_writechar('/');
    serwrite_uint32(rd_size);
    serial_writechar('\n');
  }
  else {
    serial_writestr("Not SD printing\n");
  }
}

uint8_t sd_seek(uint32_t position) {
  if ( ! rd_open || position > rd_size)
    return 0;
  rd_pos = position;
  return 1;
}

uint32_t sd_position(void) {
  return rd_pos;
}

const char *sd_file_name(void) {
  return rd_name;
}

uint32_t sd_file_size(void) {
  return rd_open ? rd_size : 0;
}

uint8_t sd_dir_entry(const char *path, uint16_t index, char *name) {
  uint16_t i;
  dir_entry_t e;

  (void)path;                             // No directories in the flash.
  if ( ! spi_flash_present())
    return SD_ENTRY_ERROR;
  for (i = 0; i < DIR_ENTRIES; i++) {
    read_entry(i, &e);
    if (e.magic == ERASED)
      break;
    if (entry_valid(&e) && index-- == 0) {
      memcpy(name, e.name, 12);
      name[12] = '\0';
      return SD_ENTRY_FILE;
    }
  }
  return SD_ENTRY_NONE;
}

/* ---- Writing ---- */

uint8_t sd_writing(void) {
  return wr_active;
}

/// Program the page buffer, erase a sector first when entering it.
static uint8_t flush_page(void) {
  uint32_t addr = wr_pos - wr_fill;

  if (wr_fill == 0)
    return 1;
  if (addr + wr_fill > files_end())
    return 0;
  spi_flash_program(addr, wr_page, wr_fill);
  wr_fill = 0;
  return 1;
}

void sd_start_write(const char *filename) {
  int16_t old, e;
  uint32_t start;
  dir_entry_t entry;

  if ( ! spi_flash_present()) {
    serial_writestr("echo:SPI flash not found\n");
    return;
  }
  if (filename[0] == '\0') {
    serial_writestr("echo:M28 needs a file name\n");
    return;
  }

  // Replacing a file: the old one gets deleted.
  if ((old = find_file(filename)) >= 0)
    mark_deleted(old);

  e = free_entry();
  start = data_end();
  if ((e < 0 || start + SPI_FLASH_SECTOR > files_end()) && ! any_valid()) {
    // Only deleted files left: start over.
    spi_flash_erase_sector(DIR_ADDR);
    e = 0;
    start = DATA_ADDR;
  }
  if (e < 0 || start + SPI_FLASH_SECTOR > files_end()) {
    serial_writestr("echo:Flash full, delete files (M30, M9002)\n");
    return;
  }

  memset(&entry, 0xFF, sizeof(entry));
  entry.magic = ENTRY_MAGIC;
  entry.start = start;
  strncpy(entry.name, filename, sizeof(entry.name) - 1);
  entry.name[sizeof(entry.name) - 1] = '\0';
  spi_flash_program(DIR_ADDR + (uint32_t)e * sizeof(entry), &entry, sizeof(entry));

  wr_entry = e;
  wr_start = start;
  wr_pos = start;
  wr_fill = 0;
  wr_active = 1;
  rd_open = 0;
  serial_writestr("Writing to file: ");
  serial_writestr(filename);
  serial_writechar('\n');
}

void sd_write_line(const char *line) {
  const char *p = line;

  if ( ! wr_active)
    return;

  for (;;) {
    char c = *p ? *p++ : '\n';

    // Entering a new sector: erase it (programming works on erased cells).
    if ((wr_pos & (SPI_FLASH_SECTOR - 1)) == 0) {
      if ( ! flush_page() || wr_pos + SPI_FLASH_SECTOR > files_end()) {
        serial_writestr("echo:Flash full, upload aborted\n");
        wr_active = 0;
        return;
      }
      spi_flash_erase_sector(wr_pos);
    }
    wr_page[wr_fill++] = (uint8_t)c;
    wr_pos++;
    if (wr_fill == SPI_FLASH_PAGE || (wr_pos & (SPI_FLASH_PAGE - 1)) == 0)
      flush_page();
    if (c == '\n')
      break;
  }
}

void sd_end_write(void) {
  uint32_t size;

  if ( ! wr_active)
    return;
  flush_page();
  size = wr_pos - wr_start;
  spi_flash_program(DIR_ADDR + (uint32_t)wr_entry * sizeof(dir_entry_t) +
                    offsetof(dir_entry_t, size), &size, 4);
  wr_active = 0;
  serial_writestr("Done saving file.\n");
}

void sd_delete(const char *filename) {
  int16_t i;

  if ( ! spi_flash_present() || (i = find_file(filename)) < 0) {
    serial_writestr("Deletion failed, File: ");
    serial_writestr(filename);
    serial_writestr(".\n");
    return;
  }
  mark_deleted(i);
  serial_writestr("File deleted:");
  serial_writestr(filename);
  serial_writechar('\n');
}

void sd_format(void) {
  if ( ! spi_flash_present())
    return;
  rd_open = 0;
  wr_active = 0;
  spi_flash_erase_sector(DIR_ADDR);
  serial_writestr("echo:Flash files deleted\n");
}

#endif /* SD_FLASH */
