/** \file
  \brief Power loss recovery (M413, M1000), see power_loss.h.

  Printing from the SD card or from files in SPI flash, the state of the
  print is stored in the SPI flash at every layer change (Z goes up, at
  most every 2 s) and at least every PLR_INTERVAL seconds.

  The movement queue holds many moves read ahead of the one executing, so
  the end of the queue isn't what the printer is doing. Each move carries
  the file line it came from (plr_line_t: start of the line in the file,
  G-code position and feedrate before it). The record stores that of the
  move executing now: file name and line position, X Y Z E and F before
  the line, temperatures, fan, E mode, speed and flow. Resuming prints
  from that line again, so nothing is missing; what was printed since the
  record (at most one layer or PLR_INTERVAL seconds) is printed twice.

  At the next start the firmware reports the record, M1000 resumes:

   1. heat the bed, then the hotend (waiting),
   2. take Z as stored (the lead screws hold it without power), lift it by
      PLR_Z_RAISE, home X and Y,
   3. prime PLR_PURGE_LENGTH mm, back to X, Y, then down to Z,
   4. restore feedrate, E mode, M220 / M221, fan, continue the file.

  M1000 C discards the record, M413 S0 switches recording off. A finished
  print (file read to the end and the movement queue empty) or a newly
  selected file (M23) discards it, too.

  Storage: the last two 4 kB sectors of the SPI flash, 128 byte records
  with sequence number and CRC, written one after the other; a sector is
  erased when writing enters it, the newest record is always in the other
  one. Printing from the flash files, the erase (about 50 ms) pauses the
  reading of the file, the movement queue bridges that.

  Without an SPI flash chip there's no place for the records: recording
  stays off.
*/

#include "power_loss.h"

#ifdef POWER_LOSS_RECOVERY

#include <string.h>
#include "sd.h"
#include "dda.h"
#include "dda_queue.h"
#include "clock.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "settings.h"
#include "temp.h"
#include "heater.h"
#include "home.h"
#include "gcode_parse.h"
#include "flash_store.h"
#include "watchdog.h"
#include "atomic.h"

#ifndef PLR_INTERVAL
  #define PLR_INTERVAL        30
#endif
#ifndef PLR_Z_RAISE
  #define PLR_Z_RAISE         2.0
#endif
#ifndef PLR_PURGE_LENGTH
  #define PLR_PURGE_LENGTH    3.0
#endif

#define PLR_MAGIC     0x31524C50UL          // "PLR1": print interrupted.
#define PLR_CLEARED   0x30524C50UL          // "PLR0": nothing to resume.
#define ERASED        0xFFFFFFFFUL
#define SLOT_SIZE     128UL
#define SLOTS         (PLR_FLASH_SIZE / SLOT_SIZE)
#define SLOTS_PER_SECTOR (SPI_FLASH_SECTOR / SLOT_SIZE)

#define FLAG_E_RELATIVE     0x01
#define FLAG_ALL_RELATIVE   0x02

typedef struct {
  uint32_t magic;
  uint32_t seq;
  char     name[32];            ///< File, as given to M23.
  uint32_t file_pos;            ///< Next line to read.
  int32_t  pos[4];              ///< X Y Z E of the queue end, um.
  uint32_t feedrate;            ///< G-code F, mm/min.
  uint16_t hotend, bed;         ///< Target temperatures, quarter degrees.
  uint8_t  fan;                 ///< M106 S.
  uint8_t  flags;
  uint16_t f_multiplier, e_multiplier;   ///< M220, M221 (256 = 100 %).
  uint8_t  reserved[50];
  uint32_t crc;                 ///< CRC32 of everything before.
} plr_record_t;

typedef char plr_record_size_check[sizeof(plr_record_t) == SLOT_SIZE ? 1 : -1];

static uint32_t base;           ///< Flash address of the records, 0 = off.
static uint16_t next_slot;
static uint32_t seq;
static uint8_t pending;         ///< rec holds an interrupted print.
static uint8_t recording;       ///< A record was written for this print.
static uint8_t printing;        ///< A file print is running.
static uint8_t finishing;       ///< File read to the end, queue executing.
static char file_name[32];      ///< File of the running print.
static plr_record_t rec;
static uint32_t last_save_ms;
static int32_t last_z = INT32_MIN;

static uint32_t record_crc(const plr_record_t *r) {
  return flash_store_crc32(r, sizeof(*r) - sizeof(r->crc));
}

static uint32_t slot_addr(uint16_t slot) {
  return base + (uint32_t)slot * SLOT_SIZE;
}

/// Whether a slot is completely erased.
static uint8_t slot_erased(uint16_t slot) {
  uint32_t w[8];
  uint32_t off, i;

  for (off = 0; off < SLOT_SIZE; off += sizeof(w)) {
    spi_flash_read(slot_addr(slot) + off, w, sizeof(w));
    for (i = 0; i < 8; i++)
      if (w[i] != ERASED)
        return 0;
  }
  return 1;
}

/// Write rec (magic and data set) into the next slot.
static void write_record(void) {
  uint16_t tries;

  rec.seq = ++seq;
  rec.crc = record_crc(&rec);
  // Skip slots damaged by a power loss during programming.
  for (tries = 0; tries < SLOTS; tries++) {
    if (next_slot % SLOTS_PER_SECTOR == 0)
      spi_flash_erase_sector(slot_addr(next_slot));
    if (slot_erased(next_slot))
      break;
    next_slot = (uint16_t)((next_slot + 1) % SLOTS);
  }
  spi_flash_program(slot_addr(next_slot), &rec, sizeof(rec));
  next_slot = (uint16_t)((next_slot + 1) % SLOTS);
}

void plr_init(void) {
  plr_record_t r;
  uint16_t slot, newest = 0;
  uint8_t found = 0;

  base = 0;
  if ( ! spi_flash_present() || spi_flash_size() < 4 * PLR_FLASH_SIZE) {
    serial_writestr("echo:Power loss recovery needs the SPI flash, off\n");
    return;
  }
  base = spi_flash_size() - PLR_FLASH_SIZE;

  for (slot = 0; slot < SLOTS; slot++) {
    spi_flash_read(slot_addr(slot), &r, sizeof(r));
    if ((r.magic != PLR_MAGIC && r.magic != PLR_CLEARED) ||
        r.crc != record_crc(&r))
      continue;
    if ( ! found || (int32_t)(r.seq - seq) > 0) {
      seq = r.seq;
      newest = slot;
      memcpy(&rec, &r, sizeof(rec));
      found = 1;
    }
  }
  next_slot = found ? (uint16_t)((newest + 1) % SLOTS) : 0;
  pending = found && rec.magic == PLR_MAGIC;
  if (pending) {
    plr_report();
    serial_writestr("//action:notification Power loss: M1000 resumes\n");
  }
}

uint8_t plr_pending(void) {
  return pending;
}

void plr_report(void) {
  if (pending) {
    serial_writestr("echo:Power loss recovery: ");
    serial_writestr(rec.name);
    sersendf_P((" at byte %lu, Z%lq. M1000 resumes, M1000 C discards\n"),
               rec.file_pos, rec.pos[Z]);
  }
  else {
    serial_writestr("echo:No interrupted print\n");
  }
}

void plr_clear(void) {
  if ( ! base || ( ! pending && ! recording))
    return;
  memset(&rec, 0, sizeof(rec));
  rec.magic = PLR_CLEARED;
  write_record();
  pending = 0;
  recording = 0;
  printing = 0;
  finishing = 0;
  last_z = INT32_MIN;
}

void plr_file_done(void) {
  finishing = 1;
}

plr_line_t plr_line;

void plr_line_begin(void) {
  uint8_t i;

  plr_line.file_pos = sd_position();
  for (i = X; i < AXIS_COUNT; i++)
    plr_line.pos[i] = startpoint.axis[i];
  plr_line.F = next_target.target.F;
  if ( ! printing) {
    // The file name, while it's open (it may be closed at the end).
    const char *name = sd_file_name();

    for (i = 0; name[i] && i < sizeof(file_name) - 1; i++)
      file_name[i] = name[i];
    file_name[i] = '\0';
  }
  printing = 1;
  finishing = 0;
}

void plr_tick(void) {
  uint32_t now = clock_millis();
  plr_line_t cur;
  DDA *d;
  uint8_t i, have = 0;

  if ( ! base || ! printing)
    return;
  // The whole file is read long before the print ends: done when the
  // queue is empty.
  if (finishing && queue_free() == MOVEBUFFER_SIZE - 1 && mb_tail_dda == NULL) {
    plr_clear();
    return;
  }
  // Paused (M25) or off: no records.
  if ( ! settings.plr_enabled ||
      ! (finishing || (gcode_sources & GCODE_SOURCE_SD)))
    return;

  // The line of the move executing now. The queue holds many moves read
  // ahead: printing again from this line loses none of them.
  ATOMIC_START();
    d = mb_tail_dda;
    if (d && d->plr.file_pos != 0xFFFFFFFFUL) {
      cur = d->plr;
      have = 1;
    }
  ATOMIC_END();
  if ( ! have) {
    // Nothing from the file moving: the end of the queue, if empty.
    if (finishing || queue_free() != MOVEBUFFER_SIZE - 1 || mb_tail_dda != NULL)
      return;
    plr_line_begin();
    cur = plr_line;
  }

  if ( ! ((cur.pos[Z] > last_z && now - last_save_ms >= 2000) ||
          now - last_save_ms >= PLR_INTERVAL * 1000UL))
    return;

  memset(&rec, 0, sizeof(rec));
  rec.magic = PLR_MAGIC;
  memcpy(rec.name, file_name, sizeof(rec.name));
  rec.file_pos = cur.file_pos;
  for (i = X; i < AXIS_COUNT; i++)
    rec.pos[i] = cur.pos[i];
  rec.feedrate = cur.F;
  #ifdef HEATER_EXTRUDER
    rec.hotend = temp_get_target(TEMP_SENSOR_extruder);
  #endif
  #ifdef HEATER_BED
    rec.bed = temp_get_target(TEMP_SENSOR_bed);
  #endif
  #ifdef HEATER_FAN
    rec.fan = fan_get();
  #endif
  if (next_target.option_e_relative)
    rec.flags |= FLAG_E_RELATIVE;
  if (next_target.option_all_relative)
    rec.flags |= FLAG_ALL_RELATIVE;
  rec.f_multiplier = (uint16_t)next_target.target.f_multiplier;
  rec.e_multiplier = (uint16_t)next_target.target.e_multiplier;
  write_record();

  recording = 1;
  last_save_ms = now;
  last_z = cur.pos[Z];
}

/// Queue a move to X, Y, Z (um), E relative 'de' um, at 'feed' mm/min.
static void move(int32_t x, int32_t y, int32_t z, int32_t de, uint32_t feed) {
  TARGET t = startpoint;

  t.axis[X] = x;
  t.axis[Y] = y;
  t.axis[Z] = z;
  t.axis[E] = de;
  t.e_relative = 1;
  t.e_multiplier = 256;
  t.f_multiplier = 256;
  t.F = feed;
  enqueue(&t);
}

uint8_t plr_resume(void) {
  int32_t z_up;
  uint8_t i;

  if ( ! pending) {
    plr_report();
    return 0;
  }
  serial_writestr("echo:Resuming ");
  serial_writestr(rec.name);
  serial_writechar('\n');
  queue_wait();

  // The file first: nothing moves if it's gone.
  sd_open(rec.name);
  if ( ! sd_seek(rec.file_pos)) {
    serial_writestr("echo:Resume failed, file not found\n");
    return 0;
  }

  #ifdef HEATER_BED
    if (rec.bed) {
      temp_set(TEMP_SENSOR_bed, rec.bed);
      temp_wait_sensor(TEMP_SENSOR_bed, 1);
    }
  #endif
  #ifdef HEATER_EXTRUDER
    if (rec.hotend) {
      temp_set(TEMP_SENSOR_extruder, rec.hotend);
      temp_wait_sensor(TEMP_SENSOR_extruder, 1);
    }
  #endif
  if (temp_wait_cancelled()) {
    serial_writestr("echo:Resume cancelled\n");
    return 0;
  }

  // Z as it was (the nozzle sits on the print), up, home X and Y.
  for (i = X; i < AXIS_COUNT; i++)
    startpoint.axis[i] = rec.pos[i];
  startpoint.e_relative = 0;
  dda_new_startpoint();
  axes_homed |= HOMED_Z;
  z_up = rec.pos[Z] + (int32_t)(PLR_Z_RAISE * 1000.);
  #ifdef Z_MAX
    if (z_up > (int32_t)(Z_MAX * 1000.) + home_offset[Z])
      z_up = (int32_t)(Z_MAX * 1000.) + home_offset[Z];
  #endif
  move(rec.pos[X], rec.pos[Y], z_up, 0, settings.max_feedrate[Z]);
  queue_wait();
  #if defined X_MIN_PIN
    home_x_negative();
  #elif defined X_MAX_PIN
    home_x_positive();
  #endif
  #if defined Y_MIN_PIN
    home_y_negative();
  #elif defined Y_MAX_PIN
    home_y_positive();
  #endif

  // Prime, back, down.
  move(startpoint.axis[X], startpoint.axis[Y], z_up,
       (int32_t)(PLR_PURGE_LENGTH * 1000.), 180);
  move(rec.pos[X], rec.pos[Y], z_up, 0, settings.max_feedrate[X]);
  move(rec.pos[X], rec.pos[Y], rec.pos[Z], 0, settings.max_feedrate[Z]);
  queue_wait();

  // G-code state as stored. The relative prime didn't count for E.
  startpoint.axis[E] = rec.pos[E];
  startpoint.e_relative = (rec.flags & FLAG_E_RELATIVE) ? 1 : 0;
  if (startpoint.e_relative)
    startpoint.axis[E] = 0;
  startpoint.f_multiplier = rec.f_multiplier ? rec.f_multiplier : 256;
  startpoint.e_multiplier = rec.e_multiplier ? rec.e_multiplier : 256;
  dda_new_startpoint();
  next_target.option_e_relative = (rec.flags & FLAG_E_RELATIVE) ? 1 : 0;
  next_target.option_all_relative = (rec.flags & FLAG_ALL_RELATIVE) ? 1 : 0;
  for (i = X; i < AXIS_COUNT; i++)
    next_target.target.axis[i] = startpoint.axis[i];
  next_target.target.F = rec.feedrate;
  next_target.target.f_multiplier = startpoint.f_multiplier;
  next_target.target.e_multiplier = startpoint.e_multiplier;
  #ifdef HEATER_FAN
    fan_set(rec.fan);
  #endif

  pending = 0;
  recording = 1;
  last_save_ms = clock_millis();
  last_z = rec.pos[Z];
  gcode_sources |= GCODE_SOURCE_SD;
  serial_writestr("echo:Print resumed\n");
  return 1;
}

#endif /* POWER_LOSS_RECOVERY */
