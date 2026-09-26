/** \file
  \brief Print job timer and statistics, see print_stats.h.

  M75 (or M24, or heating with M109 / M190, like Marlin's
  PRINTJOB_TIMER_AUTOSTART) starts the timer, M76 (M25) pauses it, M77
  stops it; the end of an SD / flash file or M104 S0 stop it once the
  movement queue is empty. M31 reports
  the time, M78 the statistics: prints started, finished, failed (started
  but never stopped, e.g. power loss or kill), total and longest print
  time, filament used (E moves forward while the timer runs).

  The statistics are stored in the settings sector as a record of their
  own (FLASH_STORE_STATS, see hal/flash_store.c), independent of M500:
  when a print starts and when it ends, as soon as the movement queue is
  empty (writing to internal Flash stalls the CPU briefly).
*/

#include "print_stats.h"

#include <string.h>
#include "clock.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "flash_store.h"
#include "dda_queue.h"

typedef struct {
  uint32_t prints;              ///< Started.
  uint32_t finished;
  uint32_t total_s;             ///< Printing time of all prints.
  uint32_t longest_s;
  uint32_t filament_mm;
} stats_t;

static stats_t stats;
static uint8_t running, paused, dirty, stop_pending;
static uint32_t job_ms;         ///< Time of the current / last print.
static uint32_t start_ms;       ///< clock_millis() when (re)started.
static uint32_t filament_um;    ///< Current print, not yet in stats.

void job_init(void) {
  if ( ! flash_store_read(&stats, sizeof(stats), FLASH_STORE_STATS))
    memset(&stats, 0, sizeof(stats));
}

/// Time of the current print so far, ms.
static uint32_t elapsed_ms(void) {
  return job_ms + (running ? clock_millis() - start_ms : 0);
}

void job_start(void) {
  stop_pending = 0;
  if (running)
    return;
  if ( ! paused) {
    job_ms = 0;
    filament_um = 0;
    stats.prints++;
    dirty = 1;
  }
  running = 1;
  paused = 0;
  start_ms = clock_millis();
}

void job_pause(void) {
  if ( ! running)
    return;
  job_ms = elapsed_ms();
  running = 0;
  paused = 1;
}

void job_stop(void) {
  uint32_t s;

  if ( ! running && ! paused)
    return;
  job_ms = elapsed_ms();
  running = 0;
  paused = 0;
  s = job_ms / 1000;
  stats.finished++;
  stats.total_s += s;
  if (s > stats.longest_s)
    stats.longest_s = s;
  stats.filament_mm += (filament_um + 500) / 1000;
  filament_um = 0;
  dirty = 1;
}

void job_stop_idle(void) {
  if (running || paused)
    stop_pending = 1;
}

uint8_t job_running(void) {
  return running;
}

void job_add_filament(uint32_t um) {
  if (running)
    filament_um += um;
}

/// "1d 2h 3m 4s", leading zero units left out, like Marlin.
static void write_duration(uint32_t s) {
  uint32_t d = s / 86400, h = s / 3600 % 24, m = s / 60 % 60;

  if (d) {
    serwrite_uint32(d);
    serial_writestr("d ");
  }
  if (d || h) {
    serwrite_uint32(h);
    serial_writestr("h ");
  }
  if (d || h || m) {
    serwrite_uint32(m);
    serial_writestr("m ");
  }
  serwrite_uint32(s % 60);
  serial_writechar('s');
}

void job_report_time(void) {
  serial_writestr("echo:Print time: ");
  write_duration(elapsed_ms() / 1000);
  serial_writechar('\n');
}

void job_report_stats(void) {
  uint32_t current = (running || paused) ? 1 : 0;
  uint32_t fil = stats.filament_mm + (filament_um + 500) / 1000;

  sersendf_P(("echo:Stats: Prints: %lu, Finished: %lu, Failed: %lu\n"),
             stats.prints, stats.finished,
             stats.prints - stats.finished - current);
  serial_writestr("echo:Stats: Total time: ");
  write_duration(stats.total_s);
  serial_writestr(", Longest job: ");
  write_duration(stats.longest_s);
  serial_writestr("\necho:Stats: Filament used: ");
  serwrite_uint32(fil / 1000);
  serial_writechar('.');
  serial_writechar((char)('0' + fil / 100 % 10));
  serial_writechar((char)('0' + fil / 10 % 10));
  serial_writestr("m\n");
}

void job_reset_stats(void) {
  memset(&stats, 0, sizeof(stats));
  if (running || paused)
    stats.prints = 1;
  dirty = 1;
}

void job_tick(void) {
  if (queue_free() != MOVEBUFFER_SIZE - 1 || mb_tail_dda != NULL)
    return;
  if (stop_pending) {
    stop_pending = 0;
    job_stop();
  }
  if ( ! dirty)
    return;
  dirty = 0;
  flash_store_write(&stats, sizeof(stats), FLASH_STORE_STATS);
}
