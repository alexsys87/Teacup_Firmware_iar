/** \file
  \brief Print job timer and statistics (M31, M75..M78), like Marlin's
  print job timer and PRINTCOUNTER.
*/

#ifndef _PRINT_STATS_H
#define _PRINT_STATS_H

#include <stdint.h>

/// Load the statistics (startup, after the settings).
void job_init(void);

/// M75, M24, M109 / M190 with a target: start or continue the timer.
void job_start(void);

/// M76, M25: pause the timer.
void job_pause(void);

/// M77, end of an SD / flash file: stop, the print counts as finished.
void job_stop(void);

/**
  Stop as soon as the movement queue is empty: the end of a file or
  M104 S0 are read long before the last moves are done.
*/
void job_stop_idle(void);

/// Whether the timer runs.
uint8_t job_running(void);

/// Whether the job is paused (M25, M76).
uint8_t job_paused(void);

/// Time of the current / last print, s.
uint32_t job_elapsed_s(void);

/// Filament fed by a move while the timer runs, um (dda_create()).
void job_add_filament(uint32_t um);

/// M31: time of the current / last print.
void job_report_time(void);

/// M78: statistics. M78 S78: reset them.
void job_report_stats(void);
void job_reset_stats(void);

/// Main loop: store changed statistics once the printer stands still.
void job_tick(void);

#endif /* _PRINT_STATS_H */
