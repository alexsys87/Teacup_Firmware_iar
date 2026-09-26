/** \file
  \brief Filament runout sensor (M412) and filament change (M600).
*/

#ifndef _FILAMENT_H
#define _FILAMENT_H

#include <stdint.h>
#include "config_wrapper.h"

/// Parameters of a filament change, um. Defaults: filament_change_defaults().
typedef struct {
  int32_t retract;    ///< Retract before moving away, prime after return.
  int32_t lift;       ///< Z lift, relative.
  int32_t park_x;     ///< Park position (only if X and Y are homed).
  int32_t park_y;
  int32_t unload;     ///< Unload length.
  int32_t load;       ///< Load (and purge) length.
} filament_change_t;

/// Configured defaults (FILAMENT_CHANGE_* in the printer config).
void filament_change_defaults(filament_change_t *p);

/**
  Filament change (M600, runout): retract, lift Z, park, unload, wait for
  M108 / M876 S0 (host "Continue"), load and purge, go back, prime.
  Logical coordinates, E included, are the same afterwards, a print
  continues where it stopped.
*/
void filament_change(const filament_change_t *p);

/// Whether a filament change (pause) is in progress.
uint8_t filament_change_active(void);

#ifdef FILAMENT_RUNOUT_PIN

/// Steps of the E motor fed forward, counted in dda_start().
extern volatile uint32_t filament_e_steps;

void filament_init(void);

/// Every 10 ms: debounce the sensor, detect a runout.
void filament_tick(void);

/**
  Main loop: after a runout, pause the print by a filament change. Between
  two commands, so the G-code state is consistent.
*/
void filament_runout_service(void);

/// Debounced sensor state: 1 = no filament.
uint8_t filament_sensor_runout(void);

/// M412: S0/S1 (-1 = unchanged), D distance in um (-1 = unchanged).
void filament_runout_set(int8_t enable, int32_t distance_um);

/// M412 R: forget a pending runout.
void filament_runout_reset(void);

/// M412 without parameters.
void filament_runout_report(void);

/// Settings (M500).
uint8_t filament_runout_enabled(void);
int32_t filament_runout_distance(void);

#else

#define filament_init()            do { } while (0)
#define filament_tick()            do { } while (0)
#define filament_runout_service()  do { } while (0)

#endif /* FILAMENT_RUNOUT_PIN */

#endif /* _FILAMENT_H */
