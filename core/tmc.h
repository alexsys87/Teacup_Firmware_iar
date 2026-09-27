/** \file
  \brief TMC2208 / TMC2209 stepper drivers configured over UART (TMC_UART).

  Like Marlin's TMC support in UART mode: run current (M906), stealthChop
  or spreadCycle (M569), microsteps (TMC_*_MICROSTEPS), driver status
  (M122). Drivers lose their configuration without motor supply: they're
  checked once a second and configured again after a reset.
*/

#ifndef _TMC_H
#define _TMC_H

#include <stdint.h>
#include "config_wrapper.h"

#ifdef TMC_UART

/// Set up the UART, configure all drivers.
void tmc_init(void);

/// Once a second: configure drivers after a reset (motor supply back).
void tmc_tick(void);

/// Set by power_on(): the drivers need their configuration.
extern volatile uint8_t tmc_power_up;

/// Configure all drivers now (after power on).
void tmc_ready(void);

/// Before queueing a move: configure the drivers if the power just came on.
static inline void tmc_before_move(void) {
  if (tmc_power_up)
    tmc_ready();
}

/// Write currents and chopper modes (settings.tmc_*) to all drivers.
void tmc_apply(void);

/// M122: status of all drivers.
void tmc_report(void);

/// M906 / M569 report lines, for M906, M569 and M503.
void tmc_report_settings(void);

#else

static inline void tmc_init(void) { }
static inline void tmc_tick(void) { }
static inline void tmc_before_move(void) { }
static inline void tmc_apply(void) { }

#endif /* TMC_UART */

#endif /* _TMC_H */
