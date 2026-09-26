/** \file
  \brief Independent watchdog (IWDG), 500 ms timeout.

  LSI runs at ~32 kHz, prescaler 32 gives ~1 kHz, reload 500 -> ~500 ms.
  LSI tolerance is large (17..47 kHz), so the real timeout is 340..940 ms.
*/

#include "watchdog.h"

#ifdef USE_WATCHDOG

void wd_init(void) {
  IWDG->KR = 0x5555;                // Unlock PR and RLR.
  IWDG->PR = IWDG_PR_PR_1 | IWDG_PR_PR_0;   // Prescaler 32.
  IWDG->RLR = 500;
  IWDG->KR = 0xAAAA;                // Reload.
  IWDG->KR = 0xCCCC;                // Start.
}

void wd_reset(void) {
  IWDG->KR = 0xAAAA;
}

#endif /* USE_WATCHDOG */
