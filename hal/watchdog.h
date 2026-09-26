/** \file
  \brief Watchdog - reset if the main loop doesn't run for too long.
*/

#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include "config_wrapper.h"

#ifdef USE_WATCHDOG

/// Start the independent watchdog, timeout 500 ms. Can't be stopped.
void wd_init(void);

/// Reset timeout, must be called at least every 500 ms.
void wd_reset(void);

#else

#define wd_init()  do { } while (0)
#define wd_reset() do { } while (0)

#endif /* USE_WATCHDOG */

#endif /* _WATCHDOG_H */
