#ifndef _CLOCK_H
#define _CLOCK_H

#include <stdint.h>

/// Called from the SysTick interrupt every TICK_TIME (2 ms).
void clock_tick(void);

/// Milliseconds since startup (TICK_TIME resolution).
uint32_t clock_millis(void);

/// Do reoccuring stuff. Call it occasionally in busy loops.
void clock_poll(void);

#endif /* _CLOCK_H */
