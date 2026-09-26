/** \file
  \brief Fatal error handling: switch everything off and halt.
*/

#ifndef _KILL_H
#define _KILL_H

#include <stdint.h>

/// Use as 'id' when no heater/sensor is involved.
#define KILL_NO_ID  (-1)

/// Set once printer_kill() was called.
extern volatile uint8_t printer_killed;

/**
  Switch off heaters and steppers immediately, report the reason to the host
  and halt. Callable from any context, including interrupts.

  \param reason Error text, e.g. "Thermal Runaway, system stopped!".
  \param id     Temperature sensor index for the "Heater_ID:" suffix, or
                KILL_NO_ID.

  In the halted state heaters are kept off and the watchdog is fed. Sending
  "M999" restarts the controller, otherwise only the reset button does.
*/
_Noreturn void printer_kill(const char *reason, int16_t id);

#endif /* _KILL_H */
