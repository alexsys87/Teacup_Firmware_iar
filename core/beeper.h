/** \file
  \brief Beeper for M300.
*/

#ifndef _BEEPER_H
#define _BEEPER_H

#include <stdint.h>

/// Set up the beeper pin (BEEPER_PIN), if any.
void beeper_init(void);

/**
  Play a tone. Keeps the clock running (heaters, watchdog, host
  communication) while playing.

  \param freq     Frequency in Hz, 0 = silence (just wait).
  \param duration Duration in ms, limited to 5000.
*/
void beeper_tone(uint16_t freq, uint16_t duration);

#endif /* _BEEPER_H */
