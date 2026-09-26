/** \file
  \brief Beeper for M300.

  BEEPER_PIN drives a passive buzzer (square wave at the requested
  frequency) or, with BEEPER_ACTIVE, an active buzzer (on/off only).
  Without BEEPER_PIN, M300 only waits for the duration.
*/

#include "beeper.h"
#include "config_wrapper.h"
#include "pinio.h"
#include "delay.h"
#include "clock.h"

void beeper_init(void) {
  #ifdef BEEPER_PIN
    WRITE(BEEPER_PIN, 0);
    SET_OUTPUT(BEEPER_PIN);
  #endif
}

void beeper_tone(uint16_t freq, uint16_t duration) {
  uint32_t total_us, elapsed = 0, since_poll = 0, half;
  #if defined BEEPER_PIN && ! defined BEEPER_ACTIVE
    uint8_t level = 0;
  #endif

  if (duration > 5000)
    duration = 5000;
  total_us = (uint32_t)duration * 1000UL;

  if (freq < 20)
    freq = 0;                       // Inaudible, treat as silence.
  if (freq > 10000)
    freq = 10000;
  #if ! defined BEEPER_PIN || defined BEEPER_ACTIVE
    // No square wave needed: wait in 1 ms steps.
    half = 1000;
  #else
    half = freq ? 500000UL / freq : 1000;
  #endif

  #if defined BEEPER_PIN && defined BEEPER_ACTIVE
    if (freq)
      WRITE(BEEPER_PIN, 1);
  #endif

  while (elapsed < total_us) {
    #if defined BEEPER_PIN && ! defined BEEPER_ACTIVE
      if (freq) {
        level ^= 1;
        WRITE(BEEPER_PIN, level);
      }
    #endif
    delay_us(half);
    elapsed += half;
    since_poll += half;
    if (since_poll >= 1000) {
      since_poll = 0;
      clock_poll();
    }
  }

  #ifdef BEEPER_PIN
    WRITE(BEEPER_PIN, 0);
  #endif
}
