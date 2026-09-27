/** \file
  \brief Spindle or laser (SPINDLE_LASER), see spindle.h.

  Output: hardware PWM on SPINDLE_LASER_PWM_PIN (a pin with a timer
  channel), SPINDLE_LASER_PWM_FREQ Hz, 0..SPINDLE_PWM_SCALE counts;
  optional enable (SPINDLE_LASER_ENA_PIN, on while the power is above 0)
  and direction (SPINDLE_DIR_PIN, M3 = low, M4 = high). S values
  0..SPEED_POWER_MAX map to 0..full power (e.g. 255, 100 or the RPM of the
  spindle at full PWM).

  Spindle (default): M3 / M4 / M5 wait for the queued moves, like Marlin,
  then switch; SPINDLE_POWERUP_DELAY / SPINDLE_POWERDOWN_DELAY ms of dwell
  let the spindle get up to speed or stop.

  Laser (LASER_MODE, Marlin's LASER_POWER_INLINE): the power travels with
  the moves. M3 S / M4 S and S on a G1 line set it for the following
  G1 / G2 / G3 moves, G0 moves run with the laser off; it is switched in
  the step interrupt when a move starts, off when the queue runs empty.
  M4 is the dynamic mode: the power follows the speed (lower while
  accelerating and decelerating), updated every 2 ms in dda_clock().
*/

#include "spindle.h"

#ifdef SPINDLE_LASER

#include "arch.h"
#include "pinio.h"
#include "heater.h"
#include "dda_queue.h"
#include "delay.h"
#include "clock.h"

#ifndef SPINDLE_LASER_PWM_PIN
  #error SPINDLE_LASER needs SPINDLE_LASER_PWM_PIN (a pin with a timer channel).
#endif
#if PIN_TIMER(SPINDLE_LASER_PWM_PIN) == TIMER_NONE
  #error SPINDLE_LASER_PWM_PIN needs a timer channel.
#endif
#ifndef SPINDLE_LASER_PWM_FREQ
  #define SPINDLE_LASER_PWM_FREQ   1000
#endif
#ifndef SPEED_POWER_MAX
  #define SPEED_POWER_MAX          255
#endif
#ifndef SPINDLE_POWERUP_DELAY
  #define SPINDLE_POWERUP_DELAY    0
#endif
#ifndef SPINDLE_POWERDOWN_DELAY
  #define SPINDLE_POWERDOWN_DELAY  0
#endif

/// PWM resolution, counts for full power.
#define SPINDLE_PWM_SCALE 1000

#define PWM_TIM   ((TIM_TypeDef *)PIN_TIMER(SPINDLE_LASER_PWM_PIN))
#define PWM_CH    PIN_CHANNEL(SPINDLE_LASER_PWM_PIN)
#define PWM_CCR   (*(&PWM_TIM->CCR1 + (PWM_CH - 1)))

static uint16_t output;                 ///< Current PWM counts.
#ifdef LASER_MODE
static uint16_t inline_power;           ///< Power of G1 moves, counts.
static uint16_t move_power;             ///< Of moves queued now.
static uint8_t dynamic;                 ///< M4: power follows the speed.
#endif

/// S value to PWM counts.
static uint16_t counts(uint32_t s) {
  if (s >= SPEED_POWER_MAX)
    return SPINDLE_PWM_SCALE;
  return (uint16_t)((s * SPINDLE_PWM_SCALE + SPEED_POWER_MAX / 2) / SPEED_POWER_MAX);
}

static void set_output(uint16_t c) {
  output = c;
  #ifdef SPINDLE_LASER_PWM_INVERT
    PWM_CCR = SPINDLE_PWM_SCALE - c;
  #else
    PWM_CCR = c;
  #endif
  #ifdef SPINDLE_LASER_ENA_PIN
    #ifdef SPINDLE_LASER_ENA_INVERT
      WRITE(SPINDLE_LASER_ENA_PIN, c ? 0 : 1);
    #else
      WRITE(SPINDLE_LASER_ENA_PIN, c ? 1 : 0);
    #endif
  #endif
}

void spindle_init(void) {
  TIM_TypeDef *tim = PWM_TIM;
  uint32_t psc = (F_CPU + (SPINDLE_PWM_SCALE * SPINDLE_LASER_PWM_FREQ) / 2) /
                 (SPINDLE_PWM_SCALE * SPINDLE_LASER_PWM_FREQ);
  uint32_t shift = ((PWM_CH - 1) & 1) * 8;
  volatile uint32_t *ccmr = (PWM_CH <= 2) ? &tim->CCMR1 : &tim->CCMR2;
  uint32_t ccer_shift = (PWM_CH - 1) * 4;

  if (psc < 1)
    psc = 1;
  if (psc > 65536)
    psc = 65536;

  #ifdef SPINDLE_LASER_ENA_PIN
    SET_OUTPUT(SPINDLE_LASER_ENA_PIN);
  #endif
  #ifdef SPINDLE_DIR_PIN
    WRITE(SPINDLE_DIR_PIN, 0);
    SET_OUTPUT(SPINDLE_DIR_PIN);
  #endif

  // Timer first, then the pin, no glitch (see heater_init()).
  timer_clock_on((uint32_t)tim);
  tim->PSC = psc - 1;
  tim->ARR = SPINDLE_PWM_SCALE - 1;
  set_output(0);
  *ccmr = (*ccmr & ~(0xFFUL << shift)) |
          ((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE) << shift);
  tim->CCER = (tim->CCER & ~(0xFUL << ccer_shift)) | (TIM_CCER_CC1E << ccer_shift);
  if ((uint32_t)tim == TIM1_BASE)
    tim->BDTR |= TIM_BDTR_MOE;
  tim->CR1 |= TIM_CR1_ARPE;
  tim->EGR = TIM_EGR_UG;
  tim->CR1 |= TIM_CR1_CEN;
  SET_AF(SPINDLE_LASER_PWM_PIN, PIN_AF(SPINDLE_LASER_PWM_PIN));
}

void spindle_on(uint8_t cw, uint8_t s_seen, uint32_t s) {
  uint16_t c = s_seen ? counts(s) : SPINDLE_PWM_SCALE;

  #ifdef LASER_MODE
    (void)cw;
    inline_power = c;
    dynamic = ! cw;
  #else
    queue_wait();
    #ifdef SPINDLE_DIR_PIN
      WRITE(SPINDLE_DIR_PIN, cw ? 0 : 1);
    #else
      (void)cw;
    #endif
    set_output(c);
    if (c && SPINDLE_POWERUP_DELAY)
      delay_ms(SPINDLE_POWERUP_DELAY);
  #endif
}

void spindle_off(void) {
  #ifdef LASER_MODE
    inline_power = 0;
    dynamic = 0;
  #else
    queue_wait();
    if (output && SPINDLE_POWERDOWN_DELAY) {
      set_output(0);
      delay_ms(SPINDLE_POWERDOWN_DELAY);
    }
    set_output(0);
  #endif
}

void spindle_set_inline(uint32_t s) {
  #ifdef LASER_MODE
    inline_power = counts(s);
  #else
    (void)s;
  #endif
}

void spindle_move_begin(uint8_t g) {
  #ifdef LASER_MODE
    move_power = (g == 0) ? 0 : inline_power;
  #else
    (void)g;
  #endif
}

void spindle_move_end(void) {
  #ifdef LASER_MODE
    move_power = 0;
  #endif
}

uint16_t spindle_move_power(void) {
  #ifdef LASER_MODE
    // Bit 15: dynamic power (M4) for this move.
    return move_power ? (uint16_t)(move_power | (dynamic ? 0x8000 : 0)) : 0;
  #else
    return 0;
  #endif
}

void spindle_apply(uint16_t power) {
  #ifdef LASER_MODE
    set_output(power & 0x7FFF);
  #else
    (void)power;
  #endif
}

void spindle_dynamic(uint16_t power, uint32_t c, uint32_t c_min) {
  #ifdef LASER_MODE
    uint32_t p;

    if ( ! (power & 0x8000) || c == 0)
      return;
    p = (uint32_t)(power & 0x7FFF) * c_min / c;
    set_output((uint16_t)p);
  #else
    (void)power; (void)c; (void)c_min;
  #endif
}

void spindle_emergency_off(void) {
  #ifdef LASER_MODE
    inline_power = move_power = 0;
  #endif
  set_output(0);
}

uint16_t spindle_output(void) {
  return output;
}

#endif /* SPINDLE_LASER */
