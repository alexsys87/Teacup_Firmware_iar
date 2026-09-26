/** \file
  \brief Manage heaters, including PID and PWM.

  Heater outputs can be operated in three ways:

   - Hardware PWM on pins with a timer channel (TIM1..4, TIM9..11), frequency
     as configured with the 'pwm' value of DEFINE_HEATER().
   - Software PWM (sigma-delta, 10 ms tick) on pins without timer channel.
   - On/off.

  Tests to pass when operating the heater via M106, temp sensors disabled,
  for each of the three ways above, not inverted and inverted:

   - Heater full on with M106 S255.
   - Heater full off with M106 S0.
   - Heater 10% on with M106 S25 on PWM pins.
   - Heater full off after reset.
*/

#include "heater.h"

#include <stdlib.h>
#include "arch.h"
#include "pinio.h"
#include "debug.h"
#include "crc.h"
#include "sersendf.h"

/** \def PWM_SCALE

  G-code gives heater settings between 0 (off) and 255 (full on). Timers
  count 0..PWM_SCALE-1, the prescaler is calculated from the configured
  frequency: f = F_CPU / (PSC + 1) / PWM_SCALE. With 84 MHz this gives
  1.3 Hz .. 82 kHz.
*/
#define PWM_SCALE 1020

/** \struct heater_definition_t

  Everything needed to operate a heater after initialisation, where the
  #include "config_wrapper.h" trick is no longer available.
*/
typedef struct {
  uint32_t    timer;        ///< Timer base address, 0 = no hardware PWM.
  uint32_t    freq;         ///< PWM frequency in Hz (hardware PWM only).
  uint16_t    max_value;    ///< Hardware/on-off: percent * 256 / 100,
                            ///< software PWM: 255 * 100 / percent.
  uint8_t     pin_id;       ///< PIN_ID() of the output pin.
  uint8_t     channel;      ///< Timer channel 1..4.
  uint8_t     af;           ///< Alternate function of the timer output.
  uint8_t     complementary;        ///< 1 = complementary output (TIM1 CHxN).
  uint8_t     pwm_type;     ///< NO_PWM, SOFTWARE_PWM, HARDWARE_PWM.
  uint8_t     invert;       ///< Wether the heater pin signal is inverted.
} heater_definition_t;

/// PWM type for a DEFINE_HEATER() line.
#define HEATER_PWM_TYPE(pwm, pin) \
  (((pwm) >= HARDWARE_PWM_START) ? \
     ((PIN_TIMER(pin) != TIMER_NONE) ? HARDWARE_PWM : SOFTWARE_PWM) : (pwm))

#undef DEFINE_HEATER_ACTUAL
#define DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_value)            \
  {                                                                        \
    (HEATER_PWM_TYPE(pwm, pin) == HARDWARE_PWM) ?                          \
      (uint32_t)PIN_TIMER(pin) : 0UL,                                      \
    (uint32_t)(pwm),                                                       \
    (HEATER_PWM_TYPE(pwm, pin) == SOFTWARE_PWM) ?                          \
      (uint16_t)(255UL * 100UL / (max_value)) :                            \
      (uint16_t)(((max_value) * 64UL + 12UL) / 25UL),                      \
    PIN_ID(pin),                                                           \
    PIN_CHANNEL(pin),                                                      \
    PIN_AF(pin),                                                           \
    PIN_COMPL(pin),                                                        \
    HEATER_PWM_TYPE(pwm, pin),                                             \
    (invert) ? 1 : 0                                                       \
  },
static const heater_definition_t heaters[NUM_HEATERS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_HEATER_ACTUAL

// We test any heater if we need software-pwm
#define DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, ...) \
  | (HEATER_PWM_TYPE(pwm, pin) == SOFTWARE_PWM)
static const uint8_t software_pwm_needed = 0
  #include "config_wrapper.h"
;
#undef DEFINE_HEATER_ACTUAL

/**
\var heaters_pid
\brief this struct holds the heater PID factors

PID is a fascinating way to control any closed loop control, combining the error (P), cumulative error (I) and rate at which we're approacing the setpoint (D) in such a way that when correctly tuned, the system will achieve target temperature quickly and with little to no overshoot

At every sample, we calculate \f$OUT = k_P (S - T) + k_I \int (S - T) + k_D \frac{dT}{dt}\f$ where S is setpoint and T is temperature.

See http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD for the full story
*/
static struct {
  int32_t   p_factor; ///< scaled P factor: mibicounts/qc
  int32_t   i_factor; ///< scaled I factor: mibicounts/(qC*qs)
  int32_t   d_factor; ///< scaled D factor: mibicounts/(qc/(TH_COUNT*qs))
  int16_t   i_limit;  ///< scaled I limit, such that \f$-i_{limit} < i_{factor} < i_{limit}\f$
} heaters_pid[NUM_HEATERS];

heater_runtime_t heaters_runtime[NUM_HEATERS];
soft_pwm_runtime_t soft_pwm_runtime[NUM_HEATERS];

/// Capture compare register of a timer channel.
static volatile uint32_t *heater_ccr(const heater_definition_t *h) {
  TIM_TypeDef *tim = (TIM_TypeDef *)h->timer;
  return &tim->CCR1 + (h->channel - 1);
}

/// Switch on the clock of a PWM timer.
void timer_clock_on(uint32_t timer) {
  switch (timer) {
    case TIM1_BASE:  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  break;
    case TIM2_BASE:  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  break;
    case TIM3_BASE:  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  break;
    case TIM4_BASE:  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  break;
    case TIM9_BASE:  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;  break;
    case TIM10_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; break;
    case TIM11_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; break;
    default: break;
  }
  (void)RCC->APB2ENR;
}

/** Initialise heater subsystem.

  PWM mode 1 is used: the output is active while CNT < CCR. So CCR = 0 is
  full off, CCR = PWM_SCALE is full on. Inverted heaters get the output
  polarity inverted in hardware, off is high then.

  Timers are fully configured before the pin gets switched to the timer
  output, so there is no glitch at startup. All timers on APB1 and APB2
  count at F_CPU (APB1 prescaler 2 doubles the timer clock).

  Heaters on the same timer share one frequency; the last defined wins.
*/
void heater_init(void) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    const heater_definition_t *h = &heaters[i];
    GPIO_TypeDef *port = PIN_ID_PORT(h->pin_id);
    uint32_t pin = PIN_ID_NUM(h->pin_id);

    if (h->pwm_type == HARDWARE_PWM) {
      TIM_TypeDef *tim = (TIM_TypeDef *)h->timer;
      uint32_t freq = h->freq ? h->freq : 1;
      uint32_t psc = (F_CPU + (PWM_SCALE * freq) / 2) / (PWM_SCALE * freq);
      uint32_t shift = ((h->channel - 1) & 1) * 8;
      volatile uint32_t *ccmr = (h->channel <= 2) ? &tim->CCMR1 : &tim->CCMR2;
      uint32_t ccer_shift = (h->channel - 1) * 4;
      uint32_t ccer;

      if (psc < 1)
        psc = 1;
      if (psc > 65536)
        psc = 65536;

      timer_clock_on(h->timer);

      tim->PSC = psc - 1;
      tim->ARR = PWM_SCALE - 1;
      *heater_ccr(h) = 0;
      // PWM mode 1 (OCxM = 110), preload enable.
      *ccmr = (*ccmr & ~(0xFFUL << shift)) |
              ((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE) << shift);
      if (h->complementary)   // CCxNE, polarity CCxNP.
        ccer = (TIM_CCER_CC1NE | (h->invert ? TIM_CCER_CC1NP : 0)) << ccer_shift;
      else            // CCxE, polarity CCxP.
        ccer = (TIM_CCER_CC1E | (h->invert ? TIM_CCER_CC1P : 0)) << ccer_shift;
      tim->CCER = (tim->CCER & ~(0xFUL << ccer_shift)) | ccer;
      if (h->timer == TIM1_BASE)
        tim->BDTR |= TIM_BDTR_MOE;              // Advanced timer: main output.
      tim->CR1 |= TIM_CR1_ARPE;
      tim->EGR = TIM_EGR_UG;
      tim->CR1 |= TIM_CR1_CEN;

      gpio_pull(port, pin, GPIO_PULL_NONE);
      gpio_open_drain(port, pin, 0);
      gpio_af(port, pin, h->af);
      gpio_speed(port, pin, GPIO_SPEED_LOW);
      gpio_mode(port, pin, GPIO_MODE_AF);
    }
    else {
      gpio_write(port, pin, h->invert);           // Off.
      pin_id_output(h->pin_id);
    }
  }

  pid_init();
}

/** Set a heater output.

  \param index The heater we're setting the output for.

  \param value The PWM value to write, range 0 (off) to 255 (full on).

  This function is called by M106 or, if a temp sensor is connected to the
  heater, every few milliseconds by its PID handler. Using M106 on an output
  with a sensor changes its setting only for a short moment.
*/
static void do_heater(heater_t index, uint8_t value) {
  const heater_definition_t *h;

  if (index >= NUM_HEATERS)
    return;

  h = &heaters[index];
  if (h->pwm_type == HARDWARE_PWM) {
    // max_value is 0..256, product max. 255 * 256 * 1020 fits 32 bits.
    *heater_ccr(h) = ((uint32_t)value * h->max_value * PWM_SCALE) /
                     (255UL * 256UL);

    if (DEBUG_PID && (debug_flags & DEBUG_PID))
      sersendf_P(("PWM %su = %lu\n"), index, *heater_ccr(h));
  }
  else {
    uint8_t on = (value >= HEATER_THRESHOLD);

    gpio_write(PIN_ID_PORT(h->pin_id), PIN_ID_NUM(h->pin_id),
               on ^ h->invert);
  }

  if (value)
    power_on();
}

/** Inititalise PID data structures.
*/
void pid_init(void) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    #ifdef HEATER_SANITY_CHECK
      // 0 is a "sane" temperature when we're trying to cool down.
      heaters_runtime[i].sane_temperature = 0;
    #endif

    heaters_pid[i].p_factor = DEFAULT_P;
    heaters_pid[i].i_factor = DEFAULT_I;
    heaters_pid[i].d_factor = DEFAULT_D;
    heaters_pid[i].i_limit = DEFAULT_I_LIMIT;
  }
}

/** \brief run heater PID algorithm
  \param h which heater we're running the loop for
  \param type which temp sensor type this heater is attached to
  \param current_temp the temperature that the associated temp sensor is reporting
  \param target_temp the temperature we're trying to achieve
*/
void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp) {
  uint8_t pid_output;
  uint8_t bang_bang = 0;
  uint16_t bb_hysteresis = 0;
  uint8_t bb_on = 255, bb_off = 0;

  (void)type;

  if (h >= NUM_HEATERS)
    return;

  if (target_temp == 0) {
    heater_set(h, 0);
    return;
  }

  #ifdef BANG_BANG
    // All heaters on/off.
    bang_bang = 1;
    bb_hysteresis = TEMP_HYSTERESIS * 4;
    bb_on = BANG_BANG_ON;
    bb_off = BANG_BANG_OFF;
  #endif
  #if defined BANG_BANG_BED && defined HEATER_BED
    // Only the bed on/off, like Marlin without PIDTEMPBED.
    if (h == HEATER_BED) {
      bang_bang = 1;
      bb_hysteresis = BANG_BANG_BED_HYSTERESIS * 4;
      bb_on = BANG_BANG_BED_ON;
      bb_off = 0;
    }
  #endif

  if (bang_bang) {
    // Keep this heater's previous output inside the hysteresis window.
    // (The original Teacup kept one shared value for all heaters.)
    pid_output = heaters_runtime[h].heater_output;
    if ((uint32_t)current_temp >= (uint32_t)target_temp + bb_hysteresis)
      pid_output = bb_off;
    else if ((int32_t)current_temp <= (int32_t)target_temp - bb_hysteresis)
      pid_output = bb_on;
  }
  #ifndef BANG_BANG
  else {
    int16_t   heater_p;
    int16_t   heater_d;
    int16_t   t_error = target_temp - current_temp;

    heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer++] = current_temp;
    heaters_runtime[h].temp_history_pointer &= (TH_COUNT - 1);

    // PID stuff
    // proportional
    heater_p = t_error; // Units: qC where 4qC=1C

    // integral
    heaters_runtime[h].heater_i += t_error;  // Units: qC*qs where 16qC*qs=1C*s
    // prevent integrator wind-up
    if (heaters_runtime[h].heater_i > heaters_pid[h].i_limit)
      heaters_runtime[h].heater_i = heaters_pid[h].i_limit;
    else if (heaters_runtime[h].heater_i < -heaters_pid[h].i_limit)
      heaters_runtime[h].heater_i = -heaters_pid[h].i_limit;

    // derivative.  Units: qC/(TH_COUNT*qs) where 1C/s=TH_COUNT*4qC/4qs=8qC/qs)
    // note: D follows temp rather than error so there's no large derivative when the target changes
    heater_d = heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer] - current_temp;

    // combine factors
    {
      int32_t pid_output_intermed = ( // Units: counts
        (((int32_t) heater_p) * heaters_pid[h].p_factor) +
        (((int32_t) heaters_runtime[h].heater_i) * heaters_pid[h].i_factor) +
        (((int32_t) heater_d) * heaters_pid[h].d_factor)
      ) / PID_SCALE;

      // rebase and limit factors
      if (pid_output_intermed > 255) {
        if (t_error > 0)
          heaters_runtime[h].heater_i -= t_error; // un-integrate
        pid_output = 255;
      }
      else if (pid_output_intermed < 0) {
        if (t_error < 0)
          heaters_runtime[h].heater_i -= t_error; // un-integrate
        pid_output = 0;
      }
      else
        pid_output = pid_output_intermed & 0xFF;

      if (DEBUG_PID && (debug_flags & DEBUG_PID))
        sersendf_P(("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, heaters_pid[h].p_factor, (int32_t) heater_p * heaters_pid[h].p_factor / PID_SCALE, heaters_runtime[h].heater_i, heaters_pid[h].i_factor, (int32_t) heaters_runtime[h].heater_i * heaters_pid[h].i_factor / PID_SCALE, heater_d, heaters_pid[h].d_factor, (int32_t) heater_d * heaters_pid[h].d_factor / PID_SCALE, pid_output_intermed, pid_output);
    }
  }
  #else
  else {
    pid_output = 0;             // Not reached, all heaters are bang-bang.
  }
  #endif

  #ifdef HEATER_SANITY_CHECK
  // check heater sanity
  // implementation is a moving window with some slow-down to compensate for thermal mass
  if (target_temp > (current_temp + (TEMP_HYSTERESIS*4))) {
    // heating
    if (current_temp > heaters_runtime[h].sane_temperature)
      // hotter than sane- good since we're heating unless too hot
      heaters_runtime[h].sane_temperature = current_temp;
    else {
      if (heaters_runtime[h].sanity_counter < 40)
        heaters_runtime[h].sanity_counter++;
      else {
        heaters_runtime[h].sanity_counter = 0;
        // ratchet up expected temp
        heaters_runtime[h].sane_temperature++;
      }
    }
    // limit to target, so if we overshoot by too much for too long an error is flagged
    if (heaters_runtime[h].sane_temperature > target_temp)
      heaters_runtime[h].sane_temperature = target_temp;
  }
  else if (target_temp < (current_temp - (TEMP_HYSTERESIS*4))) {
    // cooling
    if (current_temp < heaters_runtime[h].sane_temperature)
      // cooler than sane- good since we're cooling
      heaters_runtime[h].sane_temperature = current_temp;
    else {
      if (heaters_runtime[h].sanity_counter < 125)
        heaters_runtime[h].sanity_counter++;
      else {
        heaters_runtime[h].sanity_counter = 0;
        // ratchet down expected temp
        heaters_runtime[h].sane_temperature--;
      }
    }
    // if we're at or below 60 celsius, don't freak out if we can't drop any more.
    if (current_temp <= 240)
      heaters_runtime[h].sane_temperature = current_temp;
    // limit to target, so if we don't cool down for too long an error is flagged
    else if (heaters_runtime[h].sane_temperature < target_temp)
      heaters_runtime[h].sane_temperature = target_temp;
  }
  // we're within HYSTERESIS of our target
  else {
    heaters_runtime[h].sane_temperature = current_temp;
    heaters_runtime[h].sanity_counter = 0;
  }

  // compare where we're at to where we should be
  if (labs((int16_t)(current_temp - heaters_runtime[h].sane_temperature)) > (TEMP_HYSTERESIS*4)) {
    // no change, or change in wrong direction for a long time- heater is broken!
    pid_output = 0;
    sersendf_P(("!! heater %d or its temp sensor broken - temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), h, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[h].sane_temperature >> 2, (heaters_runtime[h].sane_temperature & 3) * 25, heaters_runtime[h].sanity_counter);
  }
  #endif /* HEATER_SANITY_CHECK */

  heater_set(h, pid_output);
}

/** \brief software PWM routine
*/
static void heater_soft_pwm(heater_t index) {
  int16_t pwm = heaters_runtime[index].heater_output;

  // full off? then put it just off
  if (pwm == 0) {
    do_heater(index, 0);
    return;
  }

  // Sigma-delta: sd_accu += pwm - sd_dir, where sd_dir is 0 or the
  // precalculated 255 * 100 / max_value(%). So heaters with a max_value
  // below 100% get a proportionally smaller duty.
  soft_pwm_runtime[index].sd_accu += pwm - soft_pwm_runtime[index].sd_dir;

  if (soft_pwm_runtime[index].sd_accu > 0) {
    soft_pwm_runtime[index].sd_dir = heaters[index].max_value;
    do_heater(index, 255);
  }
  else {
    soft_pwm_runtime[index].sd_dir = 0;
    do_heater(index, 0);
  }
}

/**
  Called every 10ms from clock.c. Tick the softPWM procedure when the heater needs it.
*/
void soft_pwm_tick(void) {
  if (software_pwm_needed) {
    uint8_t i;
    for (i = 0; i < NUM_HEATERS; i++) {
      if (heaters[i].pwm_type == SOFTWARE_PWM)
        heater_soft_pwm((heater_t)i);
    }
  }
}

/** \brief set heater value and execute it
*/
void heater_set(heater_t index, uint8_t value) {
  if (index >= NUM_HEATERS)
    return;

  heaters_runtime[index].heater_output = value;
  // Software PWM heaters are driven by soft_pwm_tick().
  if (heaters[index].pwm_type != SOFTWARE_PWM || value == 0)
    do_heater(index, value);
  else if (value)
    power_on();
}

/** \brief switch all heaters off
*/
void heater_all_off(void) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    heaters_runtime[i].heater_output = 0;
    do_heater((heater_t)i, 0);
  }
}

void heater_emergency_off(void) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    const heater_definition_t *h = &heaters[i];

    heaters_runtime[i].heater_output = 0;
    if (h->pwm_type == HARDWARE_PWM) {
      TIM_TypeDef *tim = (TIM_TypeDef *)h->timer;
      uint32_t shift = ((h->channel - 1) & 1) * 8;
      volatile uint32_t *ccmr = (h->channel <= 2) ? &tim->CCMR1 : &tim->CCMR2;

      // OCxM = 100: force inactive level (polarity/inversion still applies).
      *ccmr = (*ccmr & ~(TIM_CCMR1_OC1M << shift)) | (TIM_CCMR1_OC1M_2 << shift);
      *heater_ccr(h) = 0;
    }
    else {
      gpio_write(PIN_ID_PORT(h->pin_id), PIN_ID_NUM(h->pin_id), h->invert);
    }
  }
}

uint8_t heater_uses_timer(uint32_t timer) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++)
    if (heaters[i].pwm_type == HARDWARE_PWM && heaters[i].timer == timer)
      return 1;
  return 0;
}

/** \brief check whether all heaters are off
*/
uint8_t heaters_all_zero(void) {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    if (heaters_runtime[i].heater_output)
      return 0;
  }
  return 255;
}

/** \brief set heater P factor
  \param index heater to change factor for
  \param p scaled P factor
*/
void pid_set_p(heater_t index, int32_t p) {
  if (index >= NUM_HEATERS)
    return;

  heaters_pid[index].p_factor = p;
}

/** \brief set heater I factor
  \param index heater to change I factor for
  \param i scaled I factor
*/
void pid_set_i(heater_t index, int32_t i) {
  if (index >= NUM_HEATERS)
    return;

  heaters_pid[index].i_factor = i;
}

/** \brief set heater D factor
  \param index heater to change D factor for
  \param d scaled D factor
*/
void pid_set_d(heater_t index, int32_t d) {
  if (index >= NUM_HEATERS)
    return;

  heaters_pid[index].d_factor = d;
}

/** \brief set heater I limit
  \param index heater to set I limit for
  \param i_limit scaled I limit
*/
void pid_set_i_limit(heater_t index, int32_t i_limit) {
  if (index >= NUM_HEATERS)
    return;

  if (i_limit > 32767)
    i_limit = 32767;
  if (i_limit < 0)
    i_limit = 0;
  heaters_pid[index].i_limit = (int16_t)i_limit;
}

/** \brief read PID values, Teacup units
*/
void pid_get(heater_t index, int32_t *p, int32_t *i, int32_t *d, int32_t *i_limit) {
  if (index >= NUM_HEATERS) {
    *p = *i = *d = *i_limit = 0;
    return;
  }
  *p = heaters_pid[index].p_factor;
  *i = heaters_pid[index].i_factor;
  *d = heaters_pid[index].d_factor;
  *i_limit = heaters_pid[index].i_limit;
}

/** \brief send heater debug info to host
  \param i index of heater to send info for
*/
void heater_print(uint16_t i) {
  if (i >= NUM_HEATERS)
    return;

  sersendf_P(("P:%ld I:%ld D:%ld Ilim:%u crc:%u\n"), heaters_pid[i].p_factor,
             heaters_pid[i].i_factor, heaters_pid[i].d_factor,
             heaters_pid[i].i_limit, crc_block(&heaters_pid[i].p_factor, 14));
}
