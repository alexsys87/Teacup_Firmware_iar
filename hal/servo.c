/** \file
  \brief Servo signal for the BLTouch, see servo.h.
*/

#include "servo.h"

#ifdef BLTOUCH_SERVO_PIN

#include "arch.h"
#include "cpu.h"
#include "pinio.h"
#include "heater.h"
#include "serial.h"

#define SERVO_MIN_US   544
#define SERVO_MAX_US   2400

static uint16_t last_angle = 0;
static uint8_t servo_ok = 0;

void servo_init(void) {
  WRITE(BLTOUCH_SERVO_PIN, 0);
  SET_OUTPUT(BLTOUCH_SERVO_PIN);

  // TIM9 counts at F_CPU (APB2 prescaler 1).
  if (heater_uses_timer(TIM9_BASE)) {
    serial_writestr("echo:Servo: TIM9 is used by a heater, no servo\n");
    return;
  }
  #ifdef STEP_TIMER_PULSES
    {
      uint8_t g;

      for (g = 0; g < 4; g++)
        if ((uint32_t)step_timer[g] == TIM9_BASE) {
          serial_writestr("echo:Servo: TIM9 drives steps, no servo\n");
          return;
        }
    }
  #endif

  timer_clock_on(TIM9_BASE);
  TIM9->CR1 = 0;
  TIM9->PSC = (uint16_t)(F_CPU / 1000000UL - 1);
  TIM9->ARR = 20000 - 1;                      // 20 ms period.
  TIM9->CCR1 = 1500;
  TIM9->CCMR1 = TIM_CCMR1_OC1PE;              // Frozen, preloaded compare.
  TIM9->DIER = 0;
  TIM9->EGR = TIM_EGR_UG;
  TIM9->SR = 0;
  TIM9->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, IRQ_PRIO_SERVO);
  NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  servo_ok = 1;
}

void servo_write_us(uint16_t us) {
  if ( ! servo_ok)
    return;
  if (us == 0) {
    TIM9->DIER = 0;
    WRITE(BLTOUCH_SERVO_PIN, 0);
    return;
  }
  if (us > 2500)
    us = 2500;
  TIM9->CCR1 = us;                            // Taken over at the next period.
  TIM9->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
}

void servo_write_angle(uint16_t angle) {
  uint32_t us;

  if (angle >= SERVO_MIN_US) {
    us = angle;
    last_angle = (uint16_t)((us - SERVO_MIN_US) * 180UL /
                            (SERVO_MAX_US - SERVO_MIN_US));
  }
  else {
    if (angle > 180)
      angle = 180;
    us = SERVO_MIN_US + (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180UL;
    last_angle = angle;
  }
  servo_write_us((uint16_t)us);
}

uint16_t servo_read_angle(void) {
  return last_angle;
}

/// Update: start of the pulse. Compare 1: end of the pulse.
void TIM1_BRK_TIM9_IRQHandler(void) {
  uint32_t sr = TIM9->SR;                     // One volatile access per
  uint32_t dier = TIM9->DIER;                 // statement (IAR Pa082).

  sr &= dier;
  TIM9->SR = ~sr;
  if (sr & TIM_SR_UIF)
    WRITE(BLTOUCH_SERVO_PIN, 1);
  if (sr & TIM_SR_CC1IF)
    WRITE(BLTOUCH_SERVO_PIN, 0);
}

#endif /* BLTOUCH_SERVO_PIN */
