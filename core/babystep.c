/** \file
  \brief Babystepping (M290), see babystep.h.

  Timing of one babystep, all inside the step interrupt, so no step of a
  running move gets in between:

    - skip (retry shortly) while a STEP pulse of the Z axis is still high,
    - set DIR if it differs, wait 1 us (DRV8825: 650 ns setup),
    - STEP pulse of MIN_STEP_PULSE_US (or the Z timer's one pulse),
    - restore DIR, wait 1 us again, so the next move step sees a valid DIR.

  That's up to ~4 us of busy waiting per babystep. Babysteps come at
  BABYSTEP_FEEDRATE (default 1 mm/s, 8000 steps/s at 8000 steps/mm, one
  every 125 us), so moves get delayed by a few us at most.
*/

#include "babystep.h"

#ifdef BABYSTEPPING

#include "arch.h"
#include "atomic.h"
#include "pinio.h"
#include "dda.h"
#include "dda_maths.h"
#include "settings.h"
#include "bed_leveling.h"

/// Z offset, um.
static int32_t z_offset = 0;

/// Steps still to do, positive = up. Changed in the step interrupt.
static volatile int32_t pending = 0;

/// Cycles between two babysteps.
static uint32_t interval = 10000;

#define DIER_CC3IE_BIT       3
#define DIR_SETUP_CYCLES     (F_CPU / 1000000UL)     // 1 us
#define RETRY_CYCLES         (5UL * (F_CPU / 1000000UL))

int32_t babystep_offset(void) {
  return z_offset;
}

void babystep_set_offset(int32_t um) {
  z_offset = um;
  zcorr_sync_logical();
}

uint8_t babystep_busy(void) {
  return pending != 0;
}

/// Start the babystep timer channel, if not running. Interrupts off.
static void arm(void) {
  if ( ! (TIM5->DIER & TIM_DIER_CC3IE)) {
    TIM5->CCR3 = TIM5->CNT + RETRY_CYCLES;
    TIM5->SR = ~TIM_SR_CC3IF;
    PERIPH_BIT(TIM5->DIER, DIER_CC3IE_BIT) = 1;
  }
}

int32_t babystep_add(int32_t um) {
  const int32_t limit = (int32_t)(BABYSTEP_LIMIT * 1000.);
  int32_t new_offset = z_offset + um;
  int32_t old_steps, n;
  uint64_t cycles;

  if (new_offset > limit)
    new_offset = limit;
  if (new_offset < -limit)
    new_offset = -limit;
  um = new_offset - z_offset;

  // Motor position of the end of the queue before and after.
  old_steps = um_to_steps(startpoint.axis[Z] + bed_level_offset(startpoint.axis), Z);
  z_offset = new_offset;
  n = um_to_steps(startpoint.axis[Z] + bed_level_offset(startpoint.axis), Z) -
      old_steps;
  if (n == 0)
    return um;

  // The queue ends n steps higher (lower) now, moves created from now on
  // start there.
  startpoint_steps.axis[Z] += n;

  cycles = (uint64_t)F_CPU * 60000ULL /
           ((uint64_t)settings.steps_per_m[Z] * BABYSTEP_FEEDRATE);
  if (cycles < 2 * RETRY_CYCLES)
    cycles = 2 * RETRY_CYCLES;
  if (cycles > 0x7FFFFFFFUL)
    cycles = 0x7FFFFFFFUL;

  power_on();
  stepper_enable();
  z_enable();

  ATOMIC_START();
    interval = (uint32_t)cycles;
    pending += n;
    arm();
  ATOMIC_END();
  return um;
}

void babystep_tick(void) {
  if (pending) {
    ATOMIC_START();
      if (pending)
        arm();
    ATOMIC_END();
  }
}

/// Busy wait for 'cycles' CPU cycles.
TEACUP_INLINE void wait_cycles(uint32_t cycles) {
  uint32_t start = DWT->CYCCNT;

  while ((DWT->CYCCNT - start) < cycles)
    ;
}

/// Raw output level of a pin.
#define ODR_BIT(IO) ((PIN_PORT(IO)->ODR >> PIN_NUM(IO)) & 1UL)

TEACUP_HOT
TEACUP_STEP_RAMFUNC void babystep_isr(void) {
  int32_t p = pending;
  uint32_t dir1;
  #if defined Z2_STEP_PIN && defined Z2_DIR_PIN
    uint32_t dir2;
  #endif
  uint8_t changed;
  uint8_t up;
  TIM_TypeDef *zt = NULL;

  if (p == 0) {
    PERIPH_BIT(TIM5->DIER, DIER_CC3IE_BIT) = 0;
    return;
  }

  #ifdef STEP_TIMER_PULSES
    zt = step_timer[2];
  #endif
  // A Z step of a move still in progress: try again shortly.
  if (zt ? (zt->CR1 & TIM_CR1_CEN) != 0 : ODR_BIT(Z_STEP_PIN) != 0) {
    TIM5->CCR3 = TIM5->CNT + RETRY_CYCLES;
    return;
  }

  up = (p > 0);
  dir1 = ODR_BIT(Z_DIR_PIN);
  #if defined Z2_STEP_PIN && defined Z2_DIR_PIN
    dir2 = ODR_BIT(Z2_DIR_PIN);
  #endif
  z_direction(up);
  changed = (ODR_BIT(Z_DIR_PIN) != dir1);
  #if defined Z2_STEP_PIN && defined Z2_DIR_PIN
    changed |= (ODR_BIT(Z2_DIR_PIN) != dir2);
  #endif
  if (changed)
    wait_cycles(DIR_SETUP_CYCLES);

  if (zt) {
    #ifdef STEP_TIMER_PULSES
      STEP_TRIGGER(zt);
      while (zt->CR1 & TIM_CR1_CEN)
        ;
    #endif
  }
  else {
    _z_step(1);
    wait_cycles(STEP_PULSE_CYCLES);
    _z_step(0);
  }

  if (changed) {
    WRITE(Z_DIR_PIN, dir1);
    #if defined Z2_STEP_PIN && defined Z2_DIR_PIN
      WRITE(Z2_DIR_PIN, dir2);
    #endif
    wait_cycles(DIR_SETUP_CYCLES);
  }

  p += up ? -1 : 1;
  pending = p;
  if (p == 0) {
    PERIPH_BIT(TIM5->DIER, DIER_CC3IE_BIT) = 0;
  }
  else {
    // Steady rate, but never schedule into the past.
    uint32_t next = TIM5->CCR3 + interval;

    if ((int32_t)(next - TIM5->CNT) < (int32_t)RETRY_CYCLES)
      next = TIM5->CNT + RETRY_CYCLES;
    TIM5->CCR3 = next;
  }
}

#endif /* BABYSTEPPING */
