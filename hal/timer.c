/** \file
  \brief Timer management: SysTick for the system clock, TIM5 for steps.

  SysTick fires every TICK_TIME (2 ms) and calls clock_tick(). The possibly
  lengthy dda_clock() runs in PendSV, which has the lowest priority, so it
  gets preempted by steps and serial reception.

  The step timer is TIM5 (32 bit), running free at F_CPU. Steps are
  scheduled relative to the previous compare match, which gives a jitter
  free step distribution independent of the processing time.
*/

#include "timer.h"
#include "arch.h"
#include "cpu.h"
#include "clock.h"
#include "pinio.h"
#include "dda.h"
#include "dda_queue.h"
#include "atomic.h"
#include "babystep.h"
#include "linear_advance.h"
#include "input_shaping.h"

#if TICK_TIME > 0xFFFFFF
  #error TICK_TIME too large for the 24 bit SysTick timer.
#endif

void timer_init(void) {
  /* System clock. */
  SysTick->LOAD = TICK_TIME - 1;
  SysTick->VAL  = 0;
  NVIC_SetPriority(SysTick_IRQn, IRQ_PRIO_SYSTICK);
  NVIC_SetPriority(PendSV_IRQn, IRQ_PRIO_PENDSV);
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |    // Run at F_CPU.
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;

  /*
    Step timer. APB1 runs at F_CPU / 2, timers on APB1 get twice that, so
    TIM5 counts at F_CPU with prescaler 1.
  */
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  (void)RCC->APB1ENR;
  TIM5->CR1  = 0;
  TIM5->PSC  = 0;
  TIM5->ARR  = 0xFFFFFFFFUL;
  TIM5->CCMR1 = 0;                              // CH1 frozen output compare.
  TIM5->DIER = 0;
  TIM5->EGR  = TIM_EGR_UG;                      // Load PSC.
  TIM5->SR   = 0;
  TIM5->CNT  = 0;
  TIM5->CCR1 = 0;
  TIM5->CR1  = TIM_CR1_CEN;
  step_stats_reset();

  NVIC_SetPriority(TIM5_IRQn, IRQ_PRIO_STEP);
  NVIC_ClearPendingIRQ(TIM5_IRQn);
  NVIC_EnableIRQ(TIM5_IRQn);
}

static void timer_check_missed(void);

/** System clock interrupt. */
TEACUP_HOT
void SysTick_Handler(void) {
  clock_tick();
  timer_check_missed();
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;           // Trigger PendSV_Handler().
}

/** System clock interrupt, slow part. */
TEACUP_HOT
void PendSV_Handler(void) {
  dda_clock();
}

volatile step_stats_t step_stats;

void step_stats_reset(void) {
  step_stats.count = 0;
  step_stats.min = 0xFFFFFFFFUL;
  step_stats.max = 0;
  step_stats.sum = 0;
  step_stats.max_latency = 0;
  step_stats.late = 0;
  step_stats.pulses = 0;
  step_stats.since = clock_millis();
}

/**
  A step whose compare time has passed already (timer_set()) runs from
  this flag and a pending interrupt, not from an EGR write: CC1G would do
  on the STM32, but Renode's timer model resets the counter on any EGR
  write, which moves all time stamps (babysteps, input shaping) into the
  future.
*/
static volatile uint8_t step_kicked;

/// Bit numbers in TIM5->DIER, for atomic bit-band access.
#define DIER_CC1IE_BIT  1
#define DIER_CC2IE_BIT  2
#define DIER_CC4IE_BIT  4

/**
  Safety net, every TICK_TIME: a compare match of the step timer (next
  step, auxiliary generator) more than 1 ms overdue without its flag was
  missed, run it from software. timer_set() and timer_aux_set() catch
  compare times in the past, so on the STM32 this doesn't happen. Renode's
  timer model misses a compare set just ahead of the counter now and then,
  which would stop the steps for a full round of the counter (51 s).
*/
static void timer_check_missed(void) {
  ATOMIC_START();
    uint32_t dier = TIM5->DIER;
    uint32_t sr = TIM5->SR;
    uint32_t now = TIM5->CNT;

    if ((dier & TIM_DIER_CC1IE) && ! (sr & TIM_SR_CC1IF) &&
        (int32_t)(now - TIM5->CCR1) > (int32_t)(1 MS)) {
      PERIPH_BIT(TIM5->DIER, DIER_CC1IE_BIT) = 0;
      step_kicked = 1;
      NVIC_SetPendingIRQ(TIM5_IRQn);
      step_stats.late++;
    }
    #ifdef STEP_AUX
      if ((dier & TIM_DIER_CC4IE) && ! (sr & TIM_SR_CC4IF) &&
          (int32_t)(now - TIM5->CCR4) > (int32_t)(1 MS)) {
        PERIPH_BIT(TIM5->DIER, DIER_CC4IE_BIT) = 0;
        aux_kicked = 1;
        NVIC_SetPendingIRQ(TIM5_IRQn);
      }
    #endif
  ATOMIC_END();
}

TEACUP_HOT
TEACUP_STEP_RAMFUNC void timer_step_pulse_end(void) {
  TIM5->CCR2 = TIM5->CNT + STEP_PULSE_CYCLES;
  TIM5->SR = ~TIM_SR_CC2IF;
  PERIPH_BIT(TIM5->DIER, DIER_CC2IE_BIT) = 1;
}

#ifdef STEP_AUX
/*
  Auxiliary step generator: steps that don't follow the Bresenham steps of
  the moves right away, E of linear advance (motion/linear_advance.c) and
  X/Y of input shaping (motion/input_shaping.c). They share compare
  channel 4 of the step timer: each part does what is due and tells when
  it wants to run again, the earliest of these is scheduled.

  "Right away" is a software flag plus a pending interrupt, not an EGR
  write: CC4G would do on the STM32, but Renode's timer model resets the
  counter on any EGR write, which delays the next step.
*/
volatile uint8_t aux_kicked;

TEACUP_HOT
TEACUP_STEP_RAMFUNC static void timer_aux_set(uint32_t delay) {
  uint32_t compare = TIM5->CNT + delay;

  TIM5->CCR4 = compare;
  TIM5->SR = ~TIM_SR_CC4IF;
  PERIPH_BIT(TIM5->DIER, DIER_CC4IE_BIT) = 1;
  if ((int32_t)(TIM5->CNT - compare) >= 0) {    // Due already.
    PERIPH_BIT(TIM5->DIER, DIER_CC4IE_BIT) = 0;
    aux_kicked = 1;
    NVIC_SetPendingIRQ(TIM5_IRQn);
  }
}

void timer_aux_kick(void) {
  aux_kicked = 1;
  NVIC_SetPendingIRQ(TIM5_IRQn);
}

TEACUP_HOT
TEACUP_STEP_RAMFUNC static void aux_isr(void) {
  uint32_t now = TIM5->CNT;
  uint32_t next = AUX_NONE, d;

  aux_kicked = 0;
  #ifdef LINEAR_ADVANCE
    d = la_service(now);
    if (d < next)
      next = d;
  #endif
  #ifdef INPUT_SHAPING
    d = shaper_service(now);
    if (d < next)
      next = d;
  #endif
  if (next != AUX_NONE)
    timer_aux_set(next);
}
#endif /* STEP_AUX */

/**
  Step timer interrupt. Four sources:
   - Compare 2: end of the step pulses, lower all step pins.
   - Compare 3: babystep (M290), see core/babystep.c.
   - Compare 1: time for the next step.
   - Compare 4: auxiliary step generator (linear advance, input shaping),
     see aux_isr(). It also runs right after a step which gave it work.
  Pulse end first, in case both are due, then babysteps, which must not
  start while a Z pulse is still high.
*/
TEACUP_HOT
TEACUP_STEP_RAMFUNC void TIM5_IRQHandler(void) {
  uint32_t entry = DWT->CYCCNT;
  uint32_t sr = TIM5->SR;
  uint32_t dier = TIM5->DIER;

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 1);
  #endif

  if ((sr & TIM_SR_CC2IF) && (dier & TIM_DIER_CC2IE)) {
    PERIPH_BIT(TIM5->DIER, DIER_CC2IE_BIT) = 0;
    TIM5->SR = ~TIM_SR_CC2IF;
    unstep();
    step_stats.pulses++;
  }

  #ifdef BABYSTEPPING
    if ((sr & TIM_SR_CC3IF) && (dier & TIM_DIER_CC3IE)) {
      TIM5->SR = ~TIM_SR_CC3IF;
      babystep_isr();
    }
  #endif

  if (step_kicked || ((sr & TIM_SR_CC1IF) && (dier & TIM_DIER_CC1IE))) {
    uint32_t now = TIM5->CNT;               // One volatile access per
    uint32_t match = TIM5->CCR1;            // statement (IAR Pa082).
    uint32_t latency = now - match;
    uint32_t duration;

    // Turn off step interrupt generation, timer counter continues.
    step_kicked = 0;
    PERIPH_BIT(TIM5->DIER, DIER_CC1IE_BIT) = 0;
    TIM5->SR = ~TIM_SR_CC1IF;

    queue_step();

    duration = DWT->CYCCNT - entry;
    step_stats.count++;
    step_stats.sum += duration;
    if (duration < step_stats.min)
      step_stats.min = duration;
    if (duration > step_stats.max)
      step_stats.max = duration;
    if (latency < 0x80000000UL && latency > step_stats.max_latency)
      step_stats.max_latency = latency;
  }

  #ifdef STEP_AUX
    if (aux_kicked || ((sr & TIM_SR_CC4IF) && (dier & TIM_DIER_CC4IE))) {
      PERIPH_BIT(TIM5->DIER, DIER_CC4IE_BIT) = 0;
      TIM5->SR = ~TIM_SR_CC4IF;
      aux_isr();
    }
  #endif

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 0);
  #endif
}

/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks, counted from
         the previous step interrupt (or from timer_reset()).

  \param check_short Tell whether to check for impossibly short requests.
         Used by ACCELERATION_TEMPORAL only. Short requests then return 1
         and do not schedule a timer interrupt.

  \return 1 if the requested time was too short to schedule an interrupt.

  If the calculated compare time has already passed when we get here (very
  short delay, long interrupt latency), the interrupt is triggered by
  software immediately. Without this the step would be delayed by a full
  round of the 32 bit counter (51 s at 84 MHz).
*/
TEACUP_HOT
TEACUP_STEP_RAMFUNC uint8_t timer_set(int32_t delay, uint8_t check_short) {

  #ifdef ACCELERATION_TEMPORAL
    if (check_short) {
      // 160 cycles: time needed to complete the current interrupt.
      uint32_t now = TIM5->CNT;
      uint32_t last = TIM5->CCR1;

      if ((int32_t)(now - last) + 160 > delay)
        return 1;
    }
  #else
    (void)check_short;
  #endif

  {
    uint32_t compare = TIM5->CCR1 + (uint32_t)delay;
    uint32_t now;

    TIM5->CCR1 = compare;
    TIM5->SR = ~TIM_SR_CC1IF;                   // Drop stale matches.
    // Bit-band: atomic, the pulse end interrupt enable stays untouched.
    PERIPH_BIT(TIM5->DIER, DIER_CC1IE_BIT) = 1;

    now = TIM5->CNT;
    if ((int32_t)(now - compare) >= 0) {
      // Too late, fire right now (see step_kicked).
      PERIPH_BIT(TIM5->DIER, DIER_CC1IE_BIT) = 0;
      step_kicked = 1;
      NVIC_SetPendingIRQ(TIM5_IRQn);
      step_stats.late++;
    }
  }

  return 0;
}

/** Timer reset.

  The step timer was idle: count the next step interrupt from now. The
  counter keeps running, so babysteps and the auxiliary step generator
  keep their schedules and time stamps.
*/
void timer_reset(void) {
  TIM5->CCR1 = TIM5->CNT;
}

/** Stop timers. This means to be an emergency stop. */
void timer_stop(void) {
  SysTick->CTRL = 0;
  TIM5->DIER = 0;
  NVIC_DisableIRQ(TIM5_IRQn);
}
