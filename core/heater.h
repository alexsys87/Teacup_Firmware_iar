#ifndef	_HEATER_H
#define	_HEATER_H

#include "config_wrapper.h"
#include <stdint.h>
#include "temp.h"

/*
  PID defaults, overridable in the printer config. Teacup units, see
  PID_FROM_MARLIN() in printer.p3steel.h for converting Marlin's Kp/Ki/Kd.
*/
#ifndef DEFAULT_P
  /// Default scaled P factor, equivalent to 8.0 counts/qC or 32 counts/C.
  #define DEFAULT_P         8192
#endif
#ifndef DEFAULT_I
  /// Default scaled I factor, equivalent to 0.5 counts/(qC*qs) or 8 counts/C*s.
  #define DEFAULT_I         512
#endif
#ifndef DEFAULT_D
  /// Default scaled D factor, equivalent to 24 counts/(qc/(TH_COUNT*qs)) or
  /// 192 counts/(C/s).
  #define DEFAULT_D         24576
#endif
#ifndef DEFAULT_I_LIMIT
  /// Default scaled I limit, equivalent to 384 qC*qs, or 24 C*s.
  #define DEFAULT_I_LIMIT   384
#endif

/** \def HEATER_THRESHOLD

  Defines the threshold when to turn a non-PWM heater on and when to turn it
  off. Applies only to heaters which have only two states, on and off.
  Opposed to those heaters which allow to turn them on gradually as needed,
  usually by using PWM.
*/
#ifdef BANG_BANG
  #define HEATER_THRESHOLD ((BANG_BANG_ON + BANG_BANG_OFF) / 2)
#else
  #define HEATER_THRESHOLD 8
#endif


#undef DEFINE_HEATER_ACTUAL
#define DEFINE_HEATER_ACTUAL(name, ...) HEATER_ ## name,
typedef enum
{
	#include "config_wrapper.h"
	NUM_HEATERS,
	HEATER_noheater
} heater_t;
#undef DEFINE_HEATER_ACTUAL

/** This struct holds the runtime heater data.

  PID integrator history, temperature history, sanity checker.
*/
typedef struct {
  /// Integrator, \f$-i_{limit} < \sum{4*eC*\Delta t} < i_{limit}\f$
  int16_t heater_i;

  /// Store last TH_COUNT readings in a ring, so we can smooth out our
  /// differentiator.
  uint16_t temp_history[TH_COUNT];
  /// Pointer to last entry in ring.
  uint8_t temp_history_pointer;

  #ifdef HEATER_SANITY_CHECK
    /// How long things haven't seemed sane.
    uint16_t sanity_counter;
    /// A temperature we consider sane given the heater settings.
    uint16_t sane_temperature;
  #endif

  /// This is the PID value we eventually send to the heater.
  uint8_t heater_output;
} heater_runtime_t;

/// Runtime state of each heater, e.g. the current output (heater_output).
extern heater_runtime_t heaters_runtime[NUM_HEATERS];

typedef struct {
  /// sigma delta values for software pwm
  int16_t sd_accu;
  int16_t sd_dir;
} soft_pwm_runtime_t;

typedef enum {
  NO_PWM = 0,
  SOFTWARE_PWM = 1,
  HARDWARE_PWM = 2
} pwm_type_t;

/**
  'pwm' values from DEFINE_HEATER() at or above this use hardware PWM, if
  the pin has a timer channel. With FORCE_SOFTWARE_PWM, pwm = 1 means
  software PWM even on timer pins.
*/
#ifndef FORCE_SOFTWARE_PWM
  #define HARDWARE_PWM_START SOFTWARE_PWM
#else
  #define HARDWARE_PWM_START HARDWARE_PWM
#endif

void heater_init(void);
void pid_init(void);

void heater_set(heater_t index, uint8_t value);
/// Switch all heaters off immediately (emergency stop).
void heater_all_off(void);

/**
  Force all heater outputs inactive at the hardware level. Hardware PWM
  channels are switched to "force inactive" mode, which acts immediately
  (no waiting for the preloaded compare value) and holds regardless of the
  compare register. Only a reset returns to normal operation. Callable
  from any context.
*/
void heater_emergency_off(void);

/// Whether a heater uses this timer for hardware PWM.
uint8_t heater_uses_timer(uint32_t timer);

/// Switch on the clock of a general purpose timer (TIM1..4, TIM9..11).
void timer_clock_on(uint32_t timer);
void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp);

void soft_pwm_tick(void);

uint8_t heaters_all_zero(void);

// Runtime PID tuning (M130..M133). Not persistent, there's no EEPROM.
void pid_set_p(heater_t index, int32_t p);
void pid_set_i(heater_t index, int32_t i);
void pid_set_d(heater_t index, int32_t d);
void pid_set_i_limit(heater_t index, int32_t i_limit);
void pid_get(heater_t index, int32_t *p, int32_t *i, int32_t *d, int32_t *i_limit);

void heater_print(uint16_t i);

#endif	/* _HEATER_H */
