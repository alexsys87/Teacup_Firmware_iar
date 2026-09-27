/** \file
  \brief I/O primitives - step, enable, direction, endstops etc.

  Pins are named Arduino style (PA_5 or PA5), see pins_stm32f4.h. All
  macros resolve at compile time into single register accesses, e.g.
  WRITE(PA_5, 1) becomes GPIOA->BSRR = 1 << 5.
*/

#ifndef _PINIO_H
#define _PINIO_H

#include <stdint.h>
#include "arch.h"
#include "pins_stm32f4.h"
#include "config_wrapper.h"

// Configuration tests.
#if defined USE_INTERNAL_PULLDOWNS && defined USE_INTERNAL_PULLUPS
  #error Cant use USE_INTERNAL_PULLUPS and ..._PULLDOWNS at the same time.
#endif

/** GPIO MODER values. */
#define GPIO_MODE_INPUT   0UL
#define GPIO_MODE_OUTPUT  1UL
#define GPIO_MODE_AF      2UL
#define GPIO_MODE_ANALOG  3UL

/** GPIO PUPDR values. */
#define GPIO_PULL_NONE    0UL
#define GPIO_PULL_UP      1UL
#define GPIO_PULL_DOWN    2UL

/** GPIO OSPEEDR values. */
#define GPIO_SPEED_LOW    0UL
#define GPIO_SPEED_MEDIUM 1UL
#define GPIO_SPEED_FAST   2UL
#define GPIO_SPEED_HIGH   3UL

/*
  Low level helpers. Read-modify-write of these registers happens in
  non-interrupt code only, interrupts write BSRR only, which is atomic.
*/
TEACUP_INLINE void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode) {
  port->MODER = (port->MODER & ~(3UL << (pin * 2))) | (mode << (pin * 2));
}

TEACUP_INLINE void gpio_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull) {
  port->PUPDR = (port->PUPDR & ~(3UL << (pin * 2))) | (pull << (pin * 2));
}

TEACUP_INLINE void gpio_speed(GPIO_TypeDef *port, uint32_t pin, uint32_t spd) {
  port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2))) | (spd << (pin * 2));
}

TEACUP_INLINE void gpio_open_drain(GPIO_TypeDef *port, uint32_t pin,
                                   uint32_t od) {
  port->OTYPER = (port->OTYPER & ~(1UL << pin)) | ((od ? 1UL : 0UL) << pin);
}

TEACUP_INLINE void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af) {
  uint32_t idx = pin >> 3;
  uint32_t shift = (pin & 7UL) * 4;
  port->AFR[idx] = (port->AFR[idx] & ~(0xFUL << shift)) | (af << shift);
}

TEACUP_INLINE void gpio_write(GPIO_TypeDef *port, uint32_t pin, uint32_t v) {
  port->BSRR = v ? (1UL << pin) : (1UL << (pin + 16));
}

/// Read a pin, gives 0 or 1.
#define READ(IO)        ((PIN_PORT(IO)->IDR & MASK(PIN_NUM(IO))) ? 1 : 0)

/// Write to a pin.
#define WRITE(IO, v) \
  do { \
    if (v) \
      PIN_PORT(IO)->BSRR = MASK(PIN_NUM(IO)); \
    else \
      PIN_PORT(IO)->BSRR = MASK(PIN_NUM(IO)) << 16; \
  } while (0)

/// Set pin as input, pull resistor off.
#define SET_INPUT(IO) \
  do { \
    gpio_pull(PIN_PORT(IO), PIN_NUM(IO), GPIO_PULL_NONE); \
    gpio_mode(PIN_PORT(IO), PIN_NUM(IO), GPIO_MODE_INPUT); \
  } while (0)

/// Set pin as push-pull output. Set the level with WRITE() before.
#define SET_OUTPUT(IO) \
  do { \
    gpio_open_drain(PIN_PORT(IO), PIN_NUM(IO), 0); \
    gpio_speed(PIN_PORT(IO), PIN_NUM(IO), GPIO_SPEED_FAST); \
    gpio_mode(PIN_PORT(IO), PIN_NUM(IO), GPIO_MODE_OUTPUT); \
  } while (0)

/// Set pin as analog input.
#define SET_ANALOG(IO) \
  do { \
    gpio_pull(PIN_PORT(IO), PIN_NUM(IO), GPIO_PULL_NONE); \
    gpio_mode(PIN_PORT(IO), PIN_NUM(IO), GPIO_MODE_ANALOG); \
  } while (0)

/// Connect pin to an alternate function.
#define SET_AF(IO, af) \
  do { \
    gpio_af(PIN_PORT(IO), PIN_NUM(IO), (af)); \
    gpio_speed(PIN_PORT(IO), PIN_NUM(IO), GPIO_SPEED_HIGH); \
    gpio_mode(PIN_PORT(IO), PIN_NUM(IO), GPIO_MODE_AF); \
  } while (0)

/// Enable pullup resistor or switch from pulldown to pullup.
#define PULLUP_ON(IO)   gpio_pull(PIN_PORT(IO), PIN_NUM(IO), GPIO_PULL_UP)
/// Enable pulldown resistor or switch from pullup to pulldown.
#define PULLDOWN_ON(IO) gpio_pull(PIN_PORT(IO), PIN_NUM(IO), GPIO_PULL_DOWN)
/// Disable pull resistor.
#define PULL_OFF(IO)    gpio_pull(PIN_PORT(IO), PIN_NUM(IO), GPIO_PULL_NONE)

/*
  Runtime pin access by PIN_ID(), for pins stored in tables.
*/
TEACUP_INLINE void pin_id_write(uint8_t id, uint8_t v) {
  gpio_write(PIN_ID_PORT(id), PIN_ID_NUM(id), v);
}

TEACUP_INLINE void pin_id_output(uint8_t id) {
  gpio_open_drain(PIN_ID_PORT(id), PIN_ID_NUM(id), 0);
  gpio_speed(PIN_ID_PORT(id), PIN_ID_NUM(id), GPIO_SPEED_FAST);
  gpio_mode(PIN_ID_PORT(id), PIN_ID_NUM(id), GPIO_MODE_OUTPUT);
}

/*
  Power
*/

/// psu_timeout is set to zero when we step, and increases over time so we can
/// turn the motors off when they've been idle for a while.
/// A second function is to guarantee a minimum on time of the PSU.
/// Timeout counting is done in clock.c.
extern volatile uint8_t psu_timeout;

TEACUP_INLINE void power_init(void) {
  #ifdef PS_MOSFET_PIN
    WRITE(PS_MOSFET_PIN, 0);
    SET_OUTPUT(PS_MOSFET_PIN);
  #endif
}

void pinio_init(void);

/// Switch the power supply on (if off), waits until it's up.
void power_on(void);
/// Disable steppers, switch the power supply off. Callable with IRQs off.
void power_off(void);
/// 30 s idle, heaters off: supply off with PS_AUTO_OFF, else nothing.
void power_idle(void);
/// 1 while the supply is switched on.
uint8_t power_is_on(void);

/// Whether the stepper drivers are enabled (STEPPER_ENABLE_PIN).
uint8_t steppers_enabled(void);

/// Enable all stepper drivers (M17), switches the PSU on.
void steppers_enable_all(void);

/// Disable all stepper drivers (M18/M84), the PSU stays on.
void steppers_disable_all(void);

/**
  Stepper idle timeout (M84 S / M18 S), seconds, 0 = never. Steppers are
  disabled when nothing moved for this long (and no temperature wait is
  ongoing). Called every second with 'active' = moving or waiting.
*/
void steppers_idle_tick(uint8_t active);
void steppers_set_idle_timeout(uint16_t seconds);
uint16_t steppers_get_idle_timeout(void);

/*
  Step pulse width. The step interrupt raises the step pins, does its
  calculations and waits at the end until MIN_STEP_PULSE_US has passed
  since its entry before lowering them again.
*/
#define STEP_PULSE_CYCLES \
  ((uint32_t)MIN_STEP_PULSE_US * (F_CPU / 1000000UL) + 16UL)

TEACUP_INLINE void step_pulse_wait(uint32_t start) {
  while ((DWT->CYCCNT - start) < STEP_PULSE_CYCLES)
    ;
}

/*
  X Stepper
*/

#define _x_step(st)             WRITE(X_STEP_PIN, st)
#define x_step()                _x_step(1)
#ifndef X_INVERT_DIR
  #define x_direction(dir)      WRITE(X_DIR_PIN, dir)
#else
  #define x_direction(dir)      WRITE(X_DIR_PIN, (dir) ^ 1)
#endif
#ifdef X_MIN_PIN
  #ifndef X_INVERT_MIN
    #define x_min()             READ(X_MIN_PIN)
  #else
    #define x_min()             (READ(X_MIN_PIN) ^ 1)
  #endif
#else
  #define x_min()               (0)
#endif
#ifdef X_MAX_PIN
  #ifndef X_INVERT_MAX
    #define x_max()             READ(X_MAX_PIN)
  #else
    #define x_max()             (READ(X_MAX_PIN) ^ 1)
  #endif
#else
  #define x_max()               (0)
#endif

/*
  Y Stepper
*/

#define _y_step(st)             WRITE(Y_STEP_PIN, st)
#define y_step()                _y_step(1)
#ifndef Y_INVERT_DIR
  #define y_direction(dir)      WRITE(Y_DIR_PIN, dir)
#else
  #define y_direction(dir)      WRITE(Y_DIR_PIN, (dir) ^ 1)
#endif
#ifdef Y_MIN_PIN
  #ifndef Y_INVERT_MIN
    #define y_min()             READ(Y_MIN_PIN)
  #else
    #define y_min()             (READ(Y_MIN_PIN) ^ 1)
  #endif
#else
  #define y_min()               (0)
#endif
#ifdef Y_MAX_PIN
  #ifndef Y_INVERT_MAX
    #define y_max()             READ(Y_MAX_PIN)
  #else
    #define y_max()             (READ(Y_MAX_PIN) ^ 1)
  #endif
#else
  #define y_max()               (0)
#endif

/*
  Z Stepper
*/

#if defined Z_STEP_PIN && defined Z_DIR_PIN
  #ifndef Z_INVERT_DIR
    #define _z_dir1(dir)        WRITE(Z_DIR_PIN, dir)
  #else
    #define _z_dir1(dir)        WRITE(Z_DIR_PIN, (dir) ^ 1)
  #endif
  #if defined Z2_STEP_PIN
    /*
      Second Z motor on its own driver (Marlin: Z_DUAL_STEPPER_DRIVERS),
      stepped together with the first one. Without Z2_DIR_PIN both drivers
      share Z_DIR_PIN, only the STEP is separate (for G34).
    */
    #define _z_step(st)         do { WRITE(Z_STEP_PIN, st); \
                                     WRITE(Z2_STEP_PIN, st); } while (0)
    #if defined Z2_DIR_PIN
      #ifndef Z2_INVERT_DIR
        #define _z_dir2(dir)    WRITE(Z2_DIR_PIN, dir)
      #else
        #define _z_dir2(dir)    WRITE(Z2_DIR_PIN, (dir) ^ 1)
      #endif
      #define z_direction(dir)  do { _z_dir1(dir); _z_dir2(dir); } while (0)
    #else
      #define z_direction(dir)  _z_dir1(dir)
    #endif
  #else
    #define _z_step(st)         WRITE(Z_STEP_PIN, st)
    #define z_direction(dir)    _z_dir1(dir)
  #endif
  #define z_step()              _z_step(1)
#else
  #define _z_step(st)           do { } while (0)
  #define z_step()              do { } while (0)
  #define z_direction(dir)      do { } while (0)
#endif
#ifdef Z_MIN_PIN
  #ifndef Z_INVERT_MIN
    #define z_min()             READ(Z_MIN_PIN)
  #else
    #define z_min()             (READ(Z_MIN_PIN) ^ 1)
  #endif
#else
  #define z_min()               (0)
#endif
#ifdef Z_MAX_PIN
  #ifndef Z_INVERT_MAX
    #define z_max()             READ(Z_MAX_PIN)
  #else
    #define z_max()             (READ(Z_MAX_PIN) ^ 1)
  #endif
#else
  #define z_max()               (0)
#endif

/*
  Extruder
*/

/**
  EXTRUDERS 2: second extruder motor on E1_STEP_PIN / E1_DIR_PIN
  (E1_INVERT_DIR, optional E1_ENABLE_PIN / E1_INVERT_ENABLE). T0 / T1
  select the motor E moves go to (active_extruder, changed only with the
  queue empty). Both extruders share E steps per mm (M92 E), the hotend
  heater and the E coordinate. E1 always steps by GPIO pulses.
*/
#ifndef EXTRUDERS
  #define EXTRUDERS 1
#endif
#if EXTRUDERS < 1 || EXTRUDERS > 2
  #error EXTRUDERS must be 1 or 2.
#endif
#if EXTRUDERS == 2
  #if ! (defined E1_STEP_PIN && defined E1_DIR_PIN)
    #error EXTRUDERS 2 needs E1_STEP_PIN and E1_DIR_PIN.
  #endif
  #if ! (defined E_STEP_PIN && defined E_DIR_PIN)
    #error EXTRUDERS 2 needs E_STEP_PIN and E_DIR_PIN.
  #endif
  /// Extruder motor of E moves, 0 or 1 (T0 / T1).
  extern volatile uint8_t active_extruder;
  #ifndef E1_INVERT_DIR
    #define _e1_direction(dir)  WRITE(E1_DIR_PIN, dir)
  #else
    #define _e1_direction(dir)  WRITE(E1_DIR_PIN, (dir) ^ 1)
  #endif
#else
  #define active_extruder       0
  #define _e1_direction(dir)    do { } while (0)
#endif

#if defined E_STEP_PIN && defined E_DIR_PIN
  #if EXTRUDERS == 2
    #define _e_step(st)         do { if (active_extruder) \
                                       WRITE(E1_STEP_PIN, st); \
                                     else WRITE(E_STEP_PIN, st); } while (0)
  #else
    #define _e_step(st)         WRITE(E_STEP_PIN, st)
  #endif
  #define e_step()              _e_step(1)
  // Both direction pins: the idle motor doesn't step anyway.
  #ifndef E_INVERT_DIR
    #define e_direction(dir)    do { WRITE(E_DIR_PIN, dir); \
                                     _e1_direction(dir); } while (0)
  #else
    #define e_direction(dir)    do { WRITE(E_DIR_PIN, (dir) ^ 1); \
                                     _e1_direction(dir); } while (0)
  #endif
#else
  #define _e_step(st)           do { } while (0)
  #define e_step()              do { } while (0)
  #define e_direction(dir)      do { } while (0)
#endif

/*
  All step pins of one GPIO port are written with a single BSRR access.
  STEP_BIT(pin, port) is the pin's mask if it's on that port (index 0 = A,
  1 = B, ... 7 = H), all of it resolves at compile time.
*/
#define STEP_BIT(pin, idx) \
  ((((PIN_ID(pin)) >> 4) == (idx)) ? MASK((PIN_ID(pin)) & 0x0FU) : 0UL)

#define STEP_X_BIT(idx)   STEP_BIT(X_STEP_PIN, idx)
#define STEP_Y_BIT(idx)   STEP_BIT(Y_STEP_PIN, idx)
#if defined Z_STEP_PIN && defined Z_DIR_PIN
  #define STEP_Z_BIT(idx) STEP_BIT(Z_STEP_PIN, idx)
#else
  #define STEP_Z_BIT(idx) 0UL
#endif
#if defined Z2_STEP_PIN && defined Z_STEP_PIN
  #define STEP_Z2_BIT(idx) STEP_BIT(Z2_STEP_PIN, idx)
#else
  #define STEP_Z2_BIT(idx) 0UL
#endif
#if defined E_STEP_PIN && defined E_DIR_PIN
  #define STEP_E_BIT(idx) STEP_BIT(E_STEP_PIN, idx)
#else
  #define STEP_E_BIT(idx) 0UL
#endif
#if EXTRUDERS == 2
  #define STEP_E1_BIT(idx) STEP_BIT(E1_STEP_PIN, idx)
#else
  #define STEP_E1_BIT(idx) 0UL
#endif

/// All step pins on port idx.
#define STEP_PORT_MASK(idx) (STEP_X_BIT(idx) | STEP_Y_BIT(idx) | \
  STEP_Z_BIT(idx) | STEP_Z2_BIT(idx) | STEP_E_BIT(idx) | STEP_E1_BIT(idx))

/// Step pins to raise, per port, collected by dda_step().
typedef struct {
  uint32_t a, b, c, d, e, h;
} step_set_t;

#define STEP_ADD_BITS(s, BITS) do { \
    (s)->a |= BITS(0); (s)->b |= BITS(1); (s)->c |= BITS(2); \
    (s)->d |= BITS(3); (s)->e |= BITS(4); (s)->h |= BITS(7); \
  } while (0)
#define STEP_ZZ2_BIT(idx) (STEP_Z_BIT(idx) | STEP_Z2_BIT(idx))

#define step_set_clear(s) \
  do { (s)->a = (s)->b = (s)->c = (s)->d = (s)->e = (s)->h = 0; } while (0)
#define step_add_x(s)   STEP_ADD_BITS(s, STEP_X_BIT)
#define step_add_y(s)   STEP_ADD_BITS(s, STEP_Y_BIT)
#ifdef Z_STEPPER_ALIGN
  /**
    G34: bit 0 steps Z, bit 1 Z2. Both set except while G34 moves one of
    them alone (Z steps are GPIO pulses then, see step_timers.c).
  */
  extern volatile uint8_t z_step_mask;
  #define step_add_z(s) do { \
      if (z_step_mask & 1) STEP_ADD_BITS(s, STEP_Z_BIT); \
      if (z_step_mask & 2) STEP_ADD_BITS(s, STEP_Z2_BIT); \
    } while (0)
#else
  #define step_add_z(s)   STEP_ADD_BITS(s, STEP_ZZ2_BIT)
#endif
#if EXTRUDERS == 2
  #define step_add_e(s) do { if (active_extruder) \
                               STEP_ADD_BITS(s, STEP_E1_BIT); \
                             else STEP_ADD_BITS(s, STEP_E_BIT); } while (0)
#else
  #define step_add_e(s) STEP_ADD_BITS(s, STEP_E_BIT)
#endif

/// Whether any GPIO step pin is to be raised.
#define step_set_any(s) \
  (((s)->a | (s)->b | (s)->c | (s)->d | (s)->e | (s)->h) != 0)

/*
  STEP_TIMER_PULSES: step pins on a timer channel get their pulse from the
  timer in one pulse mode, started with a single register write. The pulse
  width comes from hardware, nothing to do for its end. Axes whose pin has
  no usable timer fall back to GPIO pulses. See hal/step_timers.c.
*/
#ifdef STEP_TIMER_PULSES
  /// Timer of each axis group (Z and Z2 together), NULL = GPIO pulses.
  extern TIM_TypeDef *step_timer[4];
  #define STEP_TRIGGER(tim)   ((tim)->CR1 = TIM_CR1_OPM | TIM_CR1_CEN)
  #define step_x(s)  do { if (step_timer[0]) STEP_TRIGGER(step_timer[0]); \
                          else step_add_x(s); } while (0)
  #define step_y(s)  do { if (step_timer[1]) STEP_TRIGGER(step_timer[1]); \
                          else step_add_y(s); } while (0)
  #define step_z(s)  do { if (step_timer[2]) STEP_TRIGGER(step_timer[2]); \
                          else step_add_z(s); } while (0)
  /// Timer of the active extruder, NULL = GPIO pulses (E1 always).
  #define step_timer_e()  (active_extruder ? (TIM_TypeDef *)0 : step_timer[3])
  #define step_e(s)  do { TIM_TypeDef *_t = step_timer_e(); \
                          if (_t) STEP_TRIGGER(_t); \
                          else step_add_e(s); } while (0)
#else
  #define step_x(s)  step_add_x(s)
  #define step_y(s)  step_add_y(s)
  #define step_z(s)  step_add_z(s)
  #define step_e(s)  step_add_e(s)
#endif

/// Set up timer step pulses (no-op without STEP_TIMER_PULSES).
void step_timers_init(void);

/// Raise the collected step pins, one BSRR write per used port.
TEACUP_INLINE void step_output(const step_set_t *s) {
  if (STEP_PORT_MASK(0) && s->a) GPIOA->BSRR = s->a;
  if (STEP_PORT_MASK(1) && s->b) GPIOB->BSRR = s->b;
  if (STEP_PORT_MASK(2) && s->c) GPIOC->BSRR = s->c;
  if (STEP_PORT_MASK(3) && s->d) GPIOD->BSRR = s->d;
  if (STEP_PORT_MASK(4) && s->e) GPIOE->BSRR = s->e;
  if (STEP_PORT_MASK(7) && s->h) GPIOH->BSRR = s->h;
}

/*
  End Step - All Steppers, one BSRR write per used port.
*/
TEACUP_INLINE void unstep_all(void) {
  if (STEP_PORT_MASK(0)) GPIOA->BSRR = STEP_PORT_MASK(0) << 16;
  if (STEP_PORT_MASK(1)) GPIOB->BSRR = STEP_PORT_MASK(1) << 16;
  if (STEP_PORT_MASK(2)) GPIOC->BSRR = STEP_PORT_MASK(2) << 16;
  if (STEP_PORT_MASK(3)) GPIOD->BSRR = STEP_PORT_MASK(3) << 16;
  if (STEP_PORT_MASK(4)) GPIOE->BSRR = STEP_PORT_MASK(4) << 16;
  if (STEP_PORT_MASK(7)) GPIOH->BSRR = STEP_PORT_MASK(7) << 16;
}

#define unstep() unstep_all()

/*
  Stepper Enable Pins
*/

#ifdef STEPPER_ENABLE_PIN
  #ifdef STEPPER_INVERT_ENABLE
    #define stepper_enable()    WRITE(STEPPER_ENABLE_PIN, 0)
    #define stepper_disable()   WRITE(STEPPER_ENABLE_PIN, 1)
  #else
    #define stepper_enable()    WRITE(STEPPER_ENABLE_PIN, 1)
    #define stepper_disable()   WRITE(STEPPER_ENABLE_PIN, 0)
  #endif
#else
  #define stepper_enable()      do { } while (0)
  #define stepper_disable()     do { } while (0)
#endif

#ifdef X_ENABLE_PIN
  #ifdef X_INVERT_ENABLE
    #define x_enable()          WRITE(X_ENABLE_PIN, 0)
    #define x_disable()         WRITE(X_ENABLE_PIN, 1)
  #else
    #define x_enable()          WRITE(X_ENABLE_PIN, 1)
    #define x_disable()         WRITE(X_ENABLE_PIN, 0)
  #endif
#else
  #define x_enable()            do { } while (0)
  #define x_disable()           do { } while (0)
#endif

#ifdef Y_ENABLE_PIN
  #ifdef Y_INVERT_ENABLE
    #define y_enable()          WRITE(Y_ENABLE_PIN, 0)
    #define y_disable()         WRITE(Y_ENABLE_PIN, 1)
  #else
    #define y_enable()          WRITE(Y_ENABLE_PIN, 1)
    #define y_disable()         WRITE(Y_ENABLE_PIN, 0)
  #endif
#else
  #define y_enable()            do { } while (0)
  #define y_disable()           do { } while (0)
#endif

#ifdef Z_ENABLE_PIN
  #ifdef Z_INVERT_ENABLE
    #define _z_en(v)            WRITE(Z_ENABLE_PIN, (v) ^ 1)
  #else
    #define _z_en(v)            WRITE(Z_ENABLE_PIN, v)
  #endif
  #ifdef Z2_ENABLE_PIN
    #ifdef Z_INVERT_ENABLE
      #define _z2_en(v)         WRITE(Z2_ENABLE_PIN, (v) ^ 1)
    #else
      #define _z2_en(v)         WRITE(Z2_ENABLE_PIN, v)
    #endif
  #else
    #define _z2_en(v)           do { } while (0)
  #endif
  #define z_enable()            do { _z_en(1); _z2_en(1); } while (0)
  #define z_disable()           do { _z_en(0); _z2_en(0); } while (0)
#else
  #define z_enable()            do { } while (0)
  #define z_disable()           do { } while (0)
#endif

#ifdef E_ENABLE_PIN
  #ifdef E_INVERT_ENABLE
    #define _e0_en(v)           WRITE(E_ENABLE_PIN, (v) ^ 1)
  #else
    #define _e0_en(v)           WRITE(E_ENABLE_PIN, v)
  #endif
#else
  #define _e0_en(v)             do { } while (0)
#endif
#if EXTRUDERS == 2 && defined E1_ENABLE_PIN
  #ifdef E1_INVERT_ENABLE
    #define _e1_en(v)           WRITE(E1_ENABLE_PIN, (v) ^ 1)
  #else
    #define _e1_en(v)           WRITE(E1_ENABLE_PIN, v)
  #endif
#else
  #define _e1_en(v)             do { } while (0)
#endif
#if defined E_ENABLE_PIN || (EXTRUDERS == 2 && defined E1_ENABLE_PIN)
  #define e_enable()            do { _e0_en(1); _e1_en(1); } while (0)
  #define e_disable()           do { _e0_en(0); _e1_en(0); } while (0)
#else
  #define e_enable()            do { } while (0)
  #define e_disable()           do { } while (0)
#endif

/*
  Internal pull resistors for endstops. Switched on only while needed.
*/
#include "endstops.h"

TEACUP_INLINE void endstops_on(void) {
  endstops_irq_enable(1);
  #ifdef USE_INTERNAL_PULLUPS
    #ifdef X_MIN_PIN
      PULLUP_ON(X_MIN_PIN);
    #endif
    #ifdef X_MAX_PIN
      PULLUP_ON(X_MAX_PIN);
    #endif
    #ifdef Y_MIN_PIN
      PULLUP_ON(Y_MIN_PIN);
    #endif
    #ifdef Y_MAX_PIN
      PULLUP_ON(Y_MAX_PIN);
    #endif
    #ifdef Z_MIN_PIN
      PULLUP_ON(Z_MIN_PIN);
    #endif
    #ifdef Z_MAX_PIN
      PULLUP_ON(Z_MAX_PIN);
    #endif
  #endif

  #ifdef USE_INTERNAL_PULLDOWNS
    #ifdef X_MIN_PIN
      PULLDOWN_ON(X_MIN_PIN);
    #endif
    #ifdef X_MAX_PIN
      PULLDOWN_ON(X_MAX_PIN);
    #endif
    #ifdef Y_MIN_PIN
      PULLDOWN_ON(Y_MIN_PIN);
    #endif
    #ifdef Y_MAX_PIN
      PULLDOWN_ON(Y_MAX_PIN);
    #endif
    #ifdef Z_MIN_PIN
      PULLDOWN_ON(Z_MIN_PIN);
    #endif
    #ifdef Z_MAX_PIN
      PULLDOWN_ON(Z_MAX_PIN);
    #endif
  #endif
}

TEACUP_INLINE void endstops_off(void) {
  endstops_irq_enable(0);
  #if defined USE_INTERNAL_PULLUPS || defined USE_INTERNAL_PULLDOWNS
    #ifdef X_MIN_PIN
      PULL_OFF(X_MIN_PIN);
    #endif
    #ifdef X_MAX_PIN
      PULL_OFF(X_MAX_PIN);
    #endif
    #ifdef Y_MIN_PIN
      PULL_OFF(Y_MIN_PIN);
    #endif
    #ifdef Y_MAX_PIN
      PULL_OFF(Y_MAX_PIN);
    #endif
    #ifdef Z_MIN_PIN
      PULL_OFF(Z_MIN_PIN);
    #endif
    #ifdef Z_MAX_PIN
      PULL_OFF(Z_MAX_PIN);
    #endif
  #endif
}

#endif /* _PINIO_H */
