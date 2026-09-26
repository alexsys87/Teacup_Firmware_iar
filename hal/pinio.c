/** \file
  \brief I/O initialisation and power supply handling.

  Power supply switching, two pins, both optional:

  PS_ON_PIN without PS_INVERT_ON: ATX PS_ON wired straight to the pin,
    active low. On = output low, off = input (released, the supply pulls
    PS_ON up to 5 V). The pin must be 5 V tolerant (FT).

  PS_ON_PIN with PS_INVERT_ON: active high, push-pull, driven all the time.
    For a relay / SSR module, or ATX PS_ON through an NPN transistor or
    optocoupler. On = high, off = low. Needs a pull-down (10 k) at the module
    input, the pin floats between reset and pinio_init().

  PS_MOSFET_PIN: active high, e.g. a MOSFET in the 12 V line of the drivers.

  The supply is switched on when a heater or a stepper needs it, or by M80.
  It's switched off by printer_kill() (every thermal error, M112), M81 and
  M2; after 30 s idle only with PS_AUTO_OFF. The MCU itself must not be
  powered from the switched supply (use USB, ATX 5VSB or a separate 5 V).
*/

#include "pinio.h"

#ifdef Z_STEPPER_ALIGN
  volatile uint8_t z_step_mask = 3;
#endif
#include "delay.h"
#include "home.h"

static uint8_t ps_is_on = 0;

/// PS_ON inactive: supply off.
static void ps_on_off(void) {
  #ifdef PS_ON_PIN
    #ifdef PS_INVERT_ON
      WRITE(PS_ON_PIN, 0);
      SET_OUTPUT(PS_ON_PIN);
    #else
      SET_INPUT(PS_ON_PIN);
    #endif
  #endif
}

/// step/psu timeout
volatile uint8_t psu_timeout = 0;

/** Initialise all I/O.

  This sets pins as input or output, appropriate for their usage. Levels are
  written before switching a pin to output to avoid glitches.
*/
void pinio_init(void) {
  /// X Stepper.
  WRITE(X_STEP_PIN, 0); SET_OUTPUT(X_STEP_PIN);
  WRITE(X_DIR_PIN, 0);  SET_OUTPUT(X_DIR_PIN);
  #ifdef X_MIN_PIN
    SET_INPUT(X_MIN_PIN);
  #endif
  #ifdef X_MAX_PIN
    SET_INPUT(X_MAX_PIN);
  #endif

  /// Y Stepper.
  WRITE(Y_STEP_PIN, 0); SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_DIR_PIN, 0);  SET_OUTPUT(Y_DIR_PIN);
  #ifdef Y_MIN_PIN
    SET_INPUT(Y_MIN_PIN);
  #endif
  #ifdef Y_MAX_PIN
    SET_INPUT(Y_MAX_PIN);
  #endif

  /// Z Stepper.
  #if defined Z_STEP_PIN && defined Z_DIR_PIN
    WRITE(Z_STEP_PIN, 0); SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_DIR_PIN, 0);  SET_OUTPUT(Z_DIR_PIN);
  #endif
  #if defined Z2_STEP_PIN
    WRITE(Z2_STEP_PIN, 0); SET_OUTPUT(Z2_STEP_PIN);
  #endif
  #if defined Z2_STEP_PIN && defined Z2_DIR_PIN
    WRITE(Z2_DIR_PIN, 0);  SET_OUTPUT(Z2_DIR_PIN);
  #endif
  #ifdef Z_MIN_PIN
    SET_INPUT(Z_MIN_PIN);
  #endif
  #ifdef Z_MAX_PIN
    SET_INPUT(Z_MAX_PIN);
  #endif

  /// E Stepper.
  #if defined E_STEP_PIN && defined E_DIR_PIN
    WRITE(E_STEP_PIN, 0); SET_OUTPUT(E_STEP_PIN);
    WRITE(E_DIR_PIN, 0);  SET_OUTPUT(E_DIR_PIN);
  #endif

  /// Enable pins, all start disabled.
  #ifdef STEPPER_ENABLE_PIN
    stepper_disable();
    SET_OUTPUT(STEPPER_ENABLE_PIN);
  #endif
  #ifdef X_ENABLE_PIN
    x_disable();
    SET_OUTPUT(X_ENABLE_PIN);
  #endif
  #ifdef Y_ENABLE_PIN
    y_disable();
    SET_OUTPUT(Y_ENABLE_PIN);
  #endif
  #ifdef Z_ENABLE_PIN
    z_disable();
    SET_OUTPUT(Z_ENABLE_PIN);
    #ifdef Z2_ENABLE_PIN
      SET_OUTPUT(Z2_ENABLE_PIN);
    #endif
  #endif
  #ifdef E_ENABLE_PIN
    e_disable();
    SET_OUTPUT(E_ENABLE_PIN);
  #endif

  ps_on_off();

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 0);
    SET_OUTPUT(DEBUG_LED_PIN);
  #endif
}

void power_on(void) {

  if (ps_is_on == 0) {
    #ifdef PS_ON_PIN
      #ifdef PS_INVERT_ON
        WRITE(PS_ON_PIN, 1);
      #else
        WRITE(PS_ON_PIN, 0);
      #endif
      SET_OUTPUT(PS_ON_PIN);
      delay_ms(500);
    #endif
    #ifdef PS_MOSFET_PIN
      WRITE(PS_MOSFET_PIN, 1);
      delay_ms(10);
    #endif
    ps_is_on = 1;
  }

  psu_timeout = 0;
}

void power_off(void) {
  axes_homed = 0;                     // Motors lose their position.

  stepper_disable();
  x_disable();
  y_disable();
  z_disable();
  e_disable();

  ps_on_off();

  #ifdef PS_MOSFET_PIN
    WRITE(PS_MOSFET_PIN, 0);
  #endif

  ps_is_on = 0;
}

void power_idle(void) {
  // Steppers have their own idle timeout (STEPPER_IDLE_TIMEOUT, M84 S),
  // this one used to disable them after 30 s regardless, even with M84 S0.
  #ifdef PS_AUTO_OFF
    power_off();
  #endif
}

uint8_t power_is_on(void) {
  return ps_is_on;
}

uint8_t steppers_enabled(void) {
  #ifdef STEPPER_ENABLE_PIN
    uint8_t level = (PIN_PORT(STEPPER_ENABLE_PIN)->ODR &
                     MASK(PIN_NUM(STEPPER_ENABLE_PIN))) ? 1 : 0;

    #ifdef STEPPER_INVERT_ENABLE
      return ! level;
    #else
      return level;
    #endif
  #else
    return ps_is_on;
  #endif
}

void steppers_enable_all(void) {
  power_on();
  stepper_enable();
  x_enable();
  y_enable();
  z_enable();
  e_enable();
}

void steppers_disable_all(void) {
  axes_homed = 0;                     // Motors lose their position.
  stepper_disable();
  x_disable();
  y_disable();
  z_disable();
  e_disable();
}

static uint16_t stepper_idle_timeout = STEPPER_IDLE_TIMEOUT;
static uint16_t stepper_idle_seconds = 0;

void steppers_idle_tick(uint8_t active) {
  if (active) {
    stepper_idle_seconds = 0;
    return;
  }
  if (stepper_idle_seconds < 0xFFFF)
    stepper_idle_seconds++;
  if (stepper_idle_timeout && stepper_idle_seconds == stepper_idle_timeout)
    steppers_disable_all();
}

void steppers_set_idle_timeout(uint16_t seconds) {
  stepper_idle_timeout = seconds;
  stepper_idle_seconds = 0;
}

uint16_t steppers_get_idle_timeout(void) {
  return stepper_idle_timeout;
}
