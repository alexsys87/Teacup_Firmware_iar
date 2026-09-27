/** \file
  \brief Configuration wrapper, included by all modules instead of config.h.

  This file is intentionally included several times per translation unit
  (X-macro technique for DEFINE_TEMP_SENSOR, DEFINE_HEATER and
  DEFINE_HOMING). It therefore has no include guard and everything in here
  must be safe for repeated inclusion.
*/

#include "arch.h"
#include "pins_stm32f4.h"

#ifndef DEFINE_HEATER_ACTUAL
  #define DEFINE_HEATER_ACTUAL(...)
#endif

// Heater definition helpers. These turn any DEFINE_HEATER(...) with 3 to 6
// arguments into DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_pwm).
// See http://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
#define GET_MACRO(_1, _2, _3, _4, _5, _6, NAME, ...) NAME
#define DEFINE_HEATER(...) GET_MACRO(__VA_ARGS__, DHTR_6, DHTR_5, DHTR_4, DHTR_3)(__VA_ARGS__)
#define DHTR_3(name, pin, pwm) DEFINE_HEATER_ACTUAL(name, pin, 0, pwm, 100)
#define DHTR_4(name, pin, invert, pwm) DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, 100)
#define DHTR_5(name, pin, invert, pwm, max_pwm)  DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_pwm)
#define DHTR_6(name, pin, invert, pwm, max_pwm, arg6)  DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_pwm)

/**
  Thermistors for TT_THERMISTOR and TT_MCP3008 sensors, converted with the
  Steinhart-Hart equation from three calibration points:

    DEFINE_THERMISTOR(name, pullup_ohm, t1_C, r1_ohm, t2_C, r2_ohm, t3_C, r3_ohm)

  A sensor refers to it as THERMISTOR_<name> in the 'additional' field.
*/
#ifndef DEFINE_THERMISTOR
  #define DEFINE_THERMISTOR(...)
#endif

#ifndef DEFINE_HOMING_ACTUAL
  #define DEFINE_HOMING_ACTUAL(...)
#endif

// Homing movements helpers.
#define GET_MACRO_HOME(_1, _2, _3, _4, NAME, ...) NAME
#define DEFINE_HOMING(...) GET_MACRO_HOME(__VA_ARGS__, DHM_4, DHM_3, DHM_2, DHM_1)(__VA_ARGS__)
#define DHM_1(STEP_1) DEFINE_HOMING_ACTUAL(STEP_1, none, none, none)
#define DHM_2(STEP_1, STEP_2) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, none, none)
#define DHM_3(STEP_1, STEP_2, STEP_3) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, STEP_3, none)
#define DHM_4(STEP_1, STEP_2, STEP_3, STEP_4) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, STEP_3, STEP_4)

/**
  Virtual "pins" for TT_MCP3008 temperature sensors: the number is the
  MCP3008 input channel, e.g. DEFINE_TEMP_SENSOR(bed, TT_MCP3008, MCP_CH1, ...).
*/
#define MCP_CH0_ADC 0
#define MCP_CH1_ADC 1
#define MCP_CH2_ADC 2
#define MCP_CH3_ADC 3
#define MCP_CH4_ADC 4
#define MCP_CH5_ADC 5
#define MCP_CH6_ADC 6
#define MCP_CH7_ADC 7
#define MCP_CH0_ID  0xFF
#define MCP_CH1_ID  0xFF
#define MCP_CH2_ID  0xFF
#define MCP_CH3_ID  0xFF
#define MCP_CH4_ID  0xFF
#define MCP_CH5_ID  0xFF
#define MCP_CH6_ID  0xFF
#define MCP_CH7_ID  0xFF

#include "config.h"

/**
  Give users a hint in case they obviously forgot to read instructions.
*/
#ifndef STEPS_PER_M_X
  #error config.h missing or incomplete.
#endif

#ifndef F_CPU
  #error F_CPU not defined in the board configuration.
#endif

#ifndef HSE_CLOCK_HZ
  #define HSE_CLOCK_HZ 25000000UL
#endif

/**
  USB CDC virtual COM port (hal/usb_cdc.c) on PA11/PA12, in addition to the
  UART. USB_SERIAL is the old Teacup name of the option.
*/
#if defined USB_SERIAL && ! defined USB_CDC
  #define USB_CDC
#endif

/**
  Features not available on STM32F4.
*/
#ifdef EECONFIG
  #error EECONFIG is not supported on STM32F4 (there is no EEPROM). Use M130..M133 at runtime.
#endif
#ifdef TEMP_INTERCOM
  #error TEMP_INTERCOM (Gen3 extruder boards) is AVR only and not supported on STM32F4.
#endif
#ifdef TEMP_PT100
  #error TEMP_PT100 is not implemented.
#endif

/**
  SPI flash (W25Qxx) on the Black Pill's bottom side footprint, SPI1:
  CS PA4, SCK PA5, MISO PA6, MOSI PA7. Uses: settings (M500) and G-code
  files (M28/M20/M23/M24) if no SD card is configured.
*/
#ifdef SPI_FLASH
  #ifndef SPI_FLASH_CS_PIN
    #define SPI_FLASH_CS_PIN PA_4
  #endif
  #if ! defined SPI_INSTANCE || SPI_INSTANCE != 1
    #error SPI_FLASH needs SPI_INSTANCE 1 (PA5 SCK, PA6 MISO, PA7 MOSI).
  #endif
  #ifndef NO_SPI_FLASH_SETTINGS
    #define SPI_FLASH_SETTINGS
  #endif
  #if ! defined SD_CARD_SELECT_PIN && ! defined NO_SPI_FLASH_FILES
    #define SPI_FLASH_FILES
  #endif
#endif

/**
  Check wether we need SPI.
*/
#if defined SD_CARD_SELECT_PIN || defined TEMP_MAX6675 || defined TEMP_MAX31865 || \
    defined TEMP_MCP3008 || defined SPI_FLASH
  #define SPI
#endif

/**
  Check wether we need I2C.
*/
#if defined DISPLAY_BUS_I2C || defined PCF8574_ADDRESS
  #define I2C
#endif

/**
  ACCELERATION_TEMPORAL doesn't support lookahead, yet. For
  ACCELERATION_REPRAP or no acceleration at all lookahead makes no sense.
*/
#if ! defined ACCELERATION_RAMPING
  #undef LOOKAHEAD
#endif

/**
  Linear advance steps E on its own, based on the speed profile of
  ACCELERATION_RAMPING with lookahead.
*/
#if defined LINEAR_ADVANCE && ! defined LOOKAHEAD
  #warning LINEAR_ADVANCE needs ACCELERATION_RAMPING and LOOKAHEAD, ignored.
  #undef LINEAR_ADVANCE
#endif
#ifndef LINEAR_ADVANCE_K
  #define LINEAR_ADVANCE_K 0.0
#endif

/**
  Input shaping and S-curve smoothing of X and Y step the axes on their
  own, like linear advance does with E. Not with ACCELERATION_TEMPORAL,
  which has a step path of its own.
*/
#if defined INPUT_SHAPING && ! defined ACCELERATION_RAMPING
  #warning INPUT_SHAPING needs ACCELERATION_RAMPING, ignored.
  #undef INPUT_SHAPING
#endif
#ifndef INPUT_SHAPING_BUFFER
  #define INPUT_SHAPING_BUFFER 1024
#endif
#ifndef INPUT_SHAPING_FREQ_X
  #define INPUT_SHAPING_FREQ_X 0.0
#endif
#ifndef INPUT_SHAPING_FREQ_Y
  #define INPUT_SHAPING_FREQ_Y 0.0
#endif
#ifndef INPUT_SHAPING_DAMPING_X
  #define INPUT_SHAPING_DAMPING_X 0.1
#endif
#ifndef INPUT_SHAPING_DAMPING_Y
  #define INPUT_SHAPING_DAMPING_Y 0.1
#endif
#ifndef INPUT_SHAPING_TYPE_X
  #define INPUT_SHAPING_TYPE_X 1
#endif
#ifndef INPUT_SHAPING_TYPE_Y
  #define INPUT_SHAPING_TYPE_Y 1
#endif
#ifndef S_CURVE_TIME
  #define S_CURVE_TIME 0
#endif

/// Auxiliary step generator (hal/timer.c) for linear advance and shaping.
#if defined LINEAR_ADVANCE || defined INPUT_SHAPING
  #define STEP_AUX
#endif

#if defined BED_LEVELING && defined LOOKAHEAD && MAX_JERK_Z == 0
  #warning When bed-leveling is activated, lookahead will be ineffective \
           because MAX_JERK_Z is zero.
#endif

/**
  TEMP_EWMA is an integer 1..1000 (1000 = filter off). A value like 1.0
  (the old float notation) would make readings extremely slow.
*/
#if defined TEMP_EWMA
  #if TEMP_EWMA < 2
    #error TEMP_EWMA is scaled 1..1000 now (1000 = off). Fix your printer config.
  #endif
#endif

/**
  Minimum width of step pulses in microseconds. A4988 needs 1 us,
  DRV8825 1.9 us, TMC drivers 0.1 us.
*/
#ifndef MIN_STEP_PULSE_US
  #define MIN_STEP_PULSE_US 2
#endif

#ifndef ENDSTOP_STEPS
  #define ENDSTOP_STEPS 4
#endif

/**
  Thermal protection defaults (Marlin's values), so protection is active
  even with printer configs which don't mention it. See printer.mendel.h.
  TEMP_ERROR_READINGS: consecutive readings beyond MINTEMP/MAXTEMP needed.
  NO_THERMAL_PROTECTION disables runaway/heating watch and the heater stuck
  on check, never MIN/MAXTEMP.
*/
#ifndef THERMAL_PROTECTION_PERIOD
  #define THERMAL_PROTECTION_PERIOD          40
#endif
#ifndef THERMAL_PROTECTION_HYSTERESIS
  #define THERMAL_PROTECTION_HYSTERESIS      4
#endif
#ifndef WATCH_TEMP_PERIOD
  #define WATCH_TEMP_PERIOD                  20
#endif
#ifndef WATCH_TEMP_INCREASE
  #define WATCH_TEMP_INCREASE                2
#endif
#ifndef THERMAL_PROTECTION_BED_PERIOD
  #define THERMAL_PROTECTION_BED_PERIOD      20
#endif
#ifndef THERMAL_PROTECTION_BED_HYSTERESIS
  #define THERMAL_PROTECTION_BED_HYSTERESIS  2
#endif
#ifndef WATCH_BED_TEMP_PERIOD
  #define WATCH_BED_TEMP_PERIOD              60
#endif
#ifndef WATCH_BED_TEMP_INCREASE
  #define WATCH_BED_TEMP_INCREASE            2
#endif
#ifndef HEATER_MINTEMP
  #define HEATER_MINTEMP                     5
#endif
#ifndef HEATER_MAXTEMP
  #define HEATER_MAXTEMP                     275
#endif
#ifndef BED_MINTEMP
  #define BED_MINTEMP                        5
#endif
#ifndef BED_MAXTEMP
  #define BED_MAXTEMP                        125
#endif
#ifndef HOTEND_OVERSHOOT
  #define HOTEND_OVERSHOOT                   15
#endif
#ifndef BED_OVERSHOOT
  #define BED_OVERSHOOT                      10
#endif
#ifndef TEMP_SENSOR_TIMEOUT
  #define TEMP_SENSOR_TIMEOUT                4
#endif
#ifndef THERMAL_PROTECTION_OFF_PERIOD
  #define THERMAL_PROTECTION_OFF_PERIOD      20
#endif
#ifndef THERMAL_PROTECTION_OFF_RISE
  #define THERMAL_PROTECTION_OFF_RISE        20
#endif
#ifndef THERMAL_PROTECTION_BED_OFF_PERIOD
  #define THERMAL_PROTECTION_BED_OFF_PERIOD  60
#endif
#ifndef THERMAL_PROTECTION_BED_OFF_RISE
  #define THERMAL_PROTECTION_BED_OFF_RISE    10
#endif
#ifndef TEMP_ERROR_READINGS
  #define TEMP_ERROR_READINGS                3
#endif
#ifndef TEMP_HYSTERESIS
  #define TEMP_HYSTERESIS                    10
#endif
/**
  Endstop interrupts (EXTI): react to an endstop edge immediately instead
  of after ENDSTOP_STEPS polls. NO_ENDSTOP_INTERRUPTS disables them.
*/
#ifndef NO_ENDSTOP_INTERRUPTS
  #define ENDSTOP_INTERRUPTS
#endif

/**
  Serial via DMA: no interrupt per character. NO_SERIAL_DMA falls back to
  the interrupt driven implementation.
*/
/*
  STEP_CODE_IN_RAM is defined in the config, after arch.h was read.
*/
#ifdef STEP_CODE_IN_RAM
  #undef  TEACUP_STEP_RAMFUNC
  #define TEACUP_STEP_RAMFUNC TEACUP_RAMFUNC
#endif

#if defined STEP_TIMER_PULSES && defined ACCELERATION_TEMPORAL
  #warning STEP_TIMER_PULSES is not supported with ACCELERATION_TEMPORAL, ignored.
#endif

#ifndef NO_SERIAL_DMA
  #define SERIAL_DMA
#endif

/**
  G2/G3 arcs (NO_ARC_SUPPORT switches them off). The arc is split into
  straight segments with at most ARC_TOLERANCE mm deviation from the true
  arc, each between ARC_SEGMENT_MIN and ARC_SEGMENT_MAX mm long.
*/
#ifndef ARC_TOLERANCE
  #define ARC_TOLERANCE     0.01
#endif
#ifndef ARC_SEGMENT_MIN
  #define ARC_SEGMENT_MIN   0.1
#endif
#ifndef ARC_SEGMENT_MAX
  #define ARC_SEGMENT_MAX   1.0
#endif

#ifndef STEPPER_IDLE_TIMEOUT
  #define STEPPER_IDLE_TIMEOUT 120
#endif

/**
  Filament runout sensor (FILAMENT_RUNOUT_PIN, board file) and filament
  change (M600, always available). See core/filament.c.

  FILAMENT_RUNOUT_STATE     pin level at runout (0 = low).
  FILAMENT_RUNOUT_DISTANCE  mm of filament still extruded after the sensor
                            triggered, before the print pauses (M412 D).
  FILAMENT_RUNOUT_DEBOUNCE  consecutive readings (10 ms apart) needed.
*/
#ifdef FILAMENT_RUNOUT_PIN
  #ifndef FILAMENT_RUNOUT_STATE
    #define FILAMENT_RUNOUT_STATE      0
  #endif
  #ifndef FILAMENT_RUNOUT_DISTANCE
    #define FILAMENT_RUNOUT_DISTANCE   25.0
  #endif
  #ifndef FILAMENT_RUNOUT_DEBOUNCE
    #define FILAMENT_RUNOUT_DEBOUNCE   5
  #endif
#endif
#ifndef FILAMENT_CHANGE_RETRACT
  #define FILAMENT_CHANGE_RETRACT            2.0
#endif
#ifndef FILAMENT_CHANGE_Z_LIFT
  #define FILAMENT_CHANGE_Z_LIFT             20.0
#endif
#ifndef FILAMENT_CHANGE_PARK_X
  #ifdef X_MIN
    #define FILAMENT_CHANGE_PARK_X           (X_MIN + 10.0)
  #else
    #define FILAMENT_CHANGE_PARK_X           10.0
  #endif
#endif
#ifndef FILAMENT_CHANGE_PARK_Y
  #ifdef Y_MAX
    #define FILAMENT_CHANGE_PARK_Y           (Y_MAX - 10.0)
  #else
    #define FILAMENT_CHANGE_PARK_Y           10.0
  #endif
#endif
#ifndef FILAMENT_CHANGE_UNLOAD
  #define FILAMENT_CHANGE_UNLOAD             50.0
#endif
#ifndef FILAMENT_CHANGE_LOAD
  #define FILAMENT_CHANGE_LOAD               40.0
#endif
#ifndef FILAMENT_CHANGE_RETRACT_FEEDRATE
  #define FILAMENT_CHANGE_RETRACT_FEEDRATE   1500
#endif
#ifndef FILAMENT_CHANGE_UNLOAD_FEEDRATE
  #define FILAMENT_CHANGE_UNLOAD_FEEDRATE    1200
#endif
#ifndef FILAMENT_CHANGE_LOAD_FEEDRATE
  #define FILAMENT_CHANGE_LOAD_FEEDRATE      180
#endif
#ifndef FILAMENT_CHANGE_XY_FEEDRATE
  #define FILAMENT_CHANGE_XY_FEEDRATE        6000
#endif
#ifndef FILAMENT_CHANGE_NOZZLE_TIMEOUT
  #define FILAMENT_CHANGE_NOZZLE_TIMEOUT     300
#endif
#ifndef FILAMENT_CHANGE_MIN_TEMP
  #define FILAMENT_CHANGE_MIN_TEMP           170
#endif

/**
  Babystepping (M290): moves Z right away, also during a print. The total
  is a Z offset, stored with M500. Needs a Z stepper.
*/
#if defined BABYSTEPPING && ! (defined Z_STEP_PIN && defined Z_DIR_PIN)
  #undef BABYSTEPPING
#endif
#ifdef BABYSTEPPING
  #ifdef ACCELERATION_TEMPORAL
    #error BABYSTEPPING is not supported with ACCELERATION_TEMPORAL.
  #endif
  #ifndef BABYSTEP_FEEDRATE
    #define BABYSTEP_FEEDRATE    60
  #endif
  #ifndef BABYSTEP_LIMIT
    #define BABYSTEP_LIMIT       2.0
  #endif
#endif

/**
  Mesh bed leveling (BED_LEVELING): bilinear grid, G29 with a probe or
  M421 by hand, M420 on/off and fade height. See src/bed_leveling.c.
*/
#ifdef BED_LEVELING
  #ifndef GRID_POINTS_X
    #define GRID_POINTS_X        3
  #endif
  #ifndef GRID_POINTS_Y
    #define GRID_POINTS_Y        GRID_POINTS_X
  #endif
  #if GRID_POINTS_X < 2 || GRID_POINTS_X > 7 || GRID_POINTS_Y < 2 || GRID_POINTS_Y > 7
    #error GRID_POINTS_X and GRID_POINTS_Y must be 2..7.
  #endif
  #ifndef MESH_INSET
    #define MESH_INSET           15.0
  #endif
  #ifndef LEVELING_FADE_HEIGHT
    #define LEVELING_FADE_HEIGHT 10.0
  #endif
  #if ! defined X_MIN || ! defined X_MAX || ! defined Y_MIN || ! defined Y_MAX
    #error BED_LEVELING needs X_MIN, X_MAX, Y_MIN and Y_MAX.
  #endif
#endif

/**
  BLTouch (board file): Z probe signal on Z_MIN_PIN, servo signal on
  BLTOUCH_SERVO_PIN. Makes G28 Z home with the probe, adds G29 / G30,
  M280, M401, M402, M851. See src/probe.c.
*/
#if ! defined BLTOUCH
  #undef BLTOUCH_SERVO_PIN              // Pin reserved, but no servo.
#endif
#if defined BLTOUCH && defined INDUCTIVE_PROBE
  #error Choose BLTOUCH or INDUCTIVE_PROBE, not both.
#endif
#ifdef BLTOUCH
  // The BLTouch signal is active high, whatever the Z switch was.
  #undef Z_INVERT_MIN
#endif
/**
  Inductive / capacitive proximity sensor on Z_MIN_PIN: active low (NPN
  NO) unless INDUCTIVE_PROBE_ACTIVE_HIGH. No servo, nothing to deploy.
*/
#ifdef INDUCTIVE_PROBE
  #undef Z_INVERT_MIN
  #ifndef INDUCTIVE_PROBE_ACTIVE_HIGH
    #define Z_INVERT_MIN
  #endif
#endif
#if defined BLTOUCH || defined INDUCTIVE_PROBE
  #define Z_PROBE
  #if defined BLTOUCH && ! defined BLTOUCH_SERVO_PIN
    #error BLTOUCH needs BLTOUCH_SERVO_PIN.
  #endif
  #ifndef Z_MIN_PIN
    #error A Z probe needs Z_MIN_PIN (probe signal).
  #endif
  #if ! defined X_MIN || ! defined X_MAX || ! defined Y_MIN || ! defined Y_MAX
    #error A Z probe needs X_MIN, X_MAX, Y_MIN and Y_MAX.
  #endif
  #ifndef Z_PROBE_OFFSET_X
    #define Z_PROBE_OFFSET_X     0.0
  #endif
  #ifndef Z_PROBE_OFFSET_Y
    #define Z_PROBE_OFFSET_Y     0.0
  #endif
  #ifndef Z_PROBE_OFFSET_Z
    #define Z_PROBE_OFFSET_Z     0.0
  #endif
  #ifndef Z_PROBE_FEEDRATE_FAST
    #define Z_PROBE_FEEDRATE_FAST 240
  #endif
  #ifndef Z_PROBE_FEEDRATE_SLOW
    #define Z_PROBE_FEEDRATE_SLOW 60
  #endif
  #ifndef Z_PROBE_SAMPLES
    #define Z_PROBE_SAMPLES      2
  #endif
  #ifndef Z_PROBE_CLEARANCE
    #define Z_PROBE_CLEARANCE    5.0
  #endif
  #ifndef Z_PROBE_RETRACT
    #define Z_PROBE_RETRACT      3.0
  #endif
  #ifndef Z_PROBE_LOW_POINT
    #define Z_PROBE_LOW_POINT    -2.0
  #endif
  #ifndef Z_PROBE_XY_FEEDRATE
    #define Z_PROBE_XY_FEEDRATE  6000
  #endif
  #ifndef BLTOUCH_DELAY
    #define BLTOUCH_DELAY        500
  #endif
#endif
/**
  Multi-stepping: with ACCELERATION_RAMPING only. MULTISTEP_MIN_CYCLES:
  below this step interval (CPU cycles) 2, then 4, up to MULTISTEP_MAX
  steps per interrupt.
*/
#ifdef MULTISTEPPING
  #ifndef ACCELERATION_RAMPING
    #undef MULTISTEPPING
  #endif
  #ifndef MULTISTEP_MIN_CYCLES
    #define MULTISTEP_MIN_CYCLES 840      // 100 kHz at 84 MHz.
  #endif
  #ifndef MULTISTEP_MAX
    #define MULTISTEP_MAX        8
  #endif
#endif

/**
  Power loss recovery: records in the SPI flash, for SD / flash prints.
*/
#if defined POWER_LOSS_RECOVERY && ! (defined SPI_FLASH && \
    (defined SD_CARD_SELECT_PIN || defined SPI_FLASH_FILES))
  #undef POWER_LOSS_RECOVERY
#endif

/**
  G34 (printer config): Z and Z2 with separate STEP pins, a Z probe.
*/
#ifdef Z_STEPPER_ALIGN
  #ifndef Z_PROBE
    #error Z_STEPPER_ALIGN (G34) needs a Z probe (BLTOUCH or INDUCTIVE_PROBE).
  #endif
  #if ! defined Z2_STEP_PIN || ! defined Z_STEP_PIN
    #error Z_STEPPER_ALIGN (G34) needs Z_STEP_PIN and Z2_STEP_PIN.
  #endif
  #ifndef Z_STEPPER_ALIGN_X1
    #define Z_STEPPER_ALIGN_X1   (X_MIN + 10.0)
    #define Z_STEPPER_ALIGN_X2   (X_MAX - 10.0)
  #endif
  #ifndef Z_STEPPER_ALIGN_Y
    #define Z_STEPPER_ALIGN_Y    ((Y_MIN + Y_MAX) / 2.0)
  #endif
  #ifndef Z_STEPPER_X1
    #define Z_STEPPER_X1         Z_STEPPER_ALIGN_X1
    #define Z_STEPPER_X2         Z_STEPPER_ALIGN_X2
  #endif
  #ifndef Z_STEPPER_ALIGN_ITERATIONS
    #define Z_STEPPER_ALIGN_ITERATIONS 5
  #endif
  #ifndef Z_STEPPER_ALIGN_ACC
    #define Z_STEPPER_ALIGN_ACC  0.02
  #endif
  #ifndef Z_STEPPER_ALIGN_MAX
    #define Z_STEPPER_ALIGN_MAX  5.0
  #endif
#endif

#ifdef NO_THERMAL_PROTECTION
  #warning NO_THERMAL_PROTECTION: thermal runaway protection is DISABLED.
#endif
