/** \file
  \brief Board configuration: WeAct "Black Pill" STM32F401CCU6 / STM32F401CEU6.

  Pins are given Arduino style, either PA_5 or PA5, see hal/pins_stm32f4.h.
  On the 48 pin package usable pins are PA0..PA15, PB0..PB10, PB12..PB15,
  PC13..PC15. PA11/PA12 are USB, PA13/PA14 are SWD, PC13 is the on-board
  LED (active low), PA0 is the KEY button, PC14/PC15 carry the 32 kHz crystal.

  Note: STM32 GPIOs are 3.3 V. Thermistor pull-ups must go to 3.3 V (VDDA),
  never to 5 V.
*/

/***************************************************************************\
* 1. CPU                                                                    *
\***************************************************************************/

/** \def F_CPU
  Core clock. STM32F401 runs up to 84 MHz. 84 MHz also gives the exact 48 MHz
  USB clock. Supported values: 48000000, 84000000.
*/
#define F_CPU                    84000000UL

/** \def HSE_CLOCK_HZ
  Frequency of the crystal on the board, integer MHz between 4 and 26.
  WeAct Black Pill boards carry 25 MHz. If the crystal doesn't start, the
  firmware falls back to the internal 16 MHz oscillator automatically and
  still runs at F_CPU (reported at startup as "clock: HSI").
*/
#define HSE_CLOCK_HZ             25000000UL

/***************************************************************************\
* 2. PINOUTS                                                                *
\***************************************************************************/

#define X_STEP_PIN               PA_10
#define X_DIR_PIN                PB_4
#define X_MIN_PIN                PB_12
//#define X_MAX_PIN                PB_13
//#define X_ENABLE_PIN             PA_9
//#define X_INVERT_DIR
#define X_INVERT_MIN
//#define X_INVERT_MAX
//#define X_INVERT_ENABLE

#define Y_STEP_PIN               PB_3
#define Y_DIR_PIN                PB_10
#define Y_MIN_PIN                PB_6
//#define Y_MAX_PIN                PB_7
//#define Y_ENABLE_PIN             PA_9
//#define Y_INVERT_DIR
#define Y_INVERT_MIN
//#define Y_INVERT_MAX
//#define Y_INVERT_ENABLE

//#define Z_STEP_PIN               PB_5
//#define Z_DIR_PIN                PA_8
//#define Z_MIN_PIN                PA_7
//#define Z_MAX_PIN                PA_15
//#define Z_ENABLE_PIN             PA_9
//#define Z_INVERT_DIR
#define Z_INVERT_MIN
//#define Z_INVERT_MAX
//#define Z_INVERT_ENABLE

#define E_STEP_PIN               PA_6
#define E_DIR_PIN                PA_5
//#define E_ENABLE_PIN             PA_9
//#define E_INVERT_DIR
//#define E_INVERT_ENABLE

//#define PS_ON_PIN                PA_4
//#define PS_INVERT_ON
//#define PS_MOSFET_PIN            PA_4
#define STEPPER_ENABLE_PIN       PA_9
/// Most stepper drivers (A4988, DRV8825, TMC) are enabled with a low level.
#define STEPPER_INVERT_ENABLE

/** \def MIN_STEP_PULSE_US
  Minimum high time of step pulses, microseconds.
*/
#define MIN_STEP_PULSE_US        2

/** \def DEBUG_LED_PIN
  Toggled high during the step interrupt. Useful for profiling with a scope.
*/
//#define DEBUG_LED_PIN            PC_13

/** \def BEEPER_PIN BEEPER_ACTIVE
  Buzzer for M300. Passive buzzers get a square wave of the requested
  frequency, active ones (BEEPER_ACTIVE) are just switched on.
*/
//#define BEEPER_PIN               PA_15
//#define BEEPER_ACTIVE

/** \def SD_CARD_SELECT_PIN
  Chip Select of the SD card. Enables SD card support (M20..M25).
*/
//#define SD_CARD_SELECT_PIN       PB_12

/** \def MCP3008_SELECT_PIN
  Chip Select of the MCP3008 ADC, needed for TT_MCP3008 sensors only.
*/
//#define MCP3008_SELECT_PIN       PA_4

/** \def SPI_FLASH
  SPI flash (W25Qxx) on the bottom side footprint of the Black Pill, SPI1:
  CS PA4, SCK PA5, MISO PA6, MOSI PA7. Needs SPI_INSTANCE 1 on these pins
  below, conflicts with E_DIR (PA5) and E_STEP (PA6) of this example.
*/
//#define SPI_FLASH

/** \def SPI_INSTANCE SPI_SCK_PIN SPI_MISO_PIN SPI_MOSI_PIN
  SPI bus for SD card, MAX6675 and MCP3008. Used only when one of those is
  configured. SPI1: PA5/PA6/PA7 or PB3/PB4/PB5, SPI2: PB13/PB14/PB15,
  SPI3: PB3/PB4/PB5 (the firmware picks the right alternate function).
*/
#define SPI_INSTANCE             2
#define SPI_SCK_PIN              PB_13
#define SPI_MISO_PIN             PB_14
#define SPI_MOSI_PIN             PB_15

/** \def I2C_INSTANCE I2C_SCL_PIN I2C_SDA_PIN I2C_SPEED
  I2C bus for the SSD1306 display. I2C1: PB6/PB7 or PB8/PB9.
*/
#define I2C_INSTANCE             1
#define I2C_SCL_PIN              PB_8
#define I2C_SDA_PIN              PB_9
#define I2C_SPEED                400000UL

/***************************************************************************\
* 3. TEMPERATURE SENSORS                                                    *
\***************************************************************************/

#ifndef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(...)
#endif

/** \def TEMP_MAX6675 TEMP_THERMISTOR TEMP_AD595 TEMP_MCP3008 TEMP_DUMMY
  Which temperature sensor types are you using? Leave all used ones
  uncommented, comment out all others.
*/
//#define TEMP_MAX6675
#define TEMP_THERMISTOR
//#define TEMP_AD595
//#define TEMP_MCP3008
//#define TEMP_DUMMY

/** \def DEFINE_TEMP_SENSOR
  One line for each sensor. Name must match the name of the corresponding
  heater. Types: TT_THERMISTOR, TT_AD595 (pin must be an ADC pin: PA0..PA7,
  PB0, PB1, PC0..PC5), TT_MAX6675 (pin is its SPI chip select),
  TT_MCP3008 (pin is MCP_CH0..MCP_CH7), TT_DUMMY (pin is ignored, use any).
  The "additional" field names the thermistor (TT_THERMISTOR, TT_MCP3008,
  THERMISTOR_<name> from DEFINE_THERMISTOR below) and is ignored otherwise.
*/

/** \def DEFINE_THERMISTOR
  Thermistors, converted with the Steinhart-Hart equation. Parameters:
  name, pull-up resistor (Ohm, to 3.3 V!), then three calibration points
  temperature (C) / resistance (Ohm) from the datasheet.
  EPCOS B57560G104F 100k (Marlin type 1): 25/100000, 150/1641.9, 250/226.15.
*/
//                name       pullup  t1    r1        t2     r2       t3     r3
DEFINE_THERMISTOR(epcos100k, 4700.0, 25.0, 100000.0, 150.0, 1641.9,  250.0, 226.15)
//DEFINE_TEMP_SENSORS_START
//                 name      type           pin    additional
DEFINE_TEMP_SENSOR(extruder, TT_THERMISTOR, PB_0,  THERMISTOR_epcos100k)
//DEFINE_TEMP_SENSORS_END

/***************************************************************************\
* 4. HEATERS                                                                *
\***************************************************************************/

/** \def FORCE_SOFTWARE_PWM
  Force software PWM when pwm is set to 1.
*/
//#define FORCE_SOFTWARE_PWM

/** \def DEFINE_HEATER
  Define your heaters and devices here. Names with special meaning:
  extruder (M104), bed (M140), fan (M106).

  Set 'invert' to 1 to invert the pin signal.
  Set 'pwm' to ...
    frequency  in Hertz to use hardware PWM on a pin with a timer channel
               (see hal/pins_stm32f4.h). Valid range 2 to 80'000 Hz. Heaters
               on the same timer share one frequency, the last one wins.
    1          with FORCE_SOFTWARE_PWM: software PWM (~ 100 Hz sigma-delta).
    0          for on/off operation.
  Pins without timer channel fall back to software PWM automatically.
  'max_pwm' limits the output, in percent.

  Timer channels (TIM5 is reserved for the steppers):
    TIM1: PA8 PA9 PA10 PA11, PB13 PB14 PB15 (CHxN)
    TIM2: PA0 PA1 PA2 PA3 PA5 PA15 PB3 PB10
    TIM3: PA6 PA7 PB0 PB1 PB4 PB5
    TIM4: PB6 PB7 PB8 PB9
*/
//DEFINE_HEATERS_START
//            name      pin      invert  pwm      max_pwm
DEFINE_HEATER(extruder, PB_8,    0,      10000,   100)

#define HEATER_EXTRUDER HEATER_extruder
//DEFINE_HEATERS_END

/***************************************************************************\
* 5. COMMUNICATION OPTIONS                                                  *
\***************************************************************************/

/** \def BAUD
  Baud rate for the serial connection to the host.
*/
#define BAUD                     115200

/** \def USB_CDC
  Virtual COM port over the board's USB socket (PA11/PA12). The UART below
  stays active in addition, answers go to the port the command came from.
  Needs F_CPU 48, 84 or 96 MHz. See hal/usb_cdc.h for USB_VID, USB_PID,
  USB_PRODUCT.
*/
#define USB_CDC

/** \def SERIAL_UART SERIAL_TX_PIN SERIAL_RX_PIN
  UART used for the host connection, in addition to USB.
  USART1: PA9/PA10 or PB6/PB7, USART2: PA2/PA3, USART6: PA11/PA12 (not
  with USB_CDC).
*/
#define SERIAL_UART              2
#define SERIAL_TX_PIN            PA_2
#define SERIAL_RX_PIN            PA_3

/** \def XONXOFF
  Xon/Xoff flow control. Needed when sending G-code files with a plain
  terminal emulator.
*/
//#define XONXOFF

/***************************************************************************\
* 6. DISPLAY SUPPORT                                                        *
\***************************************************************************/

/** \def DISPLAY_BUS_4BIT DISPLAY_BUS_I2C
  The bus used to connect the display to the controller.
*/
//#define DISPLAY_BUS_4BIT
//#define DISPLAY_BUS_I2C

/** \def DISPLAY_RS_PIN DISPLAY_RW_PIN DISPLAY_E_PIN
    \def DISPLAY_D4_PIN DISPLAY_D5_PIN DISPLAY_D6_PIN DISPLAY_D7_PIN
  Pins for the 4-bit parallel display bus. Use 5 V tolerant pins (all except
  PA0..PA7, PB0, PB1, PB5) if the display runs on 5 V.
*/
//#define DISPLAY_RS_PIN           PB_12
//#define DISPLAY_RW_PIN           PB_13
//#define DISPLAY_E_PIN            PB_14
//#define DISPLAY_D4_PIN           PB_15
//#define DISPLAY_D5_PIN           PA_8
//#define DISPLAY_D6_PIN           PA_15
//#define DISPLAY_D7_PIN           PB_9

/** \def DISPLAY_TYPE_SSD1306 DISPLAY_TYPE_HD44780
  The type of display in use.
*/
//#define DISPLAY_TYPE_SSD1306
//#define DISPLAY_TYPE_HD44780

/***************************************************************************\
* 7. ANALOG                                                                 *
\***************************************************************************/

/** \def OVERSAMPLE
  Number of ADC samples averaged per reading (min and max get discarded).
*/
#define OVERSAMPLE               16
