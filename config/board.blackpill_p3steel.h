/** \file
  \brief Board configuration: WeAct "Black Pill" (STM32F401 or STM32F411)
         driving a Prusa i3 Steel ("P3 Steel") printer.

  Taken over from the printer's Marlin configuration (RAMPS 1.4 EFB):
  DRV8825 drivers at 1/32, 100k EPCOS thermistors for hotend and bed,
  endstops at MIN, active low with pull-ups.

  Final pin plan for this board, including the planned extensions, so no
  pin has to move again (see README, "P3 Steel: pinout"):

    Pin   Function                 Pin   Function
    PA0   filament runout (KEY)    PB0   bed heater (on/off)
    PA1   T0 hotend thermistor     PB1   T1 bed thermistor
    PA2   UART TX (host)           PB2   PS_ON (BOOT1)
    PA3   UART RX (host)           PB3   Y_MIN        (EXTI3)
    PA4   SPI flash CS             PB4   E_STEP       (TIM3_CH1)
    PA5   SPI SCK                  PB5   E_DIR
    PA6   SPI MISO                 PB6   I2C1 SCL: display, PCF8574
    PA7   SPI MOSI                 PB7   I2C1 SDA: display, PCF8574
    PA8   X_STEP       (TIM1_CH1)  PB8   hotend heater (TIM10_CH1)
    PA9   X_DIR                    PB9   part fan      (TIM11_CH1)
    PA10  Y_DIR                    PB10  X_MIN        (EXTI10)
    PA11  USB DM                   PB12  Z_STEP (Z and Z2 drivers)
    PA12  USB DP                   PB13  Z_DIR  (Z and Z2 drivers)
    PA13  SWDIO                    PB14  EN (all drivers)
    PA14  SWCLK                    PB15  Z_MIN / Z probe (EXTI15)
    PA15  Y_STEP       (TIM2_CH1)  PC13  BLTouch servo (LED on the pin)
                                   PC14  SD card CS (*, remove 32 kHz xtal)
                                   PC15  spare (*, Z2_STEP for G34 / MAX31865)

    (*) stand-alone printing, optional. Encoder, buttons and beeper go to a
        PCF8574 I/O expander on the I2C bus, no pins needed; its P6 / P7
        switch the hotend fan and the controller fan.

  Changes to the wiring against the first version of this file:
   - Z2 driver: STEP and DIR in parallel to the Z driver (PB12 / PB13).
     Firmware moved Z and Z2 identically anyway. Z2_INVERT_DIR is gone:
     reverse the Z2 motor connector if it ran inverted.
   - STEP of X, Y and E on timer channels: STEP_TIMER_PULSES, the step
     interrupt only starts the timers, no pulse end interrupt.
   - Hotend PB8 and fan PB9 on TIM10 / TIM11, TIM1..TIM3 drive the steps.

  Rules:
   - PA11/PA12 are USB (USB_CDC), PA13/PA14 SWD, PA2/PA3 the host UART.
   - Thermistors need ADC pins: PA0..PA7, PB0, PB1 (not 5 V tolerant!).
   - Endstops and probes powered from 5 V need 5 V tolerant pins (all
     except PA0..PA7, PB0, PB1, PB5): PB3, PB10, PB15 are. The runout
     input PA0 is not: a switch to GND or a 3.3 V sensor, a 5 V sensor
     through a divider.

  WARNING for RAMPS 1.4 as power stage:
   - The thermistor pull-ups on RAMPS go to 5 V. The STM32 ADC tolerates
     3.6 V at most: move the 4.7k pull-ups to 3.3 V (VDDA).
   - The MOSFETs on RAMPS (STP55NF06L) are not fully on with 3.3 V gate
     voltage and overheat, especially the bed. Use a gate driver (e.g. a
     74HCT buffer at 5 V) or logic level MOSFETs rated at Vgs = 2.5..3.3 V.
   - DRV8825 logic inputs work with 3.3 V.
*/

/***************************************************************************\
* 1. CPU                                                                    *
\***************************************************************************/

/** \def F_CPU
  STM32F401: 84 MHz, STM32F411: 96 MHz (both give the 48 MHz USB clock,
  USB_CDC doesn't work at 100 MHz).
*/
#if defined STM32F411xE
  #define F_CPU                  96000000UL
#else
  #define F_CPU                  84000000UL
#endif

/** \def HSE_CLOCK_HZ
  Crystal on the board. WeAct Black Pill: 25 MHz.
*/
#define HSE_CLOCK_HZ             25000000UL

/***************************************************************************\
* 2. PINOUTS                                                                *
\***************************************************************************/

//                                       Marlin/RAMPS equivalent
#define X_STEP_PIN               PA_8    // X_STEP  (D54), TIM1_CH1
#define X_DIR_PIN                PA_9    // X_DIR   (D55)
#define X_MIN_PIN                PB_10   // X_MIN   (D3)
//#define X_INVERT_DIR                   // INVERT_X_DIR false
#define X_INVERT_MIN                     // X_MIN_ENDSTOP_INVERTING true

#define Y_STEP_PIN               PA_15   // Y_STEP  (D60), TIM2_CH1
#define Y_DIR_PIN                PA_10   // Y_DIR   (D61)
#define Y_MIN_PIN                PB_3    // Y_MIN   (D14)
//#define Y_INVERT_DIR                   // INVERT_Y_DIR false
#define Y_INVERT_MIN                     // Y_MIN_ENDSTOP_INVERTING true

/** \def Z_STEP_PIN Z_DIR_PIN Z_MIN_PIN
  Z and Z2 (Marlin Z_DUAL_STEPPER_DRIVERS, driver in the E1 slot): STEP
  and DIR of both drivers in parallel on these pins. Z stays on GPIO
  pulses, it steps rarely (layer changes, Z-hop).

  Z_MIN is also the input of a Z probe (BLTouch or inductive sensor, see
  BLTOUCH and INDUCTIVE_PROBE below), which then replaces the Z endstop
  switch. Z_INVERT_MIN is for the switch; with a probe config_wrapper.h
  sets the polarity of the probe instead.
*/
#define Z_STEP_PIN               PB_12   // Z_STEP  (D46) + E1_STEP (D36)
#define Z_DIR_PIN                PB_13   // Z_DIR   (D48) + E1_DIR  (D34)
#define Z_MIN_PIN                PB_15   // Z_MIN   (D18)
//#define Z_INVERT_DIR                   // INVERT_Z_DIR false
#define Z_INVERT_MIN                     // Z_MIN_ENDSTOP_INVERTING true

/** \def Z_STEPPER_ALIGN Z2_STEP_PIN
  Independent Z alignment (G34, like Marlin's Z_STEPPER_AUTO_ALIGN), needs
  a probe (BLTOUCH or INDUCTIVE_PROBE): the STEP input of the Z2 driver moves from PB12 to PC15 (remove
  the 32.768 kHz crystal on PC14/PC15), DIR stays in parallel on PB13. Z
  and Z2 still step together, G34 raises one of them alone until the
  gantry is parallel to the bed. Probe points and lead screw positions:
  Z_STEPPER_ALIGN_* in the printer config. Only needed if Z and Z2 drift
  apart (e.g. turning a lead screw by hand with the motors off).
*/
//#define Z_STEPPER_ALIGN
#ifdef Z_STEPPER_ALIGN
  #define Z2_STEP_PIN            PC_15   // Z2 driver STEP (E1_STEP, D36)
#endif

#define E_STEP_PIN               PB_4    // E0_STEP (D26), TIM3_CH1
#define E_DIR_PIN                PB_5    // E0_DIR  (D28)
#define E_INVERT_DIR                     // INVERT_E0_DIR true

/** \def STEP_TIMER_PULSES
  STEP pulses of X (TIM1), Y (TIM2) and E (TIM3) by timers in one pulse
  mode. Startup reports "echo:Step pulses: X TIM1 Y TIM2 Z gpio E TIM3".
*/
#define STEP_TIMER_PULSES

/** \def STEPPER_ENABLE_PIN
  One enable line for all drivers (connect the EN pins of all five drivers
  together). DRV8825 are enabled low.
*/
#define STEPPER_ENABLE_PIN       PB_14
#define STEPPER_INVERT_ENABLE

/** \def MIN_STEP_PULSE_US
  DRV8825 needs 1.9 us.
*/
#define MIN_STEP_PULSE_US        2

/** \def PS_ON_PIN PS_INVERT_ON PS_AUTO_OFF
  Power supply switch. printer_kill() (thermal errors, M112) switches it
  off: the only way to stop a heater with a shorted MOSFET. Also M80/M81.

  With PS_INVERT_ON the pin is active high and always driven (3.3 V
  push-pull), for:
   - a relay or SSR module in the mains line of a 12 V LED supply,
   - ATX PS_ON (green wire) through an NPN transistor or optocoupler,
     collector to PS_ON, emitter to GND.
  Put a 10 k pull-down at the module / transistor input: the pin floats
  from reset until the firmware runs, the supply must stay off then.

  Without PS_INVERT_ON: ATX PS_ON wired directly, active low, the pin is
  released when off (the supply pulls it to 5 V). Only on a 5 V tolerant
  pin (FT in the datasheet's pin table).

  The Black Pill must not be powered from the switched supply: USB, ATX
  5VSB or a separate 5 V. PS_AUTO_OFF additionally switches the supply off
  after 30 s idle (heaters off, no moves), which also drops the steppers'
  holding torque, M84 S0 or not.

  PB2 is BOOT1, sampled at reset while all pins are inputs. With the
  pull-down above it reads low, so BOOT0 + reset still starts the USB DFU
  bootloader. Don't wire ATX PS_ON directly to PB2: its 5 V pull-up would
  make BOOT0 + reset start from SRAM instead.
*/
#define PS_ON_PIN                PB_2
#define PS_INVERT_ON
//#define PS_AUTO_OFF

/** \def FILAMENT_RUNOUT_PIN FILAMENT_RUNOUT_STATE
  Filament runout sensor: switch or sensor output to PA0, internal pull-up
  on (FILAMENT_RUNOUT_NO_PULLUP switches it off). FILAMENT_RUNOUT_STATE is
  the level when the filament is gone: 0 = the switch closes to GND.

  The board's KEY button is parallel to PA0: pressing it during a print
  acts like a runout, i.e. it's a manual pause (M600). Without a sensor
  connected the pull-up keeps the input high = filament present.

  PA0 is not 5 V tolerant: a 5 V sensor needs a divider. M119 shows the
  state ("filament:"), M412 switches detection off/on.
*/
#define FILAMENT_RUNOUT_PIN      PA_0
#define FILAMENT_RUNOUT_STATE    0

/** \def BLTOUCH BLTOUCH_SERVO_PIN
  BLTouch (or a clone) instead of the Z endstop switch: signal (white) to
  PB15 = Z_MIN_PIN, servo input (orange) to PC13. G28 Z homes with the
  probe then, G29 probes the mesh, G30 one point. Set the probe offsets
  (M851 or Z_PROBE_OFFSET_* in the printer config) before the first use.

  PC13 has no timer channel: TIM9 generates the servo pulses by interrupt.
  The board LED on PC13 flickers with them. PC13 drives 3.3 V, which the
  BLTouch servo input accepts. Supply the BLTouch with 5 V.
*/
//#define BLTOUCH
#define BLTOUCH_SERVO_PIN        PC_13

/** \def INDUCTIVE_PROBE INDUCTIVE_PROBE_ACTIVE_HIGH
  Inductive (or capacitive) proximity sensor instead of the BLTouch, like
  the Prusa PINDA or an LJ12A3-4-Z: no servo, nothing to deploy, it
  switches when it comes near the metal bed (steel sheet or aluminium,
  not glass). Same features: G28 Z homes with it, G29, G30, G34.

  Wiring: signal to PB15 = Z_MIN_PIN (5 V tolerant, internal pull-up with
  USE_INTERNAL_PULLUPS). Take an NPN type (open collector, pulls the
  signal to GND when metal is near): the sensor may be powered with
  6..36 V then, PB15 only sees the pull-up and GND. NPN NO (normally
  open): active low, the default. NPN NC or a PNP sensor through a
  voltage divider / optocoupler to 3.3 V: INDUCTIVE_PROBE_ACTIVE_HIGH.
  Never connect a PNP output powered with 12 V directly to the pin.

  Probe offset Z (M851 Z) of an inductive sensor: minus its switching
  distance above the bed, typically -1..-4 mm. The distance depends on
  the temperature of the sensor and of the bed: measure the offset at
  printing temperatures, probe with the bed hot. Choose BLTOUCH or
  INDUCTIVE_PROBE, not both.
*/
//#define INDUCTIVE_PROBE
//#define INDUCTIVE_PROBE_ACTIVE_HIGH

/*
  Reserved for planned features (the firmware doesn't use them yet):

    spare                 PC_15  Z2_STEP (Z_STEPPER_ALIGN above) or MAX31865
                                 CS (HOTEND_MAX31865 below); PC14/PC15 are
                                 the 32.768 kHz crystal: remove it (the
                                 firmware doesn't use the LSE)
    encoder, beeper       I2C1   PCF8574 I/O expander, P4 / P5 (P0..P3:
                                 buttons, P6 / P7: fans, see below)

  PC13..PC15: low speed (2 MHz), 3 mA, never a current source. Fine for a
  servo signal, a chip select and inputs.

  No DEBUG_LED_PIN / BEEPER_PIN: PC13 is the servo, the beeper goes to the
  expander.
*/

/** \def PCF8574_ADDRESS HOTEND_FAN_EXPANDER_BIT CONTROLLER_FAN_EXPANDER_BIT
  All MCU pins are taken: the hotend fan (on above HOTEND_FAN_TEMP) and
  the controller fan (on while the drivers are enabled or a heater is on)
  go to a PCF8574 I/O expander on I2C1 (PB6/PB7), 7 bit address
  PCF8574_ADDRESS (0x20: A0..A2 to GND), outputs P6 and P7. A PCF8574
  output is only a weak pull-up when high: each fan through a MOSFET
  module with a 10 k pull-up to 5 V at its input ("high = on"). P0..P3
  are the menu buttons (DISPLAY_MENU below), P4 / P5 stay free. Without an expander
  nothing happens (the writes aren't acknowledged), fans on 12 V directly
  run all the time as before. An MCU pin instead: HOTEND_FAN_PIN /
  CONTROLLER_FAN_PIN (active high, *_FAN_INVERT for active low).
*/
#define PCF8574_ADDRESS          0x20
#define HOTEND_FAN_EXPANDER_BIT  6
#define CONTROLLER_FAN_EXPANDER_BIT 7

/** \def DISPLAY_TYPE_SSD1306 DISPLAY_TYPE_HD44780 DISPLAY_I2C_ADDRESS
  Display on I2C1 (PB6/PB7), together with the PCF8574 of the buttons and
  fans. One of:

   - DISPLAY_TYPE_SSD1306: OLED 128x64 (0.96", 21x8 characters), address
     0x3C (0x3D with the address resistor moved). DISPLAY_HEIGHT 32 for
     the 128x32 ones (0.91", 21x4), DISPLAY_SH1106 for the 1.3" modules
     with an SH1106, DISPLAY_ROTATE_180 for one mounted upside down.

   - DISPLAY_TYPE_HD44780: character LCD 20x4 with the usual PCF8574
     backpack ("LCD2004 I2C"), address 0x27 (PCF8574T) or 0x3F
     (PCF8574AT). DISPLAY_COLS / DISPLAY_LINES for 16x2, 20x2 or 16x4.

  Both run on 3.3 V logic: the OLED directly; the LCD needs 5 V supply,
  its backpack pulls SDA/SCL up to 5 V - PB6/PB7 tolerate that (FT pins).
  Without a display nothing changes.
*/
//#define DISPLAY_TYPE_SSD1306
//#define DISPLAY_TYPE_HD44780
#if defined DISPLAY_TYPE_SSD1306 || defined DISPLAY_TYPE_HD44780
  #define DISPLAY_BUS_I2C
#endif
//#define DISPLAY_HEIGHT           32
//#define DISPLAY_SH1106
//#define DISPLAY_I2C_ADDRESS      0x3F
//#define DISPLAY_COLS             16
//#define DISPLAY_LINES            2

/** \def DISPLAY_MENU BUTTON_UP_BIT BUTTON_DOWN_BIT BUTTON_OK_BIT BUTTON_BACK_BIT
  Menu on the display, operated with buttons on the PCF8574 (each from its
  pin to GND): set temperatures, fan, speed, babystep; home, move, level;
  settings (steps/mm, feedrates, accelerations, jerk, linear advance,
  probe offset, retract), stored with "Store settings" (M500, SPI flash);
  print files from the SD card (or SPI flash), pause, resume, stop,
  filament change; resume after a power loss. Back is optional, each menu
  has a "Back" item. Without DISPLAY_MENU the display shows the status
  only.
*/
//#define DISPLAY_MENU
#define BUTTON_UP_BIT            0
#define BUTTON_DOWN_BIT          1
#define BUTTON_OK_BIT            2
#define BUTTON_BACK_BIT          3

/** \def SPI_FLASH
  SPI flash chip (W25Q16..W25Q128) soldered to the footprint on the bottom
  of the Black Pill: SPI1, CS PA4, SCK PA5, MISO PA6, MOSI PA7. Holds the
  settings (M500), the power loss records and G-code files (M28 upload,
  M20/M23/M24 print; not with an SD card, which takes these commands then).
  Without a chip the firmware falls back to internal Flash for the
  settings.
  PA4..PA7 are taken then, the thermistors are on PA1/PB1 for that reason.
*/
#define SPI_FLASH
//#define SPI_FLASH_CS_PIN         PA_4

/*
  SPI bus: SPI1 on PA5..PA7, shared by SPI flash and an SD card (if any).
  I2C1 on PB6/PB7: display and I/O expander.
*/
#define SPI_INSTANCE             1
#define SPI_SCK_PIN              PA_5
#define SPI_MISO_PIN             PA_6
#define SPI_MOSI_PIN             PA_7
#define I2C_INSTANCE             1
#define I2C_SCL_PIN              PB_6
#define I2C_SDA_PIN              PB_7
#define I2C_SPEED                400000UL

/** \def SD_CARD_SELECT_PIN
  SD card in SPI mode on the bus of the SPI flash: SCK PA5, MISO PA6 (card
  DO), MOSI PA7 (card DI), chip select PC14 (remove the 32.768 kHz
  crystal). The card and its module run on 3.3 V: a module without level
  shifter, or one whose MISO driver is switched off by CS. Cheap modules
  with a 74LVC125 often drive MISO all the time: then the SPI flash (and a
  MAX31865) don't work anymore.

  M20..M27 use the card then (Petit FatFs: FAT16 / FAT32, 8.3 names, read
  only, so no M28 upload), the SPI flash keeps the settings and the power
  loss records. The card is mounted at startup, M21 after changing it.
*/
//#define SD_CARD_SELECT_PIN       PC_14

/***************************************************************************\
* 3. TEMPERATURE SENSORS                                                    *
\***************************************************************************/

#ifndef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(...)
#endif

#define TEMP_THERMISTOR

/** \def DEFINE_THERMISTOR
  Thermistors, Steinhart-Hart equation. Parameters: name, pull-up resistor
  (Ohm, to 3.3 V), three calibration points temperature (C) / resistance
  (Ohm). EPCOS B57560G104F 100k = Marlin sensor type 1, the points Marlin
  uses to generate its table.
*/
//                name       pullup  t1    r1        t2     r2       t3     r3
DEFINE_THERMISTOR(epcos100k, 4700.0, 25.0, 100000.0, 150.0, 1641.9,  250.0, 226.15)

/** \def HOTEND_MAX31865 MAX31865_CS_PIN MAX31865_WIRES MAX31865_RREF MAX31865_R0 MAX31865_50HZ
  Hotend with a PT100 (or PT1000) through a MAX31865 board instead of the
  thermistor: for 300+ C (all-metal hotends, PEEK, PC, nylon), where a
  100k thermistor gets inaccurate, and exact without calibration.

  The MAX31865 is on SPI1 (PA5 SCK, PA6 MISO -> SDO, PA7 MOSI -> SDI),
  shared with the SPI flash, chip select MAX31865_CS_PIN. All pins are
  taken: PC15 (remove the 32.768 kHz crystal; not together with
  Z_STEPPER_ALIGN, which takes PC15, then use PC14 without SD card).
  Supply the board with 3.3 V.

  MAX31865_WIRES: 2, 3 or 4 wire sensor, set the jumpers / solder bridges
  of the board to match. MAX31865_RREF: reference resistor of the board,
  Ohm (Adafruit, most clones: 430 for PT100, 4300 for PT1000).
  MAX31865_R0: 100 = PT100, 1000 = PT1000. MAX31865_50HZ: mains filter
  50 Hz (else 60 Hz). An open or shorted sensor is a fault: no reading,
  the thermal protection halts the printer.
*/
//#define HOTEND_MAX31865
#define MAX31865_CS_PIN          PC_15
#define MAX31865_WIRES           2
#define MAX31865_RREF            430.0
#define MAX31865_R0              100.0
#define MAX31865_50HZ

#ifdef HOTEND_MAX31865
  #define TEMP_MAX31865
#endif

//                 name      type           pin    additional
#ifdef HOTEND_MAX31865
DEFINE_TEMP_SENSOR(extruder, TT_MAX31865,   MAX31865_CS_PIN, MAX31865_WIRES)  // PT100
#else
DEFINE_TEMP_SENSOR(extruder, TT_THERMISTOR, PA_1,  THERMISTOR_epcos100k)  // T0 (A13)
#endif
DEFINE_TEMP_SENSOR(bed,      TT_THERMISTOR, PB_1,  THERMISTOR_epcos100k)  // T1 (A14)

/***************************************************************************\
* 4. HEATERS                                                                *
\***************************************************************************/

/** \def DEFINE_HEATER
  RAMPS EFB: Extruder (D10), Fan (D9), Bed (D8).

  TIM1..TIM3 drive the steps (STEP_TIMER_PULSES), TIM5 is the step timer.
  So hotend and fan use TIM10 / TIM11 on the same pins PB8 / PB9 (the
  PB_8_TIM10 / PB_9_TIM11 names in hal/pins_stm32f4.h), each with its own
  frequency.

  Hotend: 100 Hz hardware PWM, low frequency keeps MOSFET switching losses
  low. Bed: PB0 has TIM3_CH3 only, but TIM3 drives the E pulses. So it gets
  slow software PWM (PB_0_GPIO = PB0 without its timer), 'pwm' 2 = 2 Hz
  (2..10 possible): 500 ms period, duty
  in 10 ms steps, 4 switching events per second at most. For bang-bang
  (BANG_BANG_BED in the printer config) this acts like on/off. Fan: 500 Hz.
*/
//            name      pin         invert  pwm      max_pwm
DEFINE_HEATER(extruder, PB_8_TIM10, 0,      100,     100)   // TIM10_CH1
DEFINE_HEATER(bed,      PB_0_GPIO,  0,      2,       100)   // slow soft PWM
DEFINE_HEATER(fan,      PB_9_TIM11, 0,      500,     100)   // TIM11_CH1

#define HEATER_EXTRUDER HEATER_extruder
#define HEATER_BED      HEATER_bed
#define HEATER_FAN      HEATER_fan

/***************************************************************************\
* 5. COMMUNICATION OPTIONS                                                  *
\***************************************************************************/

/** \def USB_CDC
  Virtual COM port over the Black Pill's USB-C socket (PA11/PA12), main
  host connection. The UART below stays active in addition, both can be
  used at the same time: answers go to the port the command came from.
  See hal/usb_cdc.h for USB_VID, USB_PID, USB_PRODUCT.
*/
#define USB_CDC

/** \def BAUD SERIAL_UART
  Additional host connection over a USB-UART adapter on PA2 (TX) / PA3
  (RX).
*/
#define BAUD                     115200
#define SERIAL_UART              2
#define SERIAL_TX_PIN            PA_2
#define SERIAL_RX_PIN            PA_3
//#define XONXOFF

/***************************************************************************\
* 6. ANALOG                                                                 *
\***************************************************************************/

/** \def OVERSAMPLE
  ADC samples averaged per reading (min and max are discarded).
*/
#define OVERSAMPLE               16
