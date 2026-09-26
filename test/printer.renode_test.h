/** \file
  \brief Printer configuration for the Renode tests (short protection periods).
*/

/***************************************************************************\
* 1. MECHANICS                                                              *
\***************************************************************************/

/** \def KINEMATICS_STRAIGHT KINEMATICS_COREXY
  Kinematics type of the printer.
*/
#define KINEMATICS_STRAIGHT
//#define KINEMATICS_COREXY

/** \def STEPS_PER_M_X STEPS_PER_M_Y STEPS_PER_M_Z STEPS_PER_M_E
  Steps per meter ( = steps per mm * 1000 ), integers only.
    Valid range: 20 to 4'0960'000 (0.02 to 40960 steps/mm)
*/
#define STEPS_PER_M_X            40000
#define STEPS_PER_M_Y            40000
#define STEPS_PER_M_Z            320000
#define STEPS_PER_M_E            96271

/** \def MAXIMUM_FEEDRATE_X MAXIMUM_FEEDRATE_Y MAXIMUM_FEEDRATE_Z MAXIMUM_FEEDRATE_E
  Used for G0 rapid moves and as a cap for all other feedrates. mm/min.
*/
#define MAXIMUM_FEEDRATE_X       6000
#define MAXIMUM_FEEDRATE_Y       6000
#define MAXIMUM_FEEDRATE_Z       200
#define MAXIMUM_FEEDRATE_E       6000

/** \def SEARCH_FEEDRATE_X SEARCH_FEEDRATE_Y SEARCH_FEEDRATE_Z
  Used when doing precision endstop search and as default feedrate. mm/min.
*/
#define SEARCH_FEEDRATE_X        200
#define SEARCH_FEEDRATE_Y        200
#define SEARCH_FEEDRATE_Z        50

/** \def ENDSTOP_CLEARANCE_X ENDSTOP_CLEARANCE_Y ENDSTOP_CLEARANCE_Z
  How many micrometers the carriage may overshoot the endstop trigger point.
  Homing speed is derived from this and ACCELERATION.
*/
#define ENDSTOP_CLEARANCE_X      1000
#define ENDSTOP_CLEARANCE_Y      1000
#define ENDSTOP_CLEARANCE_Z      100

/** \def X_MIN X_MAX Y_MIN Y_MAX Z_MIN Z_MAX
  Soft axis limits in millimeters. Not defining them disables the check.
*/
//#define X_MIN                    0.0
//#define X_MAX                    200.0
//#define Y_MIN                    0.0
//#define Y_MAX                    200.0
//#define Z_MIN                    0.0
//#define Z_MAX                    140.0

/** \def E_ABSOLUTE
  Startup default for extruder coordinates (changeable with M82/M83).
*/
#define E_ABSOLUTE

/** \def DEFINE_HOMING
  Order (and number) of homing movements, up to 4. Options: none,
  x_negative, x_positive, y_negative, y_positive, z_negative, z_positive.
*/
DEFINE_HOMING(x_negative, y_negative, z_negative)

/** \def ACCELERATION_REPRAP ACCELERATION_RAMPING ACCELERATION_TEMPORAL
  Acceleration type, recommended is ACCELERATION_RAMPING.
*/
//#define ACCELERATION_REPRAP
#define ACCELERATION_RAMPING
//#define ACCELERATION_TEMPORAL

/** \def ACCELERATION
  Acceleration for ACCELERATION_RAMPING and homing, mm/s^2.
*/
#define ACCELERATION             1000

/** \def MAX_ACCELERATION_X MAX_ACCELERATION_Y MAX_ACCELERATION_Z MAX_ACCELERATION_E
  Optional max. acceleration per axis (M201), mm/s^2. Default: ACCELERATION.
*/
//#define MAX_ACCELERATION_Z       100

/** \def LOOKAHEAD
  Join moves without stopping. Works with ACCELERATION_RAMPING only.
*/
#define LOOKAHEAD

/** \def MAX_JERK_X MAX_JERK_Y MAX_JERK_Z MAX_JERK_E
  Allowed speed jump at movement crossings for lookahead, mm/min.
*/
#define MAX_JERK_X               200
#define MAX_JERK_Y               200
#define MAX_JERK_Z               20
#define MAX_JERK_E               200

/** \def BED_LEVELING
  Dynamic 3-point bed leveling with G29.
*/
//#define BED_LEVELING

/***************************************************************************\
* 2. SAFETY                                                                 *
\***************************************************************************/

/** \def THERMAL_PROTECTION_PERIOD THERMAL_PROTECTION_HYSTERESIS
  Thermal runaway: once the target was reached, the temperature must not
  stay more than HYSTERESIS degrees below the target for longer than PERIOD
  seconds. Otherwise the printer halts ("Thermal Runaway").
*/
#define THERMAL_PROTECTION_PERIOD         10
#define THERMAL_PROTECTION_HYSTERESIS     8

/** \def WATCH_TEMP_PERIOD WATCH_TEMP_INCREASE
  Heating: the temperature has to rise by INCREASE degrees within PERIOD
  seconds, over and over, until the target is reached ("Heating failed").
*/
#define WATCH_TEMP_PERIOD                 10
#define WATCH_TEMP_INCREASE               2

/** \def THERMAL_PROTECTION_BED_PERIOD THERMAL_PROTECTION_BED_HYSTERESIS
    \def WATCH_BED_TEMP_PERIOD WATCH_BED_TEMP_INCREASE
  Same for the bed (the sensor named 'bed', with HEATER_BED defined).
*/
#define THERMAL_PROTECTION_BED_PERIOD     20
#define THERMAL_PROTECTION_BED_HYSTERESIS 2
#define WATCH_BED_TEMP_PERIOD             60
#define WATCH_BED_TEMP_INCREASE           2

/** \def HEATER_MINTEMP HEATER_MAXTEMP BED_MINTEMP BED_MAXTEMP
  Readings outside this range halt the printer. MAXTEMP is checked always,
  MINTEMP only while the heater is on (catches broken thermistors).
  Degree Celsius.
*/
#define HEATER_MINTEMP                    5
#define HEATER_MAXTEMP                    275
#define BED_MINTEMP                       5
#define BED_MAXTEMP                       125

/** \def HOTEND_OVERSHOOT BED_OVERSHOOT
  Targets are limited to MAXTEMP minus this, degree Celsius.
*/
#define HOTEND_OVERSHOOT                  15
#define BED_OVERSHOOT                     10

/** \def THERMAL_PROTECTION_OFF_PERIOD THERMAL_PROTECTION_OFF_RISE
    \def THERMAL_PROTECTION_BED_OFF_PERIOD THERMAL_PROTECTION_BED_OFF_RISE
  Heater stuck on (shorted MOSFET, welded relay): while the heater output
  is off, the temperature must not rise by more than RISE degrees within
  PERIOD seconds. The first period after switching off isn't judged
  (overshoot). A heater at full power rises much faster; a nozzle parked on
  a hot bed much slower. RISE 0 disables the check.
*/
#define THERMAL_PROTECTION_OFF_PERIOD     20
#define THERMAL_PROTECTION_OFF_RISE       20
#define THERMAL_PROTECTION_BED_OFF_PERIOD 60
#define THERMAL_PROTECTION_BED_OFF_RISE   10

/** \def TEMP_SENSOR_TIMEOUT
  Seconds without a valid reading while heating before the printer halts.
*/
#define TEMP_SENSOR_TIMEOUT               4

/** \def FLASH_STORE_SIZE
  Test: use only 1 kB of the settings sector, so a few M500 fill it up and
  the erase path gets exercised.
*/
#define FLASH_STORE_SIZE                  1024

/***************************************************************************\
* 3. HOST COMMUNICATION                                                     *
\***************************************************************************/

/** \def ADVANCED_OK
  Acknowledge with "ok N<line> P<free moves> B<free command slots>" instead
  of a plain "ok". All common hosts accept it, some use it to stream ahead.
*/
#define ADVANCED_OK

/** \def CMD_BUFSIZE
  Number of G-code lines buffered ahead of execution.
*/
#define CMD_BUFSIZE                       8

/** \def HOST_KEEPALIVE_INTERVAL
  Seconds between "busy: processing" messages while a command takes long,
  e.g. homing or waiting for temperatures. 0 = off. Changeable with M113.
*/
#define HOST_KEEPALIVE_INTERVAL           2

/***************************************************************************\
* 4. MISCELLANEOUS                                                          *
\***************************************************************************/

/** \def USE_INTERNAL_PULLUPS USE_INTERNAL_PULLDOWNS
  Internal pull resistors for the endstops (only while homing or M119).
*/
//#define USE_INTERNAL_PULLUPS
//#define USE_INTERNAL_PULLDOWNS

/** \def STEPPER_IDLE_TIMEOUT
  Disable steppers after this many seconds without movement (and without
  waiting for temperatures). 0 = never. Changeable with M84 S<seconds>.
*/
#define STEPPER_IDLE_TIMEOUT     120

/** \def Z_AUTODISABLE
  Disable Z stepper when not in use. No effect with a common enable pin.
*/
#define Z_AUTODISABLE

/** \def TEMP_HYSTERESIS
  Degree Celsius around target temperature regarded as 'achieved'.
*/
#define TEMP_HYSTERESIS          2

/** \def TEMP_RESIDENCY_TIME
  Seconds the temperature has to stay within hysteresis for M116.
*/
#define TEMP_RESIDENCY_TIME      2

/** \def TEMP_EWMA
  Smoothing of noisy sensors, 1..1000, 1000 = off.
*/
#define TEMP_EWMA                1000

/** \def REPORT_TARGET_TEMPS
  M105 reports T:current/target.
*/
#define REPORT_TARGET_TEMPS

/** \def HEATER_SANITY_CHECK
  Old Teacup check, superseded by the thermal protection above (Heating
  failed, Thermal Runaway, heater stuck on): it only lowers the output and
  prints a message on every tick, it doesn't stop the printer. Keep it off.
*/
//#define HEATER_SANITY_CHECK

/** \def BANG_BANG BANG_BANG_ON BANG_BANG_OFF
  Replace PID by simple on/off control with these PWM values.
*/
//#define BANG_BANG
//#define BANG_BANG_ON             200
//#define BANG_BANG_OFF            45

/** \def MOVEBUFFER_SIZE
  Move buffer size, in number of moves.
*/
#define MOVEBUFFER_SIZE          8

/** \def DC_EXTRUDER DC_EXTRUDER_PWM
  DC motor extruder, configured as a heater above.
*/
//#define DC_EXTRUDER              HEATER_motor
//#define DC_EXTRUDER_PWM          180

/** \def USE_WATCHDOG
  Independent watchdog, 500 ms timeout. A hanging firmware resets the
  controller, which switches all heater outputs off. Keep it enabled.
*/
#define USE_WATCHDOG

/** \def TH_COUNT
  Temperature history count for the PID derivative. Power of 2.
*/
#define TH_COUNT                 8

/** \def PID_SCALE
  Scaling of internally stored PID values.
*/
#define PID_SCALE                1024L

/** \def ENDSTOP_STEPS
  Number of consecutive positive readings (every 2 ms) to accept an endstop.
*/
#define ENDSTOP_STEPS            4

/** \def CANNED_CYCLE
  G-code executed over and over again, e.g. for exhibitions.
*/
/*
#define CANNED_CYCLE "G1 X100 F3000\n" \
"G4 P500\n" \
"G1 X0\n" \
"G4 P500\n"
*/
