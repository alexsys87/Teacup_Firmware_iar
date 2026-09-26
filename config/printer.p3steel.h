/** \file
  \brief Printer configuration: Prusa i3 Steel ("P3 Steel").

  Values taken over from the printer's Marlin configuration, converted to
  Teacup units (steps per meter, mm/min). Where Marlin values were unusable
  (max. feedrate 500 mm/s, printing acceleration 50 mm/s^2, thermal
  protection off) this file says so.
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
#define STEPS_PER_M_X            160000   // 160 steps/mm: GT2, 20T, 1/32
#define STEPS_PER_M_Y            160000   // 160 steps/mm
#define STEPS_PER_M_Z            8000000  // 8000 steps/mm: M5 (0.8 mm), 1/32
#define STEPS_PER_M_E            1672000  // 1672 steps/mm: Greg's Wade, 1/32

/** \def MAXIMUM_FEEDRATE_X MAXIMUM_FEEDRATE_Y MAXIMUM_FEEDRATE_Z MAXIMUM_FEEDRATE_E
  Used for G0 rapid moves and as a cap for all other feedrates. mm/min.
  Marlin had 500/500/50/25 mm/s, far beyond what the mechanics (and the
  step rate: X/Y 160 steps/mm, Z 8000 steps/mm) allow. Limited to
  150 mm/s X/Y (24k steps/s), 4 mm/s Z (32k steps/s, Marlin's Z homing
  speed), 25 mm/s E (42k steps/s).
*/
#define MAXIMUM_FEEDRATE_X       9000
#define MAXIMUM_FEEDRATE_Y       9000
#define MAXIMUM_FEEDRATE_Z       240
#define MAXIMUM_FEEDRATE_E       1500

/** \def SEARCH_FEEDRATE_X SEARCH_FEEDRATE_Y SEARCH_FEEDRATE_Z
  Used when doing precision endstop search and as default feedrate. mm/min.
*/
#define SEARCH_FEEDRATE_X        600
#define SEARCH_FEEDRATE_Y        600
#define SEARCH_FEEDRATE_Z        120

/** \def ENDSTOP_CLEARANCE_X ENDSTOP_CLEARANCE_Y ENDSTOP_CLEARANCE_Z
  How many micrometers the carriage may overshoot the endstop trigger point.
  Homing speed is derived from this and ACCELERATION:
  v = sqrt(2 * ACCELERATION * clearance). 1.25 mm gives 50 mm/s for X/Y
  (Marlin HOMING_FEEDRATE 50 mm/s); Z is capped by MAXIMUM_FEEDRATE_Z.
*/
#define ENDSTOP_CLEARANCE_X      1250
#define ENDSTOP_CLEARANCE_Y      1250
#define ENDSTOP_CLEARANCE_Z      100

/** \def X_MIN X_MAX Y_MIN Y_MAX Z_MIN Z_MAX
  Soft axis limits in millimeters. Not defining them disables the check.
*/
#define X_MIN                    0.0
#define X_MAX                    220.0
#define Y_MIN                    0.0
#define Y_MAX                    180.0
#define Z_MIN                    0.0
#define Z_MAX                    200.0

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

/** \def ACCELERATION ACCELERATION_RETRACT ACCELERATION_TRAVEL
  Acceleration along the path for ACCELERATION_RAMPING, mm/s^2, like
  Marlin's M204: printing moves (with E, M204 P), retracts and primes
  (E only, M204 R), travel and homing (no E, M204 T).
  Marlin: max. 1000 X/Y, 50 Z, 10000 E; printing acceleration 50 (far too
  low). The per axis limits follow below (M201).
*/
#define ACCELERATION             1000
#define ACCELERATION_RETRACT     5000
#define ACCELERATION_TRAVEL      1000

/** \def MAX_ACCELERATION_X MAX_ACCELERATION_Y MAX_ACCELERATION_Z MAX_ACCELERATION_E
  Max. acceleration per axis (Marlin MAX_ACCELERATION), mm/s^2. Each axis
  gets its share of the acceleration along the path; where this exceeds
  its limit, the move accelerates slower, e.g. Z-only moves with
  50 mm/s^2. Changeable with M201.
*/
#define MAX_ACCELERATION_X       1000
#define MAX_ACCELERATION_Y       1000
#define MAX_ACCELERATION_Z       50
#define MAX_ACCELERATION_E       10000

/** \def LOOKAHEAD
  Join moves without stopping. Works with ACCELERATION_RAMPING only.
*/
#define LOOKAHEAD

/** \def MAX_JERK_X MAX_JERK_Y MAX_JERK_Z MAX_JERK_E
  Allowed speed jump at movement crossings for lookahead, mm/min.
*/
#define MAX_JERK_X               1200     // 20 mm/s
#define MAX_JERK_Y               1200     // 20 mm/s
#define MAX_JERK_Z               24       // 0.4 mm/s
#define MAX_JERK_E               300      // 5 mm/s

/** \def LINEAR_ADVANCE LINEAR_ADVANCE_K
  Linear advance (M900 K, Marlin units): the extruder runs ahead of the
  nominal position by K * extrusion speed, K in seconds (mm of filament per
  mm/s of filament speed). Evens out extrusion at corners and the seam.
  Direct drive: K about 0.02..0.1, Bowden: 0.2..1. LINEAR_ADVANCE_K is the
  default, 0 = off until M900. Needs ACCELERATION_RAMPING and LOOKAHEAD.
*/
#define LINEAR_ADVANCE
#define LINEAR_ADVANCE_K         0.0

/** \def BED_LEVELING GRID_POINTS_X GRID_POINTS_Y MESH_INSET LEVELING_FADE_HEIGHT
  Mesh bed leveling: bilinear grid of GRID_POINTS_X x GRID_POINTS_Y points
  (2..7), MESH_INSET mm away from the bed edges (X_MIN..X_MAX,
  Y_MIN..Y_MAX). G29 probes it (BLTOUCH in the board file), M421 sets
  points by hand. M420 S1/S0 on/off, M420 Z fade height: the correction
  fades out linearly up to this height (mm, 0 = never). M500 stores it.
  Moves are split at the grid lines, so Z follows the surface.
*/
#define BED_LEVELING
#define GRID_POINTS_X            3
#define GRID_POINTS_Y            3
#define MESH_INSET               15.0
#define LEVELING_FADE_HEIGHT     10.0

/** \def Z_PROBE_OFFSET_X Z_PROBE_OFFSET_Y Z_PROBE_OFFSET_Z
  BLTouch position relative to the nozzle, mm (Marlin
  NOZZLE_TO_PROBE_OFFSET). Z is negative: minus the nozzle height above the
  bed when the probe triggers. Measure Z: M851 Z-5, G28, lower the nozzle
  with G1 Z... until a sheet of paper just drags, read Z (M114), e.g. 3.40:
  offset = 3.40 - 5 = -1.60, so M851 Z-1.60, M500. Only used with BLTOUCH
  (board file). The fine tuning while printing is M290 (Z offset).
*/
#define Z_PROBE_OFFSET_X         0.0
#define Z_PROBE_OFFSET_Y         0.0
#define Z_PROBE_OFFSET_Z         0.0

/** \def Z_PROBE_FEEDRATE_FAST Z_PROBE_FEEDRATE_SLOW Z_PROBE_SAMPLES
  Probing: first fast, then (Z_PROBE_SAMPLES 2) up Z_PROBE_RETRACT mm and
  slowly again; the slow one counts. mm/min. Between points the nozzle
  travels at Z_PROBE_CLEARANCE mm. Z_PROBE_LOW_POINT: how far below Z = 0
  the probe may go before it's an error.
*/
#define Z_PROBE_FEEDRATE_FAST    240
#define Z_PROBE_FEEDRATE_SLOW    60
#define Z_PROBE_SAMPLES          2
#define Z_PROBE_RETRACT          3.0
#define Z_PROBE_CLEARANCE        5.0
#define Z_PROBE_LOW_POINT        -2.0

/** \def BABYSTEPPING BABYSTEP_FEEDRATE BABYSTEP_LIMIT
  M290 Z<mm>: move the nozzle up/down right away, also while printing, to
  adjust the first layer. The sum is a Z offset, stored with M500 (like
  Prusa's "Live adjust Z"). Speed mm/min, limit +-mm.
*/
#define BABYSTEPPING
#define BABYSTEP_FEEDRATE        60
#define BABYSTEP_LIMIT           2.0

/** \def FILAMENT_RUNOUT_DISTANCE
  Filament runout (sensor in the board file): the print pauses after this
  much filament (mm) was fed since the sensor triggered, the filament
  between sensor and nozzle is used up. M412 D changes it, M412 S0 turns
  detection off.
*/
#define FILAMENT_RUNOUT_DISTANCE 25.0

/** \def FILAMENT_CHANGE_RETRACT FILAMENT_CHANGE_Z_LIFT FILAMENT_CHANGE_PARK_X FILAMENT_CHANGE_PARK_Y
  Filament change (M600, runout), mm: retract, Z lift (relative), park
  position (bed in front, nozzle at the left). Direct drive (Greg's Wade):
  unload 50 mm, load and purge 40 mm slowly.
*/
#define FILAMENT_CHANGE_RETRACT          2.0
#define FILAMENT_CHANGE_Z_LIFT           20.0
#define FILAMENT_CHANGE_PARK_X           10.0
#define FILAMENT_CHANGE_PARK_Y           170.0
#define FILAMENT_CHANGE_UNLOAD           50.0
#define FILAMENT_CHANGE_LOAD             40.0
#define FILAMENT_CHANGE_UNLOAD_FEEDRATE  1200     // 20 mm/s
#define FILAMENT_CHANGE_LOAD_FEEDRATE    180      // 3 mm/s
#define FILAMENT_CHANGE_NOZZLE_TIMEOUT   300      // s, hotend off while waiting

/***************************************************************************\
* 2. SAFETY                                                                 *
\***************************************************************************/

/*
  Marlin had thermal runaway protection switched off. Here it's on with
  Marlin's default values - keep it that way.
*/

/** \def THERMAL_PROTECTION_PERIOD THERMAL_PROTECTION_HYSTERESIS
  Thermal runaway: once the target was reached, the temperature must not
  stay more than HYSTERESIS degrees below the target for longer than PERIOD
  seconds. Otherwise the printer halts ("Thermal Runaway").
*/
#define THERMAL_PROTECTION_PERIOD         40
#define THERMAL_PROTECTION_HYSTERESIS     4

/** \def WATCH_TEMP_PERIOD WATCH_TEMP_INCREASE
  Heating: the temperature has to rise by INCREASE degrees within PERIOD
  seconds, over and over, until the target is reached ("Heating failed").
*/
#define WATCH_TEMP_PERIOD                 20
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
#define BED_MAXTEMP                       150

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
#define USE_INTERNAL_PULLUPS     // Marlin ENDSTOPPULLUPS
//#define USE_INTERNAL_PULLDOWNS

/** \def STEPPER_IDLE_TIMEOUT
  Disable steppers after this many seconds without movement (and without
  waiting for temperatures). 0 = never. Changeable with M84 S<seconds>.
*/
#define STEPPER_IDLE_TIMEOUT     120

/** \def Z_AUTODISABLE
  Disable Z stepper when not in use. No effect with a common enable pin.
*/
//#define Z_AUTODISABLE

/** \def TEMP_HYSTERESIS
  Degree Celsius around target temperature regarded as 'achieved'.
*/
#define TEMP_HYSTERESIS          3

/** \def TEMP_RESIDENCY_TIME
  Seconds the temperature has to stay within hysteresis for M116.
*/
#define TEMP_RESIDENCY_TIME      10

/** \def TEMP_EWMA
  Smoothing of noisy sensors, 1..1000, 1000 = off.
*/
#define TEMP_EWMA                1000

/** \def REPORT_TARGET_TEMPS
  M105 reports T:current/target.
*/
#define REPORT_TARGET_TEMPS

/** \def ARC_TOLERANCE ARC_SEGMENT_MIN ARC_SEGMENT_MAX NO_ARC_SUPPORT
  G2/G3 arcs are split into segments deviating at most ARC_TOLERANCE mm
  from the arc, each ARC_SEGMENT_MIN..ARC_SEGMENT_MAX mm long.
  NO_ARC_SUPPORT removes G2/G3 (saves ~6 KB flash).
*/
#define ARC_TOLERANCE            0.01
#define ARC_SEGMENT_MIN          0.1
#define ARC_SEGMENT_MAX          1.0
//#define NO_ARC_SUPPORT

/** \def HEATER_SANITY_CHECK
  Old Teacup check, superseded by the thermal protection above (Heating
  failed, Thermal Runaway, heater stuck on): it only lowers the output and
  prints a message on every tick, it doesn't stop the printer. Keep it off.
*/
//#define HEATER_SANITY_CHECK

/** \def BANG_BANG BANG_BANG_ON BANG_BANG_OFF
  Replace PID by simple on/off control with these PWM values, all heaters.
*/
//#define BANG_BANG
//#define BANG_BANG_ON             200
//#define BANG_BANG_OFF            45

/** \def BANG_BANG_BED BANG_BANG_BED_HYSTERESIS BANG_BANG_BED_ON
  Bed only on/off (Marlin without PIDTEMPBED, MAX_BED_POWER 255): on below
  target - hysteresis, off above target + hysteresis. Degree Celsius.
*/
#define BANG_BANG_BED
#define BANG_BANG_BED_HYSTERESIS 2
#define BANG_BANG_BED_ON         255

/** \def DEFAULT_P DEFAULT_I DEFAULT_D DEFAULT_I_LIMIT
  Hotend PID from Marlin: Kp 22.2, Ki 1.08, Kd 114. Conversion to Teacup
  units (PID runs every 250 ms on quarter degrees, PID_SCALE 1024):
    P = Kp * 256, I = Ki * 64, D = Kd * 128.
  I_LIMIT allows the integral term the full output range (255 * 1024 / I).
  Runtime tuning (heater index in P): M130 S<Kp/4>, M131 S<Ki/16>,
  M132 S<Kd/8>, M133 S<I limit>.
  Marlin's PID_ADD_EXTRUSION_RATE has no equivalent.
*/
#define DEFAULT_P                ((int32_t)(22.2 * 256))   // 5683
#define DEFAULT_I                ((int32_t)(1.08 * 64))    // 69
#define DEFAULT_D                ((int32_t)(114.0 * 128))  // 14592
#define DEFAULT_I_LIMIT          (255L * 1024 / DEFAULT_I) // 3784

/** \def MOVEBUFFER_SIZE
  Move buffer size, in number of moves. Look-ahead plans over the whole
  queue: to reach speed v, it needs v^2 / (2 * ACCELERATION) of moves ahead
  to stop. 64 moves of 0.35 mm are 22 mm, enough for 200 mm/s at
  1000 mm/s^2. About 120 bytes of RAM per move.
*/
#define MOVEBUFFER_SIZE          64

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
