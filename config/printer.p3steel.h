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

/** \def INPUT_SHAPING INPUT_SHAPING_BUFFER INPUT_SHAPING_FREQ_X INPUT_SHAPING_FREQ_Y INPUT_SHAPING_DAMPING_X INPUT_SHAPING_DAMPING_Y INPUT_SHAPING_TYPE_X INPUT_SHAPING_TYPE_Y S_CURVE_TIME
  Input shaping of X and Y (M593): cancels ringing at the resonance
  frequency of the axis, the i3's Y with the heavy bed rings the most.
  Frequency in Hz, 0 = off (find it with a ringing tower: speed / distance
  of the ripples), damping ratio (0.1 is typical), type 0 = ZV (shortest
  delay), 1 = MZV (tolerates a wrong frequency better).
  S_CURVE_TIME: S-curve smoothing of X and Y in ms, 0 = off: the
  acceleration ramps up and down within this time.
  INPUT_SHAPING_BUFFER: steps per axis kept for the delayed copies, a power
  of 2, 4 bytes each. Enough for (largest delay + S_CURVE_TIME) at top
  speed: MZV at 25 Hz is 30 ms delay, at 150 mm/s (24000 steps/s) with
  10 ms S-curve 960 steps. More than that isn't shaped (M593 reports it).
*/
#define INPUT_SHAPING
#define INPUT_SHAPING_BUFFER     2048
#define INPUT_SHAPING_FREQ_X     0.0
#define INPUT_SHAPING_FREQ_Y     0.0
#define INPUT_SHAPING_DAMPING_X  0.1
#define INPUT_SHAPING_DAMPING_Y  0.1
#define INPUT_SHAPING_TYPE_X     1
#define INPUT_SHAPING_TYPE_Y     1
#define S_CURVE_TIME             0

/** \def SKEW_CORRECTION XY_SKEW_FACTOR
  XY skew correction (M852 I, like Marlin): a frame that isn't exactly
  square prints squares as rhombs. XY_SKEW_FACTOR is the tangent of the
  error angle, motor X = X - Y * factor; 0 = off. Print a square, measure
  its diagonals AC (front left to back right) and BD (front right to back
  left): factor = (AC^2 - BD^2) / (AC^2 + BD^2). A few multiplications
  per move.
*/
#define SKEW_CORRECTION
#define XY_SKEW_FACTOR           0.0

/** \def BACKLASH_COMPENSATION BACKLASH_Z
  Z backlash compensation (M425 Z F, like Marlin): M5 threaded rods with
  plain nuts have 0.05..0.2 mm play. When Z reverses, that many extra
  steps take it up. BACKLASH_Z in mm, 0 = off.
*/
#define BACKLASH_COMPENSATION
#define BACKLASH_Z               0.0

/** \def FIRMWARE_RETRACT RETRACT_LENGTH RETRACT_FEEDRATE RETRACT_ZLIFT RETRACT_RECOVER_LENGTH RETRACT_RECOVER_FEEDRATE
  Firmware retract (G10 / G11, like Marlin's FWRETRACT): the slicer sends
  G10 / G11 instead of E moves ("Use firmware retraction" in PrusaSlicer),
  M207 / M208 change length and speeds while printing. Length and lift in
  mm, speeds in mm/s. Direct drive: 0.8..2 mm.
*/
#define FIRMWARE_RETRACT
#define RETRACT_LENGTH           1.0
#define RETRACT_FEEDRATE         25.0
#define RETRACT_ZLIFT            0.0
#define RETRACT_RECOVER_LENGTH   0.0
#define RETRACT_RECOVER_FEEDRATE 25.0

/** \def HOST_WATCH HOST_TIMEOUT HOST_LOST_HOTEND_TEMP HOST_LOST_RETRACT HOST_LOST_Z_LIFT
  Reaction to a lost host (M86): printing from the host, when no line came
  for HOST_TIMEOUT seconds while the printer waited for one, or the USB
  port was closed / unplugged: retract HOST_LOST_RETRACT mm, lift Z by
  HOST_LOST_Z_LIFT mm, park at FILAMENT_CHANGE_PARK_X/Y, hotend to
  HOST_LOST_HOTEND_TEMP (0 = off). The bed keeps its temperature. Pausing
  in the host longer than HOST_TIMEOUT triggers it, too. HOST_TIMEOUT 0:
  USB unplugged only. Doesn't replace the thermal protection.
*/
#define HOST_WATCH
#define HOST_TIMEOUT             300
#define HOST_LOST_HOTEND_TEMP    0
#define HOST_LOST_RETRACT        2.0
#define HOST_LOST_Z_LIFT         10.0

/** \def POWER_LOSS_RECOVERY PLR_INTERVAL PLR_Z_RAISE PLR_PURGE_LENGTH
  Power loss recovery (M413, M1000, like Marlin), for prints from the SD
  card or SPI flash files: the print state goes into the SPI flash at
  every layer change and at least every PLR_INTERVAL seconds. After a
  power loss M1000 heats up, lifts Z by PLR_Z_RAISE mm, homes X and Y,
  primes PLR_PURGE_LENGTH mm and continues. Needs SPI_FLASH (the records
  use its last 8 kB); printing from the host it does nothing.
*/
#define POWER_LOSS_RECOVERY
#define PLR_INTERVAL             30
#define PLR_Z_RAISE              2.0
#define PLR_PURGE_LENGTH         3.0

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

/** \def Z_STEPPER_ALIGN_X1 Z_STEPPER_ALIGN_X2 Z_STEPPER_ALIGN_Y Z_STEPPER_X1 Z_STEPPER_X2 Z_STEPPER_ALIGN_ITERATIONS Z_STEPPER_ALIGN_ACC Z_STEPPER_ALIGN_MAX
  G34 (Z_STEPPER_ALIGN in the board file). Probe points (probe position,
  mm) near the left and right bed edge, the X positions of the Z (X1) and
  Z2 (X2) lead screws in the same coordinates (outside the bed, measure
  them). G34 stops after ITERATIONS or when both points are within ACC mm,
  it refuses to correct more than MAX mm.
*/
#define Z_STEPPER_ALIGN_X1       20.0
#define Z_STEPPER_ALIGN_X2       200.0
#define Z_STEPPER_ALIGN_Y        90.0
#define Z_STEPPER_X1             -35.0
#define Z_STEPPER_X2             255.0
#define Z_STEPPER_ALIGN_ITERATIONS 5
#define Z_STEPPER_ALIGN_ACC      0.02
#define Z_STEPPER_ALIGN_MAX      5.0

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
  Off: the bed runs PID (M304) with slow software PWM on PB0, see
  DEFINE_HEATER() in the board file. It holds about +-0.3 C instead of
  +-2 C, no "breathing" of the bed in Z, no big load steps on the PSU.
*/
//#define BANG_BANG_BED
#define BANG_BANG_BED_HYSTERESIS 2
#define BANG_BANG_BED_ON         255

/** \def DEFAULT_P DEFAULT_I DEFAULT_D DEFAULT_I_LIMIT
  Hotend PID from Marlin: Kp 22.2, Ki 1.08, Kd 114. Conversion to Teacup
  units: P = Kp * 256, I = Ki * 64, D = Kd * 128. The PID runs in floating
  point every 100 ms and takes these as Marlin's Kp, Ki (per second) and
  Kd (seconds), so values from Marlin or M303 fit directly.
  I_LIMIT allows the integral term the full output range (255 * 1024 / I).
  Runtime tuning: M301 P I D (Marlin units), M130..M133 (Teacup units).
  Marlin's PID_ADD_EXTRUSION_RATE has no equivalent.
*/
#define DEFAULT_P                ((int32_t)(22.2 * 256))   // 5683
#define DEFAULT_I                ((int32_t)(1.08 * 64))    // 69
#define DEFAULT_D                ((int32_t)(114.0 * 128))  // 14592
#define DEFAULT_I_LIMIT          (255L * 1024 / DEFAULT_I) // 3784

/** \def DEFAULT_BED_P DEFAULT_BED_I DEFAULT_BED_D DEFAULT_BED_I_LIMIT
  Bed PID (without BANG_BANG_BED), same units. A start for a 12 V MK2a /
  MK2B bed (Marlin's Kp 70, Ki 1.5, Kd 800); tune yours with
  M303 E-1 S60 C8 U1, then M500.
*/
#define DEFAULT_BED_P            ((int32_t)(70.0 * 256))   // 17920
#define DEFAULT_BED_I            ((int32_t)(1.5 * 64))     // 96
#define DEFAULT_BED_D            ((int32_t)(800.0 * 128))  // 102400
#define DEFAULT_BED_I_LIMIT      (255L * 1024 / DEFAULT_BED_I) // 2720

/** \def PID_D_FILTER PID_D_FILTER_BED PID_FUNCTIONAL_RANGE
  Time constant of the D term filter, seconds, for the hotend and the bed.
  Farther than PID_FUNCTIONAL_RANGE degrees from the target the heater is
  full on or off, the integral term starts over (Marlin's
  PID_FUNCTIONAL_RANGE).
*/
#define PID_D_FILTER             2.0f
#define PID_D_FILTER_BED         5.0f
#define PID_FUNCTIONAL_RANGE     10.0f

/** \def DEFAULT_PID_FAN_FF
  Hotend feed-forward for the part fan (M301 F, Marlin's PID_FAN_SCALING):
  PWM counts (0..255) the heater gets more at M106 S255, proportionally
  less at lower speeds. Without it the hotend drops 3..5 C when the fan
  starts after the first layers. To find it: hold the print temperature,
  note the mean hotend PWM (M105 @:) with the fan off and at S255, the
  difference is F. 0 = off.
*/
#define DEFAULT_PID_FAN_FF       0

/** \def FAN_KICKSTART_TIME FAN_MIN_PWM
  Part fan: starting from off, full power for FAN_KICKSTART_TIME ms first,
  so M106 S40 spins it up, too. M106 S1..255 maps to FAN_MIN_PWM..255, the
  lowest speed doesn't stall. 0 = off.
*/
#define FAN_KICKSTART_TIME       200
#define FAN_MIN_PWM              40

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
