/** \file
	\brief Manage temperature sensors

  \note All temperatures are stored as 14.2 fixed point in Teacup, so we have
  a range of 0 - 16383.75 deg Celsius at a precision of 0.25 deg.

  Thermistors are converted with the Steinhart-Hart equation, see
  thermistor_init() below. No lookup tables any more.
*/

#include "temp.h"

#include <stdlib.h>
#include "serial.h"
#include "clock.h"
#include "debug.h"
#include "sersendf.h"
#include "heater.h"
#include "pinio.h"
#include "thermal_protection.h"

#ifdef	TEMP_MAX6675
  #include "spi.h"
#endif

#ifdef TEMP_MCP3008
  #include "spi.h"
#endif

#include "analog.h"

#if defined TEMP_THERMISTOR || defined TEMP_MCP3008
  #include <math.h>
  static void thermistor_init(void);
#endif

/// holds metadata for each temperature sensor
typedef struct {
	temp_type_t temp_type; ///< type of sensor
	uint8_t     temp_pin;  ///< ADC channel (analog, MCP3008 channel)
	uint8_t     pin_id;    ///< PIN_ID() of the pin (chip select for MAX6675)
	heater_t    heater;    ///< associated heater if any
	uint8_t		additional; ///< additional, sensor type specifc config
} temp_sensor_definition_t;

#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  { (type), PIN_ADC(pin), PIN_ID(pin), (HEATER_ ## name), (additional) },
static const temp_sensor_definition_t temp_sensors[NUM_TEMP_SENSORS] =
{
#include	"config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR

/// this struct holds the runtime sensor data- read temperatures, targets, etc
static struct {
  uint_fast16_t         last_read_temp; ///< last received reading
  uint_fast16_t         target_temp;    ///< manipulate attached heater to attempt to achieve this value

  uint_fast16_t         temp_residency; ///< how long have we been close to target temperature in temp ticks?

  uint_fast8_t  active;          ///< State machine tracker for readers that need it.

  uint8_t       age;             ///< 250 ms ticks since the last valid reading.
  uint8_t       manual;          ///< Heater controlled elsewhere (M303).
  float         temp_c;          ///< Last reading in C, not rounded (PID).
} temp_sensors_runtime[NUM_TEMP_SENSORS];

/**
  Temperature of the latest conversion in C, as calculated, before rounding
  to quarter degrees. Set by conversions which have it (thermistors, the
  simulated hotend), used once by temp_sensor_tick().
*/
static float conv_c;
static uint8_t conv_c_set;

/** \def TEMP_EWMA

  Default alpha constant for the Exponentially Weighted Moving Average (EWMA)
  for smoothing noisy sensors. Instrument Engineer's Handbook, 4th ed,
  Vol 2 p126 says values of 50 to 100 are typical.

  This is scaled by factor 1000. Setting it to 1000 turns EWMA off.
*/
#ifndef TEMP_EWMA
  #define TEMP_EWMA 1000
#endif

#define EWMA_SCALE  1024L
#define EWMA_ALPHA  ((TEMP_EWMA * EWMA_SCALE + 500) / 1000)

// If EWMA is used, continuously update analog reading for more data points.
#define TEMP_READ_CONTINUOUS (EWMA_ALPHA < EWMA_SCALE)
#define TEMP_NOT_READY       0xffff

/**
  Flag for wether we currently wait for achieving temperatures.
*/
static volatile uint8_t wait_for_temp = 0;
/// Set when M108 ended a wait.
static volatile uint8_t wait_cancelled = 0;
/// Set by every M108, cleared by temp_m108_seen().
static volatile uint8_t m108_flag = 0;
/// Sensor a wait refers to, TEMP_SENSOR_none = all (M116).
static uint8_t wait_scope = TEMP_SENSOR_none;


// Automatic temperature report period (AUTOREPORT_TEMP)
static uint8_t          periodic_temp_seconds;
static temp_sensor_t    periodic_temp_index;
static uint8_t          periodic_temp_timer;

#ifdef TEMP_DUMMY
  static uint16_t dummy_temp[NUM_TEMP_SENSORS];
  volatile int32_t temp_dummy_force = -1;
  volatile int32_t temp_dummy_plant = 0;
  volatile int32_t temp_dummy_fan_loss = 0;
#endif

/// Set up temp sensors.
void temp_init(void) {
  #if defined TEMP_THERMISTOR || defined TEMP_MCP3008
    thermistor_init();
  #endif

	uint8_t i;
	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		switch(temp_sensors[i].temp_type) {
      #ifdef TEMP_MAX6675
        case TT_MAX6675:
          // The sensor's pin is its chip select.
          spi_deselect_id(temp_sensors[i].pin_id);
          pin_id_output(temp_sensors[i].pin_id);
          break;
      #endif

      #ifdef TEMP_MCP3008
        case TT_MCP3008:
          // Chip select is set up in spi_init().
          break;
      #endif

      #ifdef TEMP_DUMMY
        case TT_DUMMY:
          // Start at room temperature, 25 deg C.
          dummy_temp[i] = 25 * 4;
          break;
      #endif

			default: /* prevent compiler warning */
				break;
		}
	}
}

#if defined TEMP_THERMISTOR || defined TEMP_MCP3008
/**
  Thermistors, Steinhart-Hart equation:

    1 / T = A + B * ln(R) + C * ln(R)^3        (T in Kelvin)

  A, B and C are calculated once at startup from the three calibration
  points of DEFINE_THERMISTOR() (temperature / resistance pairs from the
  thermistor's datasheet). This is the same model Marlin's
  createTemperatureLookupMarlin.py uses to build its tables, so the same
  three points give the same curve.

  The thermistor is expected between ADC pin and ground, the pull-up
  resistor between ADC pin and the ADC reference (3.3 V, VDDA):

    R = R_pullup * adc / (adc_max - adc)
*/
typedef struct {
  float rp, t1, r1, t2, r2, t3, r3;
} thermistor_def_t;

#undef DEFINE_THERMISTOR
#define DEFINE_THERMISTOR(name, rp, t1, r1, t2, r2, t3, r3) \
  { (rp), (t1), (r1), (t2), (r2), (t3), (r3) },
static const thermistor_def_t thermistor_defs[NUM_THERMISTORS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_THERMISTOR

/// Steinhart-Hart coefficients and pull-up, calculated at startup.
static struct {
  float a, b, c, rp;
} thermistor_coef[NUM_THERMISTORS];

/// Reported for a shorted sensor (1000 C), makes MAXTEMP trigger.
#define THERMISTOR_SHORTED  (1000 * 4)

/// Calculate A, B, C from three (T, R) points.
static void thermistor_init(void) {
  uint8_t i;

  for (i = 0; i < NUM_THERMISTORS; i++) {
    const thermistor_def_t *d = &thermistor_defs[i];
    double l1 = log((double)d->r1);
    double l2 = log((double)d->r2);
    double l3 = log((double)d->r3);
    double y1 = 1.0 / ((double)d->t1 + 273.15);
    double y2 = 1.0 / ((double)d->t2 + 273.15);
    double y3 = 1.0 / ((double)d->t3 + 273.15);
    double g2 = (y2 - y1) / (l2 - l1);
    double g3 = (y3 - y1) / (l3 - l1);
    double c = (g3 - g2) / (l3 - l2) / (l1 + l2 + l3);
    double b = g2 - c * (l1 * l1 + l1 * l2 + l2 * l2);
    double a = y1 - (b + l1 * l1 * c) * l1;

    thermistor_coef[i].a = (float)a;
    thermistor_coef[i].b = (float)b;
    thermistor_coef[i].c = (float)c;
    thermistor_coef[i].rp = d->rp;
  }
}

/**
  Convert a raw ADC reading into a temperature.

  \param th      Thermistor (THERMISTOR_<name>).
  \param adc     Raw reading.
  \param adc_max Full scale of the ADC (ANALOG_MAX, 1023 for MCP3008).

  \return Temperature in 14.2 fixed point. An open sensor (reading at full
          scale) gives 0, which makes MINTEMP trigger; a shorted one (reading
          0) gives 1000 C, which makes MAXTEMP trigger.
*/
static uint16_t thermistor_to_qc(uint8_t th, uint32_t adc, uint32_t adc_max) {
  float r, l, t;

  if (th >= NUM_THERMISTORS)
    return 0;
  if (adc == 0)
    return THERMISTOR_SHORTED;
  if (adc >= adc_max)
    return 0;

  r = thermistor_coef[th].rp * (float)adc / (float)(adc_max - adc);
  l = logf(r);
  t = 1.0f / (thermistor_coef[th].a + thermistor_coef[th].b * l +
              thermistor_coef[th].c * l * l * l) - 273.15f;

  if (t <= 0.0f)
    return 0;
  if (t >= 1000.0f)
    return THERMISTOR_SHORTED;
  conv_c = t;
  conv_c_set = 1;
  return (uint16_t)(t * 4.0f + 0.5f);
}
#endif /* TEMP_THERMISTOR || TEMP_MCP3008 */

#ifdef TEMP_MAX6675
static uint16_t temp_max6675_read(temp_sensor_t i) {
  uint16_t temp;

  // Note: MAX6675 can give a reading every 0.22s, max. 4.3 MHz SPI clock.
  spi_speed_100_400();
  spi_select_id(temp_sensors[i].pin_id);
  // No delay required, see
  // https://github.com/Traumflug/Teacup_Firmware/issues/22

  // Read MSB.
  temp = spi_rw(0) << 8;
  // Read LSB.
  temp |= spi_rw(0);

  spi_deselect_id(temp_sensors[i].pin_id);

  if ((temp & 0x4) == 0) {
    temp = temp >> 3;
  }
  else { 
    // thermocouple open, send "not ready"
    temp = TEMP_NOT_READY;
    // and we will read it next time.
    temp_sensors_runtime[i].active = 0;
  }

  return temp;
}

static uint16_t temp_read_max6675(temp_sensor_t i) {
  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return temp_max6675_read(i);

    case 22:  // read temperature at most every 220ms
      temp_sensors_runtime[i].active = 0;
      break;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_MAX6675 */

#ifdef TEMP_THERMISTOR
static uint16_t temp_read_thermistor(temp_sensor_t i) {
  // Needs to be 'static', else GCC diagnostics emits a warning "'result' may
  // be used uninitialized". No surprise about this warning, it's part of the
  // logic that in some configurations temp_sensors_runtime[i].active never
  // reaches more than 1.
  static uint16_t result;

  switch (temp_sensors_runtime[i].active++) {
    case 1:  // Start ADC conversion.
      #ifdef NEEDS_START_ADC
        #if TEMP_READ_CONTINUOUS
          result = analog_read(i);
        #endif
        start_adc();
        #if ! TEMP_READ_CONTINUOUS
          return TEMP_NOT_READY;
        #endif
      #endif
      // If not in continuous mode or no need for start_adc() fall through.

    case 2:  // Convert temperature values.
      #if ! defined NEEDS_START_ADC || ! TEMP_READ_CONTINUOUS
        result = analog_read(i);
      #endif
      temp_sensors_runtime[i].active = 0;
      return thermistor_to_qc(temp_sensors[i].additional, result, ANALOG_MAX);
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_THERMISTOR */

#ifdef TEMP_MCP3008
/**
  Read a measurement from the MCP3008 analog-digital-converter (ADC).

  \param channel The ADC channel to read.

  \return The raw ADC reading.

  Documentation for this ADC see

    https://www.adafruit.com/datasheets/MCP3008.pdf.
*/
static uint16_t temp_mcp3008_read(uint8_t channel) {
  uint8_t temp_h, temp_l;

  spi_speed_100_400();
  spi_select_mcp3008();

  // Start bit.
  spi_rw(0x01);

  // Send read address and get MSB, then LSB byte.
  temp_h = spi_rw((0x08 | channel) << 4) & 0x03;
  temp_l = spi_rw(0);

  spi_deselect_mcp3008();

  return temp_h << 8 | temp_l;
}

static uint16_t temp_read_mcp3008(temp_sensor_t i) {
  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return thermistor_to_qc(temp_sensors[i].additional,
                              temp_mcp3008_read(temp_sensors[i].temp_pin), 1023);
    case 10:  // Idle for 100ms.
      temp_sensors_runtime[i].active = 0;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_MCP3008 */

#ifdef TEMP_AD595
static uint16_t temp_read_ad595(temp_sensor_t i) {
  // Needs to be 'static', see comment in temp_read_thermistor().
  static uint16_t result;

  switch (temp_sensors_runtime[i].active++) {
    case 1:  // Start ADC conversion.
      #ifdef NEEDS_START_ADC
        #if TEMP_READ_CONTINUOUS
          result = analog_read(i);
        #endif
        start_adc();
        #if ! TEMP_READ_CONTINUOUS
          return TEMP_NOT_READY;
        #endif
      #endif
      // If not in continuous mode or no need for start_adc() fall through.

    case 2:  // Convert temperature values.
      #if ! TEMP_READ_CONTINUOUS
        result = analog_read(i);
      #endif
      temp_sensors_runtime[i].active = 0;
      // AD595: 10 mV/C. 12 bit ADC with 3.3 V reference, 14.2 fixed point:
      // qC = adc * 3300 mV / 4095 / 10 mV * 4. Readable up to 330 C.
      return (uint16_t)(((uint32_t)result * 1320UL + ANALOG_MAX / 2) / ANALOG_MAX);
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_AD595 */

#ifdef TEMP_DUMMY
/**
  Simulated sensor for tests: follows the target (but not below 25 C room
  temperature) by one quarter degree per reading. temp_dummy_force >= 0 overrides the value (test hook, written by
  the Renode tests directly into memory to simulate sensor faults).
*/
/**
  Test hook: temp_dummy_plant = 1 turns the dummy sensor into a simulated
  hotend: 4 C/s heating at full power, cooling with a time constant of
  40 s towards 25 C, 1 s dead time. The part fan (HEATER_FAN) cools the hotend by
  another temp_dummy_fan_loss / 1000 C/s at full speed, also 1 s delayed
  (spin-up). Used to test PID,
  its fan compensation and M303.
*/
static void dummy_plant(temp_sensor_t i) {
  static float t[NUM_TEMP_SENSORS];
  static uint8_t delay_line[NUM_TEMP_SENSORS][4];
  static uint8_t fan_line[NUM_TEMP_SENSORS][4];
  static uint8_t pos[NUM_TEMP_SENSORS];
  static uint32_t last[NUM_TEMP_SENSORS];
  uint32_t now = clock_millis();
  heater_t h = temp_sensors[i].heater;

  if (t[i] < 1.0f) {
    t[i] = (float)dummy_temp[i] / 4.0f;
    last[i] = now;
  }
  while (now - last[i] >= 250) {
    uint8_t out = (h < NUM_HEATERS) ? heaters_runtime[h].heater_output : 0;
    uint8_t delayed = delay_line[i][pos[i]];
    float fan = (float)fan_line[i][pos[i]] / 255.0f *
                (float)temp_dummy_fan_loss / 1000.0f;

    fan_line[i][pos[i]] = 0;
    #if defined HEATER_FAN && defined HEATER_EXTRUDER
      if (h == HEATER_EXTRUDER)
        fan_line[i][pos[i]] = heaters_runtime[HEATER_FAN].heater_output;
    #endif
    delay_line[i][pos[i]] = out;
    pos[i] = (pos[i] + 1) & 3;
    t[i] += 0.25f * (4.0f * (float)delayed / 255.0f - (t[i] - 25.0f) / 40.0f -
                     fan);
    last[i] += 250;
  }
  dummy_temp[i] = (uint16_t)(t[i] * 4.0f + 0.5f);
  conv_c = t[i];
  conv_c_set = 1;
}

static uint16_t temp_read_dummy(temp_sensor_t i) {
  int32_t force = temp_dummy_force;

  uint16_t goal = temp_sensors_runtime[i].target_temp;

  if (goal < 25 * 4)              // Cools down to room temperature only.
    goal = 25 * 4;

  if (force >= 0)
    dummy_temp[i] = (uint16_t)force;
  else if (temp_dummy_plant)
    dummy_plant(i);
  else if (goal > dummy_temp[i])
    dummy_temp[i]++;
  else if (goal < dummy_temp[i])
    dummy_temp[i]--;

  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return dummy_temp[i] ;
    case 5:  // Idle for 50ms.
      temp_sensors_runtime[i].active = 0;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_DUMMY */

static uint16_t read_temp_sensor(temp_sensor_t i) {
  switch (temp_sensors[i].temp_type) {
    #ifdef TEMP_MAX6675
      case TT_MAX6675:
        return temp_read_max6675(i);
    #endif

    #ifdef TEMP_THERMISTOR
      case TT_THERMISTOR:
        return temp_read_thermistor(i);
    #endif

    #ifdef TEMP_MCP3008
      case TT_MCP3008:
        return temp_read_mcp3008(i);
    #endif

    #ifdef TEMP_AD595
      case TT_AD595:
        return temp_read_ad595(i);
    #endif

    #ifdef TEMP_DUMMY
      case TT_DUMMY:
        return temp_read_dummy(i);
    #endif

    default: /* Prevent compiler warning. */
      return 0;
  }
}

uint8_t temp_m108_seen(void) {
  uint8_t f = m108_flag;

  m108_flag = 0;
  return f;
}

void temp_set_manual(temp_sensor_t index, uint8_t manual) {
  if (index < NUM_TEMP_SENSORS)
    temp_sensors_runtime[index].manual = manual;
}

static void run_pid_loop(int i) {
  if (temp_sensors[i].heater < NUM_HEATERS && ! temp_sensors_runtime[i].manual) {
    heater_tick(temp_sensors[i].heater, temp_sensors[i].temp_type,
                temp_sensors_runtime[i].last_read_temp,
                temp_sensors_runtime[i].target_temp,
                temp_sensors_runtime[i].temp_c);
  }
}

/**
  Called every 10ms from clock.c. Check all temp sensors that are ready for
  checking. When complete, update the PID loop for sensors tied to heaters.
*/
void temp_sensor_tick(void) {
	uint8_t i = 0;

	for (; i < NUM_TEMP_SENSORS; i++) {
    if (TEMP_READ_CONTINUOUS)
      if ( ! temp_sensors_runtime[i].active)
        temp_sensors_runtime[i].active = 1;

    if (temp_sensors_runtime[i].active) {
      uint16_t temp;
      float c;

      conv_c_set = 0;
      temp = read_temp_sensor((temp_sensor_t)i);
      if (temp == TEMP_NOT_READY)
        continue;

      temp_sensors_runtime[i].age = 0;
      c = conv_c_set ? conv_c : (float)temp * 0.25f;

      // Handle moving average.
      temp_sensors_runtime[i].last_read_temp = (uint16_t)(
        (EWMA_ALPHA * temp +
         (EWMA_SCALE - EWMA_ALPHA) * temp_sensors_runtime[i].last_read_temp) /
        EWMA_SCALE);
      if (temp_sensors_runtime[i].temp_c == 0.0f)
        temp_sensors_runtime[i].temp_c = c;
      else
        temp_sensors_runtime[i].temp_c +=
          (float)EWMA_ALPHA / (float)EWMA_SCALE *
          (c - temp_sensors_runtime[i].temp_c);

      if ( ! TEMP_READ_CONTINUOUS) {
        /**
          In one-shot mode we only update temps when triggered by
          temp_pid_tick(). So here we run the PID loop through a cycle.
        */
        run_pid_loop(i);
      }
    }
  }
}

/**
  Called every 250ms from clock.c. Thermal protection for all sensors
  driving a heater.
*/
void temp_heater_tick(void) {
  uint8_t i;

  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (temp_sensors[i].heater < NUM_HEATERS) {
      if (temp_sensors_runtime[i].age < 255)
        temp_sensors_runtime[i].age++;
      thermal_protection_check(i, temp_sensors_runtime[i].last_read_temp,
                               temp_sensors_runtime[i].target_temp,
                               temp_sensors_runtime[i].age,
        heaters_runtime[temp_sensors[i].heater].heater_output != 0);
    }
  }
}

/**
  Called every 100 ms from clock.c: PID loops at 10 Hz. Sensors read
  continuously get their loop run right away, the others a new reading
  first; they run the loop when it's done (at most as often as the sensor
  can deliver, e.g. MAX6675 every 220 ms).
*/
void temp_pid_tick(void) {
  uint8_t i;

  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (TEMP_READ_CONTINUOUS)
      run_pid_loop(i);
    else if ( ! temp_sensors_runtime[i].active)
      temp_sensors_runtime[i].active = 1;
  }
}

/**
  Called every 1s from clock.c. Update temperature residency info.
*/
void temp_residency_tick(void) {
  uint8_t i;

  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		if (labs((int16_t)(temp_sensors_runtime[i].last_read_temp - temp_sensors_runtime[i].target_temp)) < (TEMP_HYSTERESIS*4)) {
			if (temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*120))
        temp_sensors_runtime[i].temp_residency += 100;
		}
		else {
			// Deal with flakey sensors which occasionally report a wrong value
			// by setting residency back, but not entirely to zero.
      if (temp_sensors_runtime[i].temp_residency > 100)
        temp_sensors_runtime[i].temp_residency -= 100;
			else
				temp_sensors_runtime[i].temp_residency = 0;
		}

    if (DEBUG_PID && (debug_flags & DEBUG_PID))
      sersendf_P(("DU temp: {%d %d %d.%d}"), i,
                 temp_sensors_runtime[i].last_read_temp,
                 temp_sensors_runtime[i].last_read_temp / 4,
                 (temp_sensors_runtime[i].last_read_temp & 0x03) * 25);
	}
  if (DEBUG_PID && (debug_flags & DEBUG_PID))
    sersendf_P(("\n"));
}

/**
 * Report whether all temp sensors in use are reading their target
 * temperatures. Used for M116 and friends.
 */
uint8_t	temp_achieved(void) {
	uint8_t i;
	uint8_t all_ok = 255;

	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (temp_sensors_runtime[i].target_temp > 0 &&
        temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*100))
			all_ok = 0;
	}
	return all_ok;
}

/**
  Note that we're waiting for temperatures to achieve their target. Typically
  set by M116.
*/
/// Whether sensor i reached its target for the residency time.
static uint8_t sensor_achieved(uint8_t i) {
  return temp_sensors_runtime[i].target_temp == 0 ||
         temp_sensors_runtime[i].temp_residency >= (TEMP_RESIDENCY_TIME*100);
}

/// Whether the sensor(s) of the current wait reached the target.
static uint8_t scope_achieved(void) {
  if (wait_scope < NUM_TEMP_SENSORS)
    return sensor_achieved(wait_scope);
  return temp_achieved();
}

void temp_set_wait(void) {
  wait_scope = TEMP_SENSOR_none;
  wait_cancelled = 0;
  wait_for_temp = 1;
}

/**
  Wether we're currently waiting for achieving temperatures.
*/
uint8_t temp_waiting(void) {
  if (wait_for_temp && scope_achieved()) {
    wait_for_temp = 0;
  }
  return wait_for_temp;
}

uint8_t temp_wait_active(void) {
  return wait_for_temp;
}

uint8_t temp_wait_cancelled(void) {
  return wait_cancelled;
}

/**
  Stop waiting for temperatures (M108, from the emergency parser).
*/
void temp_cancel_wait(void) {
  m108_flag = 1;
  if (wait_for_temp)
    wait_cancelled = 1;
  wait_for_temp = 0;
}

/**
  Wait until all temperatures are achieved.
*/
void temp_wait(void) {
  while (wait_for_temp && ! scope_achieved()) {
    clock_poll();
  }
}

/**
  Wait for one sensor (M109, M190).

  \param index     Temperature sensor.
  \param heat_only 1: don't wait if the sensor is already hotter than the
                   target (M109 S), 0: wait for heating and cooling (M109 R).

  Returns early with M108.
*/
void temp_wait_sensor(temp_sensor_t index, uint8_t heat_only) {
  uint16_t target;

  if (index >= NUM_TEMP_SENSORS)
    return;

  target = temp_sensors_runtime[index].target_temp;
  if (target == 0)
    return;
  if (heat_only && temp_sensors_runtime[index].last_read_temp >= target)
    return;

  wait_scope = index;
  wait_cancelled = 0;
  wait_for_temp = 1;
  while (wait_for_temp && ! sensor_achieved(index))
    clock_poll();
  wait_for_temp = 0;
  wait_scope = TEMP_SENSOR_none;
}

/// specify a target temperature
/// \param index sensor to set a target for
/// \param temperature target temperature to aim for
void temp_set(temp_sensor_t index, uint16_t temperature) {
	if (index >= NUM_TEMP_SENSORS)
		return;

	if (temp_sensors[index].heater < NUM_HEATERS)
		temperature = thermal_protection_limit_target(index, temperature);

	// only reset residency if temp really changed
	if (temp_sensors_runtime[index].target_temp != temperature) {
		temp_sensors_runtime[index].target_temp = temperature;
		temp_sensors_runtime[index].temp_residency = 0;
	}
}

/// return most recent reading for a sensor
/// \param index sensor to read
uint16_t temp_get(temp_sensor_t index) {
	if (index >= NUM_TEMP_SENSORS)
		return 0;

	return temp_sensors_runtime[index].last_read_temp;
}

uint16_t temp_get_target(temp_sensor_t index) {
  if (index >= NUM_TEMP_SENSORS)
    return 0;
  return temp_sensors_runtime[index].target_temp;
}

// extruder doesn't have sersendf_P
static void single_temp_print(temp_sensor_t index) {
	uint8_t c = (temp_sensors_runtime[index].last_read_temp & 3) * 25;
	sersendf_P(("%u.%u"), temp_sensors_runtime[index].last_read_temp >> 2, c);
  #ifdef REPORT_TARGET_TEMPS
    sersendf_P(("/"));
    c = (temp_sensors_runtime[index].target_temp & 3) * 25;
    sersendf_P(("%u.%u"), temp_sensors_runtime[index].target_temp >> 2, c);
  #endif
}

/// set parameters for periodic temperature reporting
/// \param secs reporting interval in seconds; 0 to disable reporting
/// \param index sensor to report
void temp_periodic_config(uint8_t secs, temp_sensor_t index) {
  periodic_temp_seconds = secs;
  periodic_temp_index = index;
  periodic_temp_timer = secs;
}

/// send temperatures to the host periodically.
/// Called once per second from clock.c
void temp_periodic_print(void) {
  if (periodic_temp_seconds == 0)
    return;
  if (--periodic_temp_timer != 0)
    return;

  temp_print(periodic_temp_index);
  periodic_temp_timer = periodic_temp_seconds;
}

/// send temperatures to host
/// \param index sensor value to send
void temp_print(temp_sensor_t index) {

	if (index == TEMP_SENSOR_none) { // standard behaviour
		#ifdef HEATER_EXTRUDER
			sersendf_P(("T:"));
      single_temp_print(TEMP_SENSOR_extruder);
		#endif
		#ifdef HEATER_BED
			sersendf_P((" B:"));
      single_temp_print(TEMP_SENSOR_bed);
		#endif
    // Heater outputs 0..255, like Marlin (hosts show them as power).
    #ifdef HEATER_EXTRUDER
      sersendf_P((" @:%su"), heaters_runtime[HEATER_EXTRUDER].heater_output);
    #endif
    #ifdef HEATER_BED
      sersendf_P((" B@:%su"), heaters_runtime[HEATER_BED].heater_output);
    #endif
	}
	else {
		if (index >= NUM_TEMP_SENSORS)
			return;
		sersendf_P(("T[%su]:"), index);
		single_temp_print(index);
	}
  serial_writechar('\n');
}
