#ifndef	_TEMP_H
#define	_TEMP_H

#include "config_wrapper.h"
#include <stdint.h>


#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) TEMP_SENSOR_ ## name,
typedef enum {
	#include "config_wrapper.h"
	NUM_TEMP_SENSORS,
	TEMP_SENSOR_none
} temp_sensor_t;
#undef DEFINE_TEMP_SENSOR

/// Thermistor definitions (DEFINE_THERMISTOR), referenced as THERMISTOR_<name>.
#undef DEFINE_THERMISTOR
#define DEFINE_THERMISTOR(name, ...) THERMISTOR_ ## name,
typedef enum {
	#include "config_wrapper.h"
	NUM_THERMISTORS
} thermistor_t;
#undef DEFINE_THERMISTOR

typedef enum {
	TT_THERMISTOR,
	TT_MAX6675,
	TT_AD595,
	TT_PT100,
	TT_MCP3008,
	TT_DUMMY
} temp_type_t;


void temp_init(void);

void temp_sensor_tick(void);

void temp_heater_tick(void);

void temp_residency_tick(void);

void temp_periodic_config(uint8_t secs, temp_sensor_t index);
void temp_periodic_print(void);

uint8_t	temp_achieved(void);

void temp_set_wait(void);
void temp_cancel_wait(void);
/// Whether M108 arrived since the last call (clears the flag).
uint8_t temp_m108_seen(void);
/// Heater of this sensor is controlled elsewhere (M303), no PID.
void temp_set_manual(temp_sensor_t index, uint8_t manual);
/// Wait flag without side effects.
uint8_t temp_wait_active(void);
/// Whether the last wait was ended by M108.
uint8_t temp_wait_cancelled(void);
/// Wait for one sensor (M109/M190), see temp.c.
void temp_wait_sensor(temp_sensor_t index, uint8_t heat_only);
uint8_t temp_waiting(void);
void temp_wait(void);

void temp_set(temp_sensor_t index, uint16_t temperature);
uint16_t temp_get(temp_sensor_t index);
/// Target temperature of a sensor, quarter degrees.
uint16_t temp_get_target(temp_sensor_t index);

void temp_print(temp_sensor_t index);

#ifdef TEMP_DUMMY
  /// Test hook for TT_DUMMY sensors, see temp.c.
  extern volatile int32_t temp_dummy_force;
  extern volatile int32_t temp_dummy_plant;
#endif

#endif	/* _TEMP_H */
