#ifndef ODIN_FLIGHT_SENSORS_H
#define ODIN_FLIGHT_SENSORS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
	float altitude;
	float pressure;
	float temperature;
	float velocity;
	float x_mg;
	float y_mg;
	float z_mg;
	float x_mg_IMU;
	float y_mg_IMU;
	float z_mg_IMU;
	float x_gy;
	float y_gy;
	float z_gy;
	float kalman_altitude;
	float kalman_velocity;
	uint8_t flight_state;
} FlightSensorData;

int odin_sensors_init(void);
int odin_sensors_read_imu(FlightSensorData *data);
int odin_sensors_read_baro(FlightSensorData *data);

#endif
