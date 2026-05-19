
#ifndef FLIGHT_SENSORS_H
#define FLIGHT_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "BMP388.h"
#include "lsm6dsox_reg.h"
#include "h3lis331dl_reg.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Flight sensor data structure */
typedef struct
{
    float altitude;
    float pressure;
    float temperature;

    /* High-g accelerometer */
    float x_mg;
    float y_mg;
    float z_mg;

    /* IMU accelerometer */
    float x_mg_IMU;
    float y_mg_IMU;
    float z_mg_IMU;

    /* IMU gyroscope */
    float x_gy;
    float y_gy;
    float z_gy;

} FlightSensorData;

/* Initialization */
HAL_StatusTypeDef flight_sensors_init(void);

/* Read + update all flight sensor values */
HAL_StatusTypeDef flight_sensors_update(FlightSensorData *sensordata);

#ifdef __cplusplus
}
#endif

#endif