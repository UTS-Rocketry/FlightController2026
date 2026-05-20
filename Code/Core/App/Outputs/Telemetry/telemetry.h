#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "flight_sensors.h"
#include "W25Q128_HAL.h"

/* Serial print for flight sensor values */
void serial_print(const FlightSensorData *sensordata);
HAL_StatusTypeDef lora_tx_telemetry(FlightSensorData *sensordata);
HAL_StatusTypeDef flash_log_telemetry(FlightSensorData *sensorData);
HAL_StatusTypeDef flash_dump_serial(void);

#ifdef __cplusplus
}
#endif

#endif