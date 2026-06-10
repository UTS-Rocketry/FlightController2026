#ifndef LORA_APP_H
#define LORA_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "flight_sensors.h"
#include "crc16.h"
#include "packets.h"



typedef enum {

    IDLE            = 0x00,
    ARMED           = 0x01,
    POWERED_ASCENT  = 0x02,
    COASTING        = 0x03,
    APOGEE          = 0x04,
    DESCENT         = 0x05,
    LANDED          = 0x06,
    FAULT           = 0x07

} FlightStateLoRa;

extern volatile uint8_t lora_tx_done_flag;

void HAL_GPIOEXTI_Callback(uint16_t GPIO_PIN);
HAL_StatusTypeDef lora_App_Init();

#ifdef __cplusplus
}
#endif

#endif