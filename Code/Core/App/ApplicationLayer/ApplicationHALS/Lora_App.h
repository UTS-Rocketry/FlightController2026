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



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
HAL_StatusTypeDef lora_App_Init(void);

#ifdef __cplusplus
}
#endif

#endif
