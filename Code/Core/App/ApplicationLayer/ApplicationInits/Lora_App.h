#ifndef LORA_APP_H
#define LORA_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

extern volatile uint8_t lora_tx_done_flag;

void HAL_GPIOEXTI_Callback(uint16t GPIO_PIN);

#ifdef __cplusplus
}
#endif

#endif