#ifndef W25Q128_HAL_H
#define W25Q128_HAL_H

#include "W25Q128.h"
#include "stm32f4xx_hal.h"
#include "main.h"

HAL_StatusTypeDef flash_memory_init();
HAL_StatusTypeDef flash_sanity_check();


#endif

