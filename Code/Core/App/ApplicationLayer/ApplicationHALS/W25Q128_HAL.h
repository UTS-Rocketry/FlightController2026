#ifndef W25Q128_HAL_H
#define W25Q128_HAL_H

#include "W25Q128.h"
#include "stm32f4xx_hal.h"
#include "main.h"

HAL_StatusTypeDef flash_memory_init();
HAL_StatusTypeDef flash_sanity_check();
HAL_StatusTypeDef flash_read_record(uint32_t index, uint8_t *buff, uint16_t len);
HAL_StatusTypeDef flash_log_packet(uint8_t *buff, uint16_t len);


#endif

