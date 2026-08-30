#ifndef ODIN_STM32_HAL_COMPAT_H
#define ODIN_STM32_HAL_COMPAT_H

/*
 * Narrow compatibility surface for portable legacy modules.  Hardware access
 * in the Zephyr application goes through Zephyr drivers, never these types.
 */
#include <stdint.h>
#include <zephyr/kernel.h>

typedef enum {
	HAL_OK = 0,
	HAL_ERROR = 1,
	HAL_BUSY = 2,
	HAL_TIMEOUT = 3,
} HAL_StatusTypeDef;

typedef struct { uint8_t unused; } SPI_HandleTypeDef;
typedef struct { uint8_t unused; } UART_HandleTypeDef;
typedef struct { uint8_t unused; } GPIO_TypeDef;

static inline uint32_t HAL_GetTick(void)
{
	return k_uptime_get_32();
}

#endif
