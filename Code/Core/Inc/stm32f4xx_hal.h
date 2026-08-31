#ifndef ODIN_STM32_HAL_COMPAT_H
#define ODIN_STM32_HAL_COMPAT_H

/*
 * Keep legacy includes working while using the STM32 HAL types supplied by
 * Zephyr's pinned hal_stm32 module.  Zephyr also supplies HAL_GetTick() and
 * HAL_Delay(), so defining local substitutes here would conflict with the SoC
 * headers pulled in by <zephyr/kernel.h>.
 */
#include_next <stm32f4xx_hal.h>

#endif
