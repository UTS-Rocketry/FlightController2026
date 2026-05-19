#include "W25Q128_HAL.h"
#include "W25Q128.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal_def.h"

static W25Q128_Handle_t flash;

HAL_StatusTypeDef flash_memory_init() {

    HAL_StatusTypeDef result;

    flash.hspi = &hspi3;
    flash.cs_port = CSFlashmMemory_GPIO_Port;
    flash.cs_pin = CSFlashmMemory_Pin;

    result = W25Q128_Init(&flash);
    if(result != HAL_OK) {

        printf("FLash memory init failed\r\n");

        return HAL_ERROR;
    }

    return result;


}

HAL_StatusTypeDef flash_sanity_check() {
    
    HAL_StatusTypeDef result;

    uint8_t m, t, c;
    result = W25Q128_ReadJEDECID(&flash, &m, &t, &c);

    if(result != HAL_OK) {

        printf("FLash JEDECID failed\r\n");

        return HAL_ERROR;
    }

    // Should print EF 40 18
    printf("JEDEC: %02X %02X %02X\r\n", m, t, c);
    
    return result;

    
}
