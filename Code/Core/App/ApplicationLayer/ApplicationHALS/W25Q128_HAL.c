#include "W25Q128_HAL.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define FLASH_LOG_START_ADDR    0x000000
#define FLASH_RECORD_SIZE       64         /*matches serializer output */ 
#define FLASH_MAX_RECORDS       (W25Q128_TOTAL_SIZE / FLASH_RECORD_SIZE)

static uint32_t flash_write_addr = FLASH_LOG_START_ADDR;
static uint32_t flash_record_count = 0;

static W25Q128_Handle_t flash;
extern SPI_HandleTypeDef hspi3;

HAL_StatusTypeDef flash_memory_init() {

    HAL_StatusTypeDef result;

    flash.hspi = &hspi3;
    flash.cs_port = CSFlashmMemory_GPIO_Port;
    flash.cs_pin = CSFlashmMemory_Pin;

    result = W25Q128_Init(&flash);
    if(result != HAL_OK) {

        #ifdef DEBUG
            printf("FLash memory init failed\r\n");
        #endif
        return HAL_ERROR;
    }

    return result;


}

HAL_StatusTypeDef flash_sanity_check() {
    
    HAL_StatusTypeDef result;

    uint8_t m, t, c;
    result = W25Q128_ReadJEDECID(&flash, &m, &t, &c);

    if(result != HAL_OK) {

        #ifdef DEBUG
            printf("FLash JEDECID failed\r\n");
        #endif

        return HAL_ERROR;
    }

    // Should print EF 40 18
    #ifdef DEBUG
        printf("JEDEC: %02X %02X %02X\r\n", m, t, c);
    #endif
    return result;

    
}

HAL_StatusTypeDef flash_log_packet(uint8_t *buff, uint16_t len)
{
    if (flash_record_count >= FLASH_MAX_RECORDS) {
        #ifdef DEBUG
            printf("Flash full\r\n");
        #endif

        return HAL_ERROR;
    }

    if ((flash_write_addr % W25Q128_SECTOR_SIZE) == 0) {
        HAL_StatusTypeDef ret = W25Q128_SectorErase(&flash, flash_write_addr);
        if (ret != HAL_OK) {
            #ifdef DEBUG
                printf("Sector erase failed at 0x%06lX\r\n", flash_write_addr);
            #endif
            return ret;
        }
    }

    HAL_StatusTypeDef ret = W25Q128_PageProgram(&flash, flash_write_addr, buff, len);
    if (ret != HAL_OK) {
        #ifdef DEBUG
            printf("Flash write failed at 0x%06lX\r\n", flash_write_addr);
        #endif
        return ret;
    }

    flash_write_addr += FLASH_RECORD_SIZE;
    flash_record_count++;

    return HAL_OK;
}

HAL_StatusTypeDef flash_read_record(uint32_t index, uint8_t *buff, uint16_t len)
{
    if (index >= flash_record_count) return HAL_ERROR;
    
    uint32_t addr = FLASH_LOG_START_ADDR + (index * FLASH_RECORD_SIZE);
    return W25Q128_ReadData(&flash, addr, buff, len);
}

uint32_t flash_get_record_count(void)
{
    return flash_record_count;
}