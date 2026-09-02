#include "W25Q128_HAL.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define FLASH_LOG_START_ADDR    0x000000
#define FLASH_RECORD_SIZE       64         /*matches serializer output */ 
#define FLASH_MAX_RECORDS       (W25Q128_TOTAL_SIZE / FLASH_RECORD_SIZE)
#define FLASH_SYNC_WORD 0xAA
static uint32_t flash_erased_through_addr = FLASH_LOG_START_ADDR;

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

    result = flash_sanity_check();
  
    #ifdef DEBUG
        if (result != HAL_OK) {
        printf("flash memory failed sanity check\r\n");
        } else { 
        printf("Flash memory OK\r\n");
        }
    #endif

    result = flash_recover_write_pointer(); 

  #ifdef DEBUG
    if (result != HAL_OK) {
      printf("flash memory failed find start address\r\n");
    } else { 
      printf("Flash memory OK\r\n");
    }
  #endif

  result = flash_prepare_log_region(150);   // <-- add here, e.g. ~6 min of logging headroom
  #ifdef DEBUG
    if (result != HAL_OK) {
      printf("flash pre-erase failed\r\n");
    } else {
      printf("Flash pre-erase OK\r\n");
    }
  #endif



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
    if (flash_record_count >= FLASH_MAX_RECORDS) return HAL_ERROR;
    if (len > FLASH_RECORD_SIZE) return HAL_ERROR;   // NEW — bounds check, freebie fix #4

    if ((flash_write_addr % W25Q128_SECTOR_SIZE) == 0 &&
        flash_write_addr >= flash_erased_through_addr) {   // NEW — only erase if not pre-cleared
        HAL_StatusTypeDef ret = W25Q128_SectorErase(&flash, flash_write_addr);
        if (ret != HAL_OK) return ret;
        flash_erased_through_addr = flash_write_addr + W25Q128_SECTOR_SIZE;
    }

    HAL_StatusTypeDef ret = W25Q128_PageProgram(&flash, flash_write_addr, buff, len);
    if (ret != HAL_OK) return ret;

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


static uint8_t flash_record_is_written(uint32_t index)
{
    uint8_t sync_byte;
    uint32_t addr = FLASH_LOG_START_ADDR + (index * FLASH_RECORD_SIZE);

    if (W25Q128_ReadData(&flash, addr, &sync_byte, 1) != HAL_OK) {
        return 1; // read failure — assume written; costs a slot, never risks overwriting real data
    }
    return (sync_byte == FLASH_SYNC_WORD);
}

HAL_StatusTypeDef flash_recover_write_pointer(void)
{
    uint32_t lo = 0;
    uint32_t hi = FLASH_MAX_RECORDS; // exclusive; assumes append-only, no gaps

    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (flash_record_is_written(mid)) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }

    flash_record_count = lo;
    flash_write_addr = FLASH_LOG_START_ADDR + (lo * FLASH_RECORD_SIZE);

#ifdef DEBUG
    printf("Flash recovery: resuming at record %lu (addr 0x%06lX)\r\n",
           flash_record_count, flash_write_addr);
#endif

    return HAL_OK;
}

HAL_StatusTypeDef flash_prepare_log_region(uint32_t num_sectors)
{
    uint32_t erase_addr = (flash_write_addr % W25Q128_SECTOR_SIZE == 0)
        ? flash_write_addr
        : flash_write_addr - (flash_write_addr % W25Q128_SECTOR_SIZE) + W25Q128_SECTOR_SIZE;

    uint32_t chip_end = FLASH_LOG_START_ADDR + (FLASH_MAX_RECORDS * FLASH_RECORD_SIZE);

    for (uint32_t i = 0; i < num_sectors && erase_addr < chip_end; i++) {
        HAL_StatusTypeDef ret = W25Q128_SectorErase(&flash, erase_addr);
        if (ret != HAL_OK) return ret;
        erase_addr += W25Q128_SECTOR_SIZE;
    }

    flash_erased_through_addr = erase_addr;   // NEW — remember the known-clean boundary
    return HAL_OK;
}

#ifdef FLASH_ERASE_BUILD
  HAL_StatusTypeDef flash_full_chip_erase(void)
  {
      uint32_t chip_end = W25Q128_TOTAL_SIZE;

      for (uint32_t addr = 0; addr < chip_end; addr += W25Q128_SECTOR_SIZE) {
          HAL_StatusTypeDef ret = W25Q128_SectorErase(&flash, addr);
          if (ret != HAL_OK) {
  #ifdef DEBUG
              printf("Chip erase failed at 0x%06lX\r\n", addr);
  #endif
              return ret;
          }
      }

      flash_write_addr = FLASH_LOG_START_ADDR;
      flash_record_count = 0;

  #ifdef DEBUG
      printf("Flash chip erase complete\r\n");
  #endif
      return HAL_OK;
  }

#endif /* FLASH_ERASE_BUILD */