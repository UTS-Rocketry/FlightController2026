#ifndef W25Q128_H
#define W25Q128_H

#include "stm32f4xx_hal.h"

/* W25Q128 command set */
#define W25Q_WRITE_ENABLE       0x06
#define W25Q_WRITE_DISABLE      0x04
#define W25Q_READ_STATUS_REG1   0x05
#define W25Q_READ_STATUS_REG2   0x35
#define W25Q_WRITE_STATUS_REG   0x01
#define W25Q_PAGE_PROGRAM       0x02
#define W25Q_SECTOR_ERASE_4K    0x20
#define W25Q_BLOCK_ERASE_64K    0xD8
#define W25Q_CHIP_ERASE         0xC7
#define W25Q_READ_DATA          0x03
#define W25Q_FAST_READ          0x0B
#define W25Q_JEDEC_ID           0x9F
#define W25Q_POWER_DOWN         0xB9
#define W25Q_RELEASE_POWER_DOWN 0xAB

/* Status register bits */
#define W25Q_SR1_BUSY           0x01   /* Bit 0: device busy */
#define W25Q_SR1_WEL            0x02   /* Bit 1: write enable latch */

/* Device geometry */
#define W25Q128_PAGE_SIZE       256    /* bytes */
#define W25Q128_SECTOR_SIZE     4096   /* bytes (4KB) */
#define W25Q128_BLOCK_SIZE      65536  /* bytes (64KB) */
#define W25Q128_TOTAL_SIZE      (16 * 1024 * 1024)  /* 16MB */
#define W25Q128_NUM_SECTORS     4096
#define W25Q128_NUM_PAGES       65536

/* Expected JEDEC ID for W25Q128JV */
#define W25Q128_JEDEC_MANUF     0xEF   /* Winbond */
#define W25Q128_JEDEC_MEM_TYPE  0x40
#define W25Q128_JEDEC_CAPACITY  0x18

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} W25Q128_Handle_t;

/* Initialization & identity */
HAL_StatusTypeDef W25Q128_Init(W25Q128_Handle_t *dev);
HAL_StatusTypeDef W25Q128_ReadJEDECID(W25Q128_Handle_t *dev, uint8_t *manuf, uint8_t *mem_type, uint8_t *capacity);

/* Core operations */
HAL_StatusTypeDef W25Q128_ReadData(W25Q128_Handle_t *dev, uint32_t addr, uint8_t *buf, uint32_t len);
HAL_StatusTypeDef W25Q128_PageProgram(W25Q128_Handle_t *dev, uint32_t addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef W25Q128_SectorErase(W25Q128_Handle_t *dev, uint32_t addr);
HAL_StatusTypeDef W25Q128_ChipErase(W25Q128_Handle_t *dev);

/* Power management */
HAL_StatusTypeDef W25Q128_PowerDown(W25Q128_Handle_t *dev);
HAL_StatusTypeDef W25Q128_ReleasePowerDown(W25Q128_Handle_t *dev);

/* Utility */
uint8_t W25Q128_ReadStatusReg1(W25Q128_Handle_t *dev);
HAL_StatusTypeDef W25Q128_WaitBusy(W25Q128_Handle_t *dev, uint32_t timeout_ms);

#endif /* W25Q128_H */