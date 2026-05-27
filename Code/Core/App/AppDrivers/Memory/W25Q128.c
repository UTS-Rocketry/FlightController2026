#include "W25Q128.h"

/* ── CS helpers ──────────────────────────────────────────────────── */

static inline void CS_Low(W25Q128_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void CS_High(W25Q128_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ── Low-level SPI transfer ──────────────────────────────────────── */

static HAL_StatusTypeDef SPI_Transmit(W25Q128_Handle_t *dev, uint8_t *tx, uint16_t len)
{
    return HAL_SPI_Transmit(dev->hspi, tx, len, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef SPI_Receive(W25Q128_Handle_t *dev, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_Receive(dev->hspi, rx, len, HAL_MAX_DELAY);
}

/* ── Status register ─────────────────────────────────────────────── */

uint8_t W25Q128_ReadStatusReg1(W25Q128_Handle_t *dev)
{
    uint8_t cmd = W25Q_READ_STATUS_REG1;
    uint8_t status = 0;

    CS_Low(dev);
    SPI_Transmit(dev, &cmd, 1);
    SPI_Receive(dev, &status, 1);
    CS_High(dev);

    return status;
}

/* Polls BUSY bit until clear or timeout */
HAL_StatusTypeDef W25Q128_WaitBusy(W25Q128_Handle_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (W25Q128_ReadStatusReg1(dev) & W25Q_SR1_BUSY) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* ── Write Enable / Disable ──────────────────────────────────────── */

static HAL_StatusTypeDef WriteEnable(W25Q128_Handle_t *dev)
{
    uint8_t cmd = W25Q_WRITE_ENABLE;
    CS_Low(dev);
    HAL_StatusTypeDef ret = SPI_Transmit(dev, &cmd, 1);
    CS_High(dev);
    return ret;
}

/* ── Init ────────────────────────────────────────────────────────── */

HAL_StatusTypeDef W25Q128_Init(W25Q128_Handle_t *dev)
{
    /* Ensure CS is deasserted at startup */
    CS_High(dev);
    HAL_Delay(5);  /* tVSL: device powers up within 5ms */

    /* Release from power-down in case it was left in that state */
    W25Q128_ReleasePowerDown(dev);
    HAL_Delay(1);

    /* Verify JEDEC ID */
    uint8_t manuf, mem_type, capacity;
    HAL_StatusTypeDef ret = W25Q128_ReadJEDECID(dev, &manuf, &mem_type, &capacity);
    if (ret != HAL_OK) return ret;

    if (manuf != W25Q128_JEDEC_MANUF ||
        mem_type != W25Q128_JEDEC_MEM_TYPE ||
        capacity != W25Q128_JEDEC_CAPACITY) {
        return HAL_ERROR;  /* Wrong device or no response */
    }

    return HAL_OK;
}

/* ── JEDEC ID ────────────────────────────────────────────────────── */

HAL_StatusTypeDef W25Q128_ReadJEDECID(W25Q128_Handle_t *dev,
                                       uint8_t *manuf,
                                       uint8_t *mem_type,
                                       uint8_t *capacity)
{
    uint8_t cmd = W25Q_JEDEC_ID;
    uint8_t rx[3] = {0};

    CS_Low(dev);
    HAL_StatusTypeDef ret = SPI_Transmit(dev, &cmd, 1);
    if (ret == HAL_OK) ret = SPI_Receive(dev, rx, 3);
    CS_High(dev);

    if (ret == HAL_OK) {
        *manuf     = rx[0];
        *mem_type  = rx[1];
        *capacity  = rx[2];
    }
    return ret;
}

/* ── Read Data ───────────────────────────────────────────────────── */

HAL_StatusTypeDef W25Q128_ReadData(W25Q128_Handle_t *dev,
                                    uint32_t addr,
                                    uint8_t *buf,
                                    uint32_t len)
{
    /* READ command: 1 byte cmd + 3 bytes 24-bit address */
    uint8_t cmd[4] = {
        W25Q_READ_DATA,
        (addr >> 16) & 0xFF,
        (addr >>  8) & 0xFF,
        (addr >>  0) & 0xFF
    };

    CS_Low(dev);
    HAL_StatusTypeDef ret = SPI_Transmit(dev, cmd, 4);
    if (ret == HAL_OK) ret = SPI_Receive(dev, buf, (uint16_t)len);
    CS_High(dev);

    return ret;
}

/* ── Page Program ────────────────────────────────────────────────── */
/*
 * Writes up to 256 bytes. addr must be page-aligned, or at minimum
 * addr+len must not cross a 256-byte page boundary — the device
 * wraps around if it does. Caller is responsible for alignment.
 */
HAL_StatusTypeDef W25Q128_PageProgram(W25Q128_Handle_t *dev,
                                       uint32_t addr,
                                       uint8_t *buf,
                                       uint16_t len)
{
    if (len == 0 || len > W25Q128_PAGE_SIZE) return HAL_ERROR;

    /* Must be write-enabled before every program/erase */
    HAL_StatusTypeDef ret = WriteEnable(dev);
    if (ret != HAL_OK) return ret;

    uint8_t cmd[4] = {
        W25Q_PAGE_PROGRAM,
        (addr >> 16) & 0xFF,
        (addr >>  8) & 0xFF,
        (addr >>  0) & 0xFF
    };

    CS_Low(dev);
    ret = SPI_Transmit(dev, cmd, 4);
    if (ret == HAL_OK) ret = SPI_Transmit(dev, buf, len);
    CS_High(dev);

    if (ret != HAL_OK) return ret;

    /* Page program takes up to 3ms (tPP); wait for BUSY to clear */
    return W25Q128_WaitBusy(dev, 10);
}

/* ── Sector Erase (4KB) ──────────────────────────────────────────── */
/*
 * Erases the 4KB sector containing addr. All bytes in the sector
 * are set to 0xFF. addr is masked to sector boundary internally.
 * Takes up to 400ms (tSE).
 */
HAL_StatusTypeDef W25Q128_SectorErase(W25Q128_Handle_t *dev, uint32_t addr)
{
    /* Align to sector boundary */
    addr &= ~(W25Q128_SECTOR_SIZE - 1);

    HAL_StatusTypeDef ret = WriteEnable(dev);
    if (ret != HAL_OK) return ret;

    uint8_t cmd[4] = {
        W25Q_SECTOR_ERASE_4K,
        (addr >> 16) & 0xFF,
        (addr >>  8) & 0xFF,
        (addr >>  0) & 0xFF
    };

    CS_Low(dev);
    ret = SPI_Transmit(dev, cmd, 4);
    CS_High(dev);

    if (ret != HAL_OK) return ret;

    return W25Q128_WaitBusy(dev, 500);  /* tSE max 400ms */
}

/* ── Chip Erase ──────────────────────────────────────────────────── */
/* Takes up to 200 seconds on a full 16MB chip. Don't call on a flight. */

HAL_StatusTypeDef W25Q128_ChipErase(W25Q128_Handle_t *dev)
{
    HAL_StatusTypeDef ret = WriteEnable(dev);
    if (ret != HAL_OK) return ret;

    uint8_t cmd = W25Q_CHIP_ERASE;
    CS_Low(dev);
    ret = SPI_Transmit(dev, &cmd, 1);
    CS_High(dev);

    if (ret != HAL_OK) return ret;

    return W25Q128_WaitBusy(dev, 200000);  /* 200s max */
}

/* ── Power management ────────────────────────────────────────────── */

HAL_StatusTypeDef W25Q128_PowerDown(W25Q128_Handle_t *dev)
{
    uint8_t cmd = W25Q_POWER_DOWN;
    CS_Low(dev);
    HAL_StatusTypeDef ret = SPI_Transmit(dev, &cmd, 1);
    CS_High(dev);
    HAL_Delay(1);  /* tDP: 3µs typical, 1ms is conservative */
    return ret;
}

HAL_StatusTypeDef W25Q128_ReleasePowerDown(W25Q128_Handle_t *dev)
{
    uint8_t cmd = W25Q_RELEASE_POWER_DOWN;
    CS_Low(dev);
    HAL_StatusTypeDef ret = SPI_Transmit(dev, &cmd, 1);
    CS_High(dev);
    HAL_Delay(1);  /* tRES1: 3µs typical */
    return ret;
}