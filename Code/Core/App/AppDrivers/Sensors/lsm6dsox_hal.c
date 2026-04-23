#include "lsm6dsox_reg.h"
#include <stdio.h>

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* Defines           ---------------------------------------------------------*/
#define  BOOT_TIME     50 //ms



/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
//static uint8_t tx_buffer[1000];


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle , uint8_t reg, uint8_t *bufp,
                             uint16_t len);
//static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);


static stmdev_ctx_t dev_ctx;


HAL_StatusTypeDef lsm6dso_init(lsm6dso_HandleTypedef *l6)
{

  int32_t resultINT = 0;
  uint8_t rst = 0;
  
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = l6;
  
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  resultINT = lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  printf("resultINT: 0x%02X\r\n", whoamI);  // add this

  if (resultINT != 0) {
    return HAL_ERROR;
  }

  if (whoamI != LSM6DSOX_ID) {
    return HAL_ERROR;
  }

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  resultINT = lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  if (resultINT != 0) {
    return HAL_ERROR;
  }

  /* Set XL Output Data Rate to 416 Hz */
  resultINT = lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  if (resultINT != 0) {
    return HAL_ERROR;
  }

  /* Set 16g full XL scale */
  resultINT = lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_16g);
  if (resultINT != 0) {
    return HAL_ERROR;
  }

 

  /*Set gy to 2000 dps */
  resultINT = lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
  if (resultINT != 0) {
    return HAL_ERROR;
  }

  /* Set GY output to 416 hz aswell */
  resultINT = lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_417Hz);
  if (resultINT != 0) {
    return HAL_ERROR;
  }


  /* Low pass filter on the gyro */
  resultINT = lsm6dsox_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSOX_MEDIUM);
  if (resultINT != 0) {
    return HAL_ERROR;
  }
  

  return HAL_OK;
  
  
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  lsm6dso_HandleTypedef *l6 = (lsm6dso_HandleTypedef*)handle;

  BitSet(reg,7);
  //reg |= 0x80;
 
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(l6->hspi, &reg, 1, 1000);
  HAL_SPI_Transmit(l6->hspi, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);

  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{

  lsm6dso_HandleTypedef *l6 = (lsm6dso_HandleTypedef*)handle;
  reg |= 0xC0;
  
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(l6->hspi, &reg, 1, 1000);
  HAL_SPI_Receive(l6->hspi, bufp, len, 1000);
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);

  return 0;

}

static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}