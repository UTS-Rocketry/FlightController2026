#include "lsm6dsox_reg.h"
#include <stdio.h>

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

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

  if (resultINT != 0) {
    return HAL_ERROR;
  }

  if (whoamI != LSM6DSOX_ID) {
    return HAL_ERROR;
  }

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  uint32_t last = HAL_GetTick();
  do {
      lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst && (HAL_GetTick() - last) < 100);

  if (rst) return HAL_ERROR;

  /* Disable I3C interface */
  resultINT = lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  if (resultINT != 0) {
    return HAL_ERROR;
  }

  /* Set XL Output Data Rate to 416 Hz */
  resultINT = lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_104Hz);
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
  resultINT = lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_104Hz);
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
  HAL_StatusTypeDef result;
  reg &= ~0x80;
 
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(l6->hspi, &reg, 1, 1000);
  if (result != 0) {
    HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);
    return 1;
  }
  result = HAL_SPI_Transmit(l6->hspi, (uint8_t*) bufp, len, 1000);
  if (result != 0) {
    HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);
    return 1;
  }
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);

  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  HAL_StatusTypeDef result;
  lsm6dso_HandleTypedef *l6 = (lsm6dso_HandleTypedef*)handle;
  reg |= 0x80;
  
  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(l6->hspi, &reg, 1, 1000);
  
  if (result != 0) {
    HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);
    return 1;
  } 

  result = HAL_SPI_Receive(l6->hspi, bufp, len, 1000);
  
  if (result != 0) {
    HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);
    return 1;
  }

  HAL_GPIO_WritePin(l6->cs_port, l6->cs_pin, GPIO_PIN_SET);

  return 0;

}

static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* this function calibrates the offset to be used agains the sensor gyro and accelerometer readings */
HAL_StatusTypeDef lsm6dso_Calib(float *xl_Offset, float *gy_Offset) {

  lsm6dsox_status_reg_t status = {0};
  int16_t xl_Buff[3];
  int16_t gy_Buff[3];
  int32_t result = 0;
  int x = 0;
  
  int32_t xl_Store[3] = {0};
  int32_t gy_Store[3] = {0};
  
  /* Take average over 100 readings */
  for (x = 0; x < 100; x++) {
    
    /* this is sa blocking function however if it doesnt start it is a nogo as our primary
       functions rely on the lsm */
    do {
      lsm6dsox_status_reg_get(&dev_ctx, &status );
    } while(!status.xlda || !status.gda);

    /* Raw getter for accelereomter*/
    result = lsm6dsox_acceleration_raw_get(&dev_ctx, xl_Buff);
    if (result != 0) return HAL_ERROR;
    
    /* raw getter for the gyro*/
    result = lsm6dsox_angular_rate_raw_get(&dev_ctx, gy_Buff);
    if (result != 0) return HAL_ERROR;

    /* these arrays store the average values of the offset they will then be divided by 100 to get accual offset */
    xl_Store[0] += xl_Buff[0];
    xl_Store[1] += xl_Buff[1];
    xl_Store[2] += xl_Buff[2];

    gy_Store[0] += gy_Buff[0];
    gy_Store[1] += gy_Buff[1];
    gy_Store[2] += gy_Buff[2];

  }

  /* driver provided functions to convert to float*/
  xl_Offset[0] = lsm6dsox_from_fs16_to_mg((int16_t)(xl_Store[0] / 100)) - 0.0f;
  xl_Offset[1] = lsm6dsox_from_fs16_to_mg((int16_t)(xl_Store[1] / 100)) - 0.0f;
  xl_Offset[2] = lsm6dsox_from_fs16_to_mg((int16_t)(xl_Store[2] / 100)) - 1000.0f;
  
  gy_Offset[0] =lsm6dsox_from_fs2000_to_mdps((int16_t)(gy_Store[0]/ 100));
  gy_Offset[1] =lsm6dsox_from_fs2000_to_mdps((int16_t)(gy_Store[1]/ 100));
  gy_Offset[2] =lsm6dsox_from_fs2000_to_mdps((int16_t)(gy_Store[2]/ 100));



  return HAL_OK;
  
}

HAL_StatusTypeDef lsm6dso_ExternalReader(int16_t *xl_Val, int16_t *gy_Val) {

  lsm6dsox_status_reg_t status = {0};
  int32_t result = 0;

  uint32_t last = HAL_GetTick();
	uint32_t timeout;
    
    do {
      
      uint32_t now = HAL_GetTick();
		  timeout = now - last;
      lsm6dsox_status_reg_get(&dev_ctx, &status );

    } while((!status.xlda || !status.gda) && timeout < 50);
    if (!status.xlda || !status.gda) return HAL_TIMEOUT;

    result = lsm6dsox_acceleration_raw_get(&dev_ctx, xl_Val);
    if (result != 0) return HAL_ERROR;
    
    result = lsm6dsox_angular_rate_raw_get(&dev_ctx, gy_Val);
    if (result != 0) return HAL_ERROR;

    return  HAL_OK;

}
