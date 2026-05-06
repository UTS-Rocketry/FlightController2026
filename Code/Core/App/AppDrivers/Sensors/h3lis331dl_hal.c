#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "h3lis331dl_reg.h"
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
// static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);

static stmdev_ctx_t dev_ctx;


HAL_StatusTypeDef h3lis331dl_init(h3lis331dl_HandleTypeDef *h3)
{
  h3lis331dl_int1_on_th_conf_t int_route;
  int32_t resultINT = 0;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = h3;
  

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */


  whoamI = 0;
  resultINT = h3lis331dl_device_id_get(&dev_ctx, &whoamI);  
  if (resultINT != 0){

    printf("error at whoami 1 ");
    return HAL_ERROR;
  }

  if ( whoamI != H3LIS331DL_ID )
  {
    printf("error at whoami 2 ");
    return HAL_ERROR;
  }

  uint8_t boot = 0x80;
  h3lis331dl_write_reg(&dev_ctx, H3LIS331DL_CTRL_REG2, &boot, 1);
  HAL_Delay(10);


  /* Set full scale */
  resultINT = h3lis331dl_full_scale_set(&dev_ctx, H3LIS331DL_200g);
   if (resultINT != 0){
    printf("set full scale");
    return HAL_ERROR;
  }

  /* Configure filtering chain */
  resultINT = h3lis331dl_hp_path_set(&dev_ctx, H3LIS331DL_HP_DISABLE);
   if (resultINT != 0){
      
    printf("HP");

    return HAL_ERROR;
  }

  /* Set Output Data Rate */
  resultINT = h3lis331dl_data_rate_set(&dev_ctx, H3LIS331DL_ODR_100Hz);
   if (resultINT != 0){
        printf("Data Rate");

    return HAL_ERROR;
  }

  HAL_Delay(100);

  return HAL_OK;

 
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  h3lis331dl_HandleTypeDef *h3 = (h3lis331dl_HandleTypeDef*)handle;
  
  reg &= ~0x80; 
 
  HAL_GPIO_WritePin(h3->cs_port, h3->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(h3->hspi, &reg, 1, 1000);
  HAL_SPI_Transmit(h3->hspi, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(h3->cs_port, h3->cs_pin, GPIO_PIN_SET);

  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{

  h3lis331dl_HandleTypeDef *h3 = (h3lis331dl_HandleTypeDef*)handle;
  reg |= 0xC0;
  
  HAL_GPIO_WritePin(h3->cs_port, h3->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(h3->hspi, &reg, 1, 1000);
  HAL_SPI_Receive(h3->hspi, bufp, len, 1000);
  HAL_GPIO_WritePin(h3->cs_port, h3->cs_pin, GPIO_PIN_SET);

  return 0;
}

static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

HAL_StatusTypeDef h3lis331dl_externalRead(int16_t *val) {

  uint8_t status = 0;
    
    // wait for data ready
    do {
        HAL_Delay(1);
        h3lis331dl_read_reg(&dev_ctx, H3LIS331DL_STATUS_REG, &status, 1);
    } while (!(status & 0x08));  // wait for ZYXDA bit
    
    int32_t result = h3lis331dl_acceleration_raw_get(&dev_ctx, val);
    
    if (result != 0) return HAL_ERROR;
    return HAL_OK;

}

HAL_StatusTypeDef h3lis331dl_Calibration(float *offset) {

  uint8_t status = 0;
  int16_t buff[3];
  int32_t result = 0;
  int x = 0;
  
  int32_t val[3] = {0};
  
  for (x = 0; x < 100; x++) {

    do {
        HAL_Delay(1);
        h3lis331dl_read_reg(&dev_ctx, H3LIS331DL_STATUS_REG, &status, 1);
    } while (!(status & 0x08));  // wait for ZYXDA bit
    
    result = h3lis331dl_acceleration_raw_get(&dev_ctx, buff);
    if (result != 0) return HAL_ERROR;

    val[0] += buff[0];
    val[1] += buff[1];
    val[2] += buff[2];  

  }

  offset[0] = h3lis331dl_from_fs200_to_mg((int16_t)(val[0] / 100)) - 0.0f;
  offset[1] = h3lis331dl_from_fs200_to_mg((int16_t)(val[1] / 100)) - 0.0f;
  offset[2] = h3lis331dl_from_fs200_to_mg((int16_t)(val[2] / 100)) - 1000.0f;


  return HAL_OK;

}