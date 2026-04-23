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
  printf("resultINT: 0x%02X\r\n", whoamI);  // add this

  if (resultINT != 0){
    return HAL_ERROR;
  }

  if ( whoamI != H3LIS331DL_ID )
  {
    return HAL_ERROR;
  }

  /* Set full scale */
  resultINT = h3lis331dl_full_scale_set(&dev_ctx, H3LIS331DL_100g);
   if (resultINT != 0){
    return HAL_ERROR;
  }
  printf("full_scale: %ld\r\n", resultINT);

  /* Configure filtering chain */
  resultINT = h3lis331dl_hp_path_set(&dev_ctx, H3LIS331DL_HP_DISABLE);
   if (resultINT != 0){
    return HAL_ERROR;
  }

   printf("hp_path: %ld\r\n", resultINT);

  /* Set minimum event duration for wakeup */
  resultINT = h3lis331dl_int1_dur_set(&dev_ctx, 1);
   if (resultINT != 0){
    return HAL_ERROR;
  }
  printf("int1_dur: %ld\r\n", resultINT);
  /*
   * Apply wakeup axis threshold (lsb is FS/128)
   */
  resultINT = h3lis331dl_int1_threshold_set(&dev_ctx, 3);
   if (resultINT != 0){
    return HAL_ERROR;
  }

  resultINT = h3lis331dl_int1_on_threshold_mode_set(&dev_ctx, H3LIS331DL_INT1_ON_THRESHOLD_OR);
   if (resultINT != 0){
    return HAL_ERROR;
  }

  resultINT = h3lis331dl_int1_on_threshold_conf_get(&dev_ctx, &int_route);
   if (resultINT != 0){
    return HAL_ERROR;
  }
  int_route.int1_xhie = PROPERTY_ENABLE;
  int_route.int1_yhie = PROPERTY_ENABLE;
  int_route.int1_zhie = PROPERTY_ENABLE;
  resultINT = h3lis331dl_int1_on_threshold_conf_set(&dev_ctx, int_route);
   if (resultINT != 0){
    return HAL_ERROR;
  }
  resultINT = h3lis331dl_pin_int1_route_set(&dev_ctx, H3LIS331DL_PAD1_INT1_SRC);
   if (resultINT != 0){
    return HAL_ERROR;
  }
  resultINT = h3lis331dl_int1_notification_set(&dev_ctx, H3LIS331DL_INT1_LATCHED);
   if (resultINT != 0){
    return HAL_ERROR;
  }

  /* Set Output Data Rate */
  resultINT = h3lis331dl_data_rate_set(&dev_ctx, H3LIS331DL_ODR_100Hz);
   if (resultINT != 0){
    return HAL_ERROR;
  }

  return HAL_OK;

 
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  h3lis331dl_HandleTypeDef *h3 = (h3lis331dl_HandleTypeDef*)handle;

  BitSet(reg,7);

  //reg |= 0x80;
 
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