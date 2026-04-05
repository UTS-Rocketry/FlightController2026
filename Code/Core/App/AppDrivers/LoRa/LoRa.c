#include "LoRa.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

/*GLOBAL Variable**********************************************************************************/

static LORA_CONFIG_TYPEDEF lora_1;
static SX1276_HandleTypedef sx1;

/*FUNCTION DEFS ***********************************************************************************/
/*PRIVATE DEFS ***********************************************************************************/

static HAL_StatusTypeDef platform_write(SX1276_HandleTypedef *sx1276, uint8_t reg, const uint8_t *bufp,
                                 uint16_t len);

static HAL_StatusTypeDef platform_read(SX1276_HandleTypedef *sx1276, uint8_t reg, uint8_t *bufp,
                                uint16_t len);

static HAL_StatusTypeDef platform_write_FIFO(SX1276_HandleTypedef *sx1276, uint8_t *bufp, uint16_t len);

static HAL_StatusTypeDef lora_set_frequency(uint32_t frequency);
static HAL_StatusTypeDef lora_set_modem_1(uint32_t bandwidth, uint8_t coding_rate, uint8_t implicit_header);
static HAL_StatusTypeDef lora_set_modem_2 (uint8_t spreading_factor, uint8_t enable_crc);
static HAL_StatusTypeDef lora_set_modem_3 (uint8_t spreading_factor, uint32_t bandwidth);
static HAL_StatusTypeDef lora_set_syncword(uint8_t syncword);
static HAL_StatusTypeDef lora_set_TX_power(uint8_t use_pa_boost, uint8_t tx_power);

/*FUNCTIONS **************************************************************************************/

/*Initialization*/

HAL_StatusTypeDef lora_init(SX1276_HandleTypedef *sx1276, const LORA_CONFIG_TYPEDEF *lora_config) {

    HAL_StatusTypeDef result;
    if(sx1276 == NULL) return HAL_ERROR;
    if(lora_config == NULL) return HAL_ERROR;
    
    sx1 = *sx1276;
    lora_1 = *lora_config;

    /*Hardware reset*/
    HAL_GPIO_WritePin(sx1.rst_port , sx1.rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sx1.rst_port , sx1.rst_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    /*checks address*/
    uint8_t buffer;
    result = platform_read(&sx1,&buffer,REG_VERSION,1);
    if(result != HAL_OK) return HAL_ERROR;
    if(buffer != 0x12) return HAL_ERROR;

    /* Set LoRa Mode */
    buffer = MODE_LONG_RANGE | MODE_SLEEP;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    /*Set FIFO to bottom*/
    buffer = 0x00;
    result = platform_write(&sx1, REG_FIFO_RX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    result = platform_write(&sx1, REG_FIFO_TX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    /* Set LORA FREQUENCY */
    result = lora_set_frequency(lora_1.frequency);
    if(result != HAL_OK) return HAL_ERROR;
    
    /* Setting LNA gain and boost */
    buffer = 0x23;
    result = platform_write(&sx1,REG_LNA, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    /* setting bandwith coding rate and implicit header */
    result = lora_set_modem_1(lora_1.bandwidth, lora_1.coding_Rate, lora_1.implicit_header);
    if(result != HAL_OK) return HAL_ERROR;

    /* setting spreading factor and crc */
    result = lora_set_modem_2(lora_1.spreading_Factor, lora_1.enable_CRC);
    if(result != HAL_OK) return HAL_ERROR;

    result = lora_set_modem_3 (lora_1.spreading_Factor, lora_1.bandwidth);
    if(result != HAL_OK) return HAL_ERROR;

    result = lora_set_syncword(lora_1.sync_Word);
    if(result != HAL_OK) return HAL_ERROR;

    buffer = lora_1.preamble_Length >> 8;
    result = platform_write(&sx1, REG_PREAMBLE_MSB, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    buffer = lora_1.preamble_Length & 0xFF;
    result = platform_write(&sx1, REG_PREAMBLE_LSB, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    result =  lora_set_TX_power(lora_1.use_Pa_Boost, lora_1.tx_Power);
    if(result != HAL_OK) return HAL_ERROR;

    /* Set LoRa Mode */
    buffer = MODE_LONG_RANGE | MODE_STAND_BY;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;


    return result;
}

/* write register */
static HAL_StatusTypeDef platform_write(SX1276_HandleTypedef *sx1276, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  
  HAL_StatusTypeDef result;

  if(sx1276 == NULL) return HAL_ERROR;

  reg |= 0x80;
 
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_RESET);
  
  result = HAL_SPI_Transmit(sx1276->hspi, &reg, 1, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  result = HAL_SPI_Transmit(sx1276->hspi, (uint8_t*) bufp, len, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_SET);
  
  return result;
  
}

/* read register */
static HAL_StatusTypeDef platform_read(SX1276_HandleTypedef *sx1276, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
   
  HAL_StatusTypeDef result;
  
  if(sx1276 == NULL) return HAL_ERROR;
  
  
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_RESET);
  
  result = HAL_SPI_Transmit(sx1276->hspi, &reg, 1, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  result = HAL_SPI_Receive(sx1276->hspi, bufp, len, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_SET);
  

  return result;

}

/* write FIFO register */
static HAL_StatusTypeDef platform_write_FIFO(SX1276_HandleTypedef *sx1276, uint8_t *bufp, uint16_t len)
{
   
  if(sx1276 == NULL) return HAL_ERROR;
  HAL_StatusTypeDef result;

  /*Sets reg to write*/
  uint8_t fifo_addr = REG_FIFO | 0x80;
  
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_RESET);
  
  result = HAL_SPI_Transmit(sx1276->hspi, &fifo_addr, 1, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  result = HAL_SPI_Transmit(sx1276->hspi, bufp, len, 1000);
  if(result != HAL_OK) return HAL_ERROR;
  
  HAL_GPIO_WritePin(sx1276->nss_port, sx1276->nss_pin, GPIO_PIN_SET);
  

  return result;

}

static HAL_StatusTypeDef lora_set_frequency(uint32_t frequency) {

    HAL_StatusTypeDef result;

    uint64_t freq = (uint64_t) frequency;
    
    uint32_t frf = (freq << 19) / 32000000;

    uint8_t buffer = frf >> 16 ;
    result = platform_write(&sx1, REG_FR_MSB, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    buffer = (frf >> 8) & 0xFF;
    result = platform_write(&sx1, REG_FR_MID, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    buffer = frf & 0xFF;
    result = platform_write(&sx1, REG_FR_LSB, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    return result;

}

static HAL_StatusTypeDef lora_set_modem_1(uint32_t bandwidth, uint8_t coding_rate, uint8_t implicit_header) {
    
    HAL_StatusTypeDef result;
    uint8_t config_byte = 0;
    uint8_t bw_code = 0;
    switch (bandwidth){
        case 7800:
            bw_code = 0;
            break;
        case 10400:
            bw_code = 1;
            break;
        case 15600:
            bw_code = 2;
            break;
        case 20800:
            bw_code = 3;
            break;
        case 31250:
            bw_code = 4;
            break;
        case 41700:
            bw_code = 5;
            break;
        case 62500:
            bw_code = 6;
            break;
        case 125000:
            bw_code = 7;
            break;
        case 250000:
            bw_code = 8;
            break;
        case 500000:
            bw_code = 9;
            break;
        default:
            return HAL_ERROR;
    }

    config_byte = bw_code << 4;

    config_byte |= (coding_rate - 4 ) << 1;

    config_byte |= implicit_header;

    result = platform_write(&sx1, REG_MODEM_CONFIG_1, &config_byte, 1);
    if(result != HAL_OK) return HAL_ERROR;

    return result;

        
}

static HAL_StatusTypeDef lora_set_modem_2 (uint8_t spreading_factor, uint8_t enable_crc) {
    HAL_StatusTypeDef result;

    uint8_t config_byte = 0;

    config_byte = spreading_factor << 4;

    /* Set normal tx mode is zero by default*/

    config_byte |= enable_crc << 2;

    result = platform_write(&sx1, REG_MODEM_CONFIG_2, &config_byte, 1);
    if(result != HAL_OK) return HAL_ERROR;

    return result;

}

static HAL_StatusTypeDef lora_set_modem_3 (uint8_t spreading_factor, uint32_t bandwidth) {
    
    HAL_StatusTypeDef result;

    uint8_t config_byte = 0;

    if (spreading_factor >= 11 && bandwidth <= 125000) {
        config_byte = 1 << 3;
    }
    
    /* Setting AGC auto on*/
    config_byte |= 1 << 2;

     result = platform_write(&sx1, REG_MODEM_CONFIG_3, &config_byte, 1);
    if(result != HAL_OK) return HAL_ERROR;

    return result;

   
}

static HAL_StatusTypeDef lora_set_syncword(uint8_t syncword) {
    HAL_StatusTypeDef result;

    uint8_t config_byte = syncword;

    result = platform_write(&sx1, REG_SYNC_WORD, &config_byte, 1);
    if(result != HAL_OK) return HAL_ERROR;

    return result;
}

static HAL_StatusTypeDef lora_set_TX_power(uint8_t use_pa_boost, uint8_t tx_power) {
    
    HAL_StatusTypeDef result;
    uint8_t config_byte = 0;

    if (use_pa_boost == 1) {
        
        uint8_t output_power = tx_power - 2;

        config_byte = 0x80;
        config_byte |= output_power; 

        result = platform_write(&sx1, REG_PA_CONFIG, &config_byte, 1);
        if(result != HAL_OK) return HAL_ERROR;

        if (tx_power == 20) {
            
            config_byte = 0X87;
            result = platform_write(&sx1, REG_PA_DAC, &config_byte, 1);
            if(result != HAL_OK) return HAL_ERROR;

            config_byte = 0X3B;
            result = platform_write(&sx1, REG_OCP, &config_byte, 1);
            if(result != HAL_OK) return HAL_ERROR;



        }
        else {
            config_byte = 0x84;
            result = platform_write(&sx1, REG_PA_DAC, &config_byte, 1);
            if(result != HAL_OK) return HAL_ERROR;

        }

        
    }

    else {

        uint8_t output_power =  tx_power - 1;
        uint8_t max_power = 7;

        config_byte = (max_power << 4) | output_power;
        
        result = platform_write(&sx1, REG_PA_CONFIG, &config_byte, 1);
        if(result != HAL_OK) return HAL_ERROR;

        config_byte = 0x84;
        result = platform_write(&sx1, REG_PA_DAC, &config_byte, 1);
        if(result != HAL_OK) return HAL_ERROR;

    
    }

    return result;


}