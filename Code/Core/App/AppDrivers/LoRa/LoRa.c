#include "LoRa.h"
#include "main.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

/*GLOBAL Variable**********************************************************************************/

static LORA_CONFIG_TYPEDEF lora_1;
static SX1276_HandleTypedef sx1;

typedef enum {
    LORA_OPERATION_IDLE = 0,
    LORA_OPERATION_TX,
    LORA_OPERATION_RX
} LoraOperation;

static LoraOperation lora_operation = LORA_OPERATION_IDLE;
static volatile uint8_t lora_dio0_pending = 0U;
static HAL_StatusTypeDef lora_last_status = HAL_OK;
static uint32_t lora_operation_started_ms = 0U;
static uint32_t lora_operation_timeout_ms = 0U;
static uint8_t *lora_rx_buffer = NULL;
static uint8_t *lora_rx_length = NULL;
static uint8_t lora_rx_max_length = 0U;

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
static uint8_t lora_take_dio0_event(void);
static HAL_StatusTypeDef lora_finish_operation(HAL_StatusTypeDef status);

/*FUNCTIONS **************************************************************************************/

/*Initialization*/

HAL_StatusTypeDef lora_init(SX1276_HandleTypedef *sx1276, const LORA_CONFIG_TYPEDEF *lora_config) {

    HAL_StatusTypeDef result;
    /* checks if the sx1276 struct is null this comes from lora app init it initializes both 
       structs */
    if(sx1276 == NULL) return HAL_ERROR;
    if(lora_config == NULL) return HAL_ERROR;
    
    /* This passes the values into static local variable as once that information is set it will
       be used for the entire program */
    sx1 = *sx1276;
    lora_1 = *lora_config;
    lora_operation = LORA_OPERATION_IDLE;
    (void)lora_take_dio0_event();
    lora_last_status = HAL_OK;
    lora_rx_buffer = NULL;
    lora_rx_length = NULL;
    lora_rx_max_length = 0U;



    /*Hardware reset required according to the data sheet
      delays are ok here becuase not timing critical */
    HAL_GPIO_WritePin(sx1.rst_port , sx1.rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sx1.rst_port , sx1.rst_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    
    /*checks address return value is valis*/
    uint8_t buffer;
    result = platform_read(&sx1,REG_VERSION,&buffer,1);
    
    if(result != HAL_OK) {
        return HAL_ERROR;
    }

    if(buffer != 0x12) {
        return HAL_ERROR;
    }


    /* Set LoRa Mode */
    /* these masks dont overlap with eachother so you can combine them */
    buffer = MODE_LONG_RANGE | MODE_SLEEP;
    /*Spi platform write external funcion*/
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    /*Set FIFO to bottom, this is done as we need to set the first location of the FIFO
      buffer so it at the correct location and we dont have undefined behavior */
    buffer = 0x00;

    /* we do this for the RX and the TX FIFO buffer */
    result = platform_write(&sx1, REG_FIFO_RX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    result = platform_write(&sx1, REG_FIFO_TX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;


    /* Set LORA FREQUENCY, from the struct */
    result = lora_set_frequency(lora_1.frequency);
    if(result != HAL_OK) return HAL_ERROR;
    
    
    /* Setting LNA gain and boost, the value is the standard high sensitifity RX on the sx1276 for 915MHz */
    buffer = 0x23;
    result = platform_write(&sx1,REG_LNA, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;


    /* setting bandwith coding rate and implicit header from the struct */
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
  if(result == HAL_OK) {
    result = HAL_SPI_Transmit(sx1276->hspi, (uint8_t*) bufp, len, 1000);
  }
  
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
  if(result == HAL_OK) {
    result = HAL_SPI_Receive(sx1276->hspi, bufp, len, 1000);
  }
  
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
  if(result == HAL_OK) {
    result = HAL_SPI_Transmit(sx1276->hspi, bufp, len, 1000);
  }
  
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

HAL_StatusTypeDef lora_TX(const uint8_t *data, uint8_t length, uint32_t timeout_ms) {

    HAL_StatusTypeDef result;
    uint8_t buffer;

    if ((data == NULL) || (length == 0U)) return HAL_ERROR;
    if (lora_operation != LORA_OPERATION_IDLE) return HAL_BUSY;

    /* Force a known start mode so a previous timeout cannot wedge the radio. */
    result = lora_standby();
    if(result != HAL_OK) return result;

    /* Set DIO 0 to tx done*/
    buffer = 0x40;
    result = platform_write(&sx1, REG_DIO_MAPPING_1, &buffer, 1);
    if(result != HAL_OK) return result;

    /* Clear stale flags before DIO0 is armed for this operation. */
    buffer = 0xFF;
    result = platform_write(&sx1, REG_IRQ_FLAGS, &buffer, 1);
    if(result != HAL_OK) return result;

    /* write poiter of tx base addres to ptr addr */
    result = platform_read(&sx1, REG_FIFO_TX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return result;
    result = platform_write( &sx1, REG_FIFO_ADDR_PTR, &buffer, 1);
    if(result != HAL_OK) return result;

    result = platform_write_FIFO(&sx1, (uint8_t*)data, length);
    if(result != HAL_OK) return result;

    buffer = length;
    result = platform_write(&sx1, REG_PAYLOAD_LENGTH, &buffer, 1);
    if(result != HAL_OK) return result;

    (void)lora_take_dio0_event();
    lora_operation_started_ms = HAL_GetTick();
    lora_operation_timeout_ms = timeout_ms;
    lora_last_status = HAL_BUSY;
    lora_operation = LORA_OPERATION_TX;

    buffer = MODE_LONG_RANGE | MODE_TRANSMIT;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if(result != HAL_OK) {
        lora_operation = LORA_OPERATION_IDLE;
        lora_last_status = result;
        return result;
    }

    return HAL_OK;


}

HAL_StatusTypeDef lora_RX(uint8_t *buff, uint8_t *rx_length, uint8_t max_length, uint32_t timeout_ms) {

    HAL_StatusTypeDef result;
    uint8_t buffer;

    if ((buff == NULL) || (rx_length == NULL) || (max_length == 0U)) return HAL_ERROR;
    if (lora_operation != LORA_OPERATION_IDLE) return HAL_BUSY;

    *rx_length = 0U;

    /* Force a known start mode so TX and RX cannot inherit each other's mode. */
    result = lora_standby();
    if(result != HAL_OK) return result;

    /* Set DIO 0 to rx done*/
    buffer = 0x00;
    result = platform_write(&sx1, REG_DIO_MAPPING_1, &buffer, 1);
    if(result != HAL_OK) return result;

    /* Clear IRQ flags */
    buffer = 0xFF;
    result = platform_write(&sx1, REG_IRQ_FLAGS, &buffer, 1);
    if(result != HAL_OK) return result;

    /* write poiter of rx base addres to ptr addr */
    result = platform_read(&sx1, REG_FIFO_RX_BASE_ADDR, &buffer, 1);
    if(result != HAL_OK) return result;
    result = platform_write( &sx1, REG_FIFO_ADDR_PTR, &buffer, 1);
    if(result != HAL_OK) return result;

    (void)lora_take_dio0_event();
    lora_rx_buffer = buff;
    lora_rx_length = rx_length;
    lora_rx_max_length = max_length;
    lora_operation_started_ms = HAL_GetTick();
    lora_operation_timeout_ms = timeout_ms;
    lora_last_status = HAL_BUSY;
    lora_operation = LORA_OPERATION_RX;

    buffer = MODE_LONG_RANGE | MODE_RX_SINGLE;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if(result != HAL_OK) {
        lora_operation = LORA_OPERATION_IDLE;
        lora_last_status = result;
        lora_rx_buffer = NULL;
        lora_rx_length = NULL;
        lora_rx_max_length = 0U;
        return result;
    }

    return HAL_OK;


}

void lora_dio0_irq_handler(void) {
    lora_dio0_pending = 1U;
}

static uint8_t lora_take_dio0_event(void) {
    uint8_t pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    pending = lora_dio0_pending;
    lora_dio0_pending = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return pending;
}

static HAL_StatusTypeDef lora_finish_operation(HAL_StatusTypeDef status) {
    uint8_t buffer = MODE_LONG_RANGE | MODE_STAND_BY;
    HAL_StatusTypeDef cleanup_status;

    cleanup_status = platform_write(&sx1, REG_OPMODE, &buffer, 1);
    if ((cleanup_status != HAL_OK) && (status == HAL_OK)) {
        status = cleanup_status;
    }

    buffer = 0xFF;
    cleanup_status = platform_write(&sx1, REG_IRQ_FLAGS, &buffer, 1);
    if ((cleanup_status != HAL_OK) && (status == HAL_OK)) {
        status = cleanup_status;
    }

    lora_operation = LORA_OPERATION_IDLE;
    lora_last_status = status;
    lora_rx_buffer = NULL;
    lora_rx_length = NULL;
    lora_rx_max_length = 0U;

    return status;
}

HAL_StatusTypeDef lora_service(void) {
    HAL_StatusTypeDef result;
    uint8_t irq_flags;
    uint8_t buffer;

    if (lora_operation == LORA_OPERATION_IDLE) {
        return lora_last_status;
    }

    if (lora_take_dio0_event() != 0U) {
        result = platform_read(&sx1, REG_IRQ_FLAGS, &irq_flags, 1);
        if (result != HAL_OK) {
            return lora_finish_operation(result);
        }

        if (lora_operation == LORA_OPERATION_TX) {
            if ((irq_flags & IRQ_TX_DONE_MASK) != 0U) {
                return lora_finish_operation(HAL_OK);
            }
            return lora_finish_operation(HAL_ERROR);
        }

        if ((irq_flags & IRQ_RX_DONE_MASK) == 0U) {
            return lora_finish_operation(HAL_ERROR);
        }

        if ((irq_flags & IRQ_PAYLOAD_CRC_ERR_MASK) != 0U) {
            return lora_finish_operation(HAL_ERROR);
        }

        result = platform_read(&sx1, REG_RX_NB_BYTES, &buffer, 1);
        if (result != HAL_OK) {
            return lora_finish_operation(result);
        }

#ifdef DEBUG
        printf("RX_NB_BYTES = %d\r\n", buffer);
#endif

        if (buffer > lora_rx_max_length) {
            return lora_finish_operation(HAL_ERROR);
        }
        *lora_rx_length = buffer;

        result = platform_read(&sx1, REG_FIFO_RX_CURRENT_ADDR, &buffer, 1);
        if (result != HAL_OK) {
            return lora_finish_operation(result);
        }
        result = platform_write(&sx1, REG_FIFO_ADDR_PTR, &buffer, 1);
        if (result != HAL_OK) {
            return lora_finish_operation(result);
        }

        result = platform_read(&sx1, REG_FIFO, lora_rx_buffer, *lora_rx_length);
        if (result != HAL_OK) {
            return lora_finish_operation(result);
        }

        return lora_finish_operation(HAL_OK);
    }

    if ((HAL_GetTick() - lora_operation_started_ms) >= lora_operation_timeout_ms) {
        return lora_finish_operation(HAL_TIMEOUT);
    }

    return HAL_BUSY;
}

uint8_t lora_is_busy(void) {
    return (lora_operation != LORA_OPERATION_IDLE) ? 1U : 0U;
}

HAL_StatusTypeDef lora_get_status(void) {
    if (lora_operation != LORA_OPERATION_IDLE) {
        return HAL_BUSY;
    }
    return lora_last_status;
}

HAL_StatusTypeDef lora_sleep() {

    HAL_StatusTypeDef result;
    
    uint8_t buffer;
    
    buffer = MODE_LONG_RANGE | MODE_SLEEP;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);

    if(result != HAL_OK) return HAL_ERROR;

    return result;
    

}

HAL_StatusTypeDef lora_standby() {

    HAL_StatusTypeDef result;
    uint8_t buffer;
    
    buffer = MODE_LONG_RANGE | MODE_STAND_BY;
    result = platform_write(&sx1, REG_OPMODE, &buffer, 1);

    if(result != HAL_OK) return HAL_ERROR;

    return result;


}


HAL_StatusTypeDef lora_packet_rssi(int16_t *dbm) {

    HAL_StatusTypeDef result;
    uint8_t buffer;

    result = platform_read(&sx1, REG_PKT_RSSI_VALUE, &buffer, 1);
    if(result != HAL_OK) return HAL_ERROR;

    if (dbm == NULL) return HAL_ERROR;


    *dbm = - 157 + buffer;

    return result;
    

} /*returns dBm*/


HAL_StatusTypeDef lora_version(uint8_t *lora_version) {

    HAL_StatusTypeDef result;

    result = platform_read(&sx1, REG_VERSION, lora_version, 1);
    if(result != HAL_OK) return HAL_ERROR;
    if(lora_version == NULL) return HAL_ERROR;

    return result;

} /*should return 0x12*/

HAL_StatusTypeDef lora_packet_snr(float *snr) {

    HAL_StatusTypeDef result;
    uint8_t buffer;

    if (snr == NULL) return HAL_ERROR;

    result = platform_read(&sx1, REG_PKT_SNR_VALUE, &buffer, 1);

    if(result != HAL_OK) return HAL_ERROR;

    /* The register is an 8-bit two's-complement value in 0.25 dB units. */
    *snr = (float)(int8_t)buffer / 4.0f;

    return result;

}
