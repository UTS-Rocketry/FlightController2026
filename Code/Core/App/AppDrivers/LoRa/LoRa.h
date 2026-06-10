#ifndef  LORA_H
#define LORA_H

#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*REGISTER DEFINES *******************************************************************************/

#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01

#define REG_FR_MSB                  0x06
#define REG_FR_MID                  0x07
#define REG_FR_LSB                  0x08

#define REG_PA_CONFIG               0x09
#define REG_PA_RAMP                 0x0A

/* Overload Current protection */
#define REG_OCP                     0x0B

#define REG_LNA                     0x0C

#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10

#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12

#define REG_RX_NB_BYTES             0x13
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15

#define REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17

#define REG_MODEM_STAT              0x18

#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_RSSI_VALUE              0x1B

#define REG_HOP_CHANNEL             0x1C

#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E

#define REG_SYMB_TIMEOUT_LSB        0x1F

#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21

#define REG_PAYLOAD_LENGTH          0x22
#define REG_MAX_PAYLOAD_LENGTH      0x23

#define REG_HOP_PERIOD              0x24
#define REG_FIFO_RX_BYTE_ADDR       0x25
#define REG_MODEM_CONFIG_3          0x26


/*FREQ ERROR*/
#define REG_FEI_MSB                 0x28
#define REG_FEI_MID                 0x29
#define REG_FEI_LSB                 0x2A


#define REG_RSSI_WIDEBAND           0x2C

#define REG_DETECT_OPTIMIZE         0x31
#define REG_INVERT_IQ               0x33
#define REG_DETECTION_THRESHOLD     0x37
#define REG_INVERT_IQ_2             0x3B


/*SYNCWORD TO WRITE IN INIT*/
#define REG_SYNC_WORD               0x39

#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42

#define REG_TCXO                    0x4B
#define REG_PA_DAC                  0x4D

/*BITMASKS ***************************************************************************************/

/*OPERATION MODES*/
#define MODE_LONG_RANGE             0x80
#define MODE_SLEEP                  0x00
#define MODE_STAND_BY               0x01
#define MODE_FSTX                   0x02
#define MODE_TRANSMIT               0x03
#define MODE_FSRX                   0x04
#define MODE_RX_CONTINUOUS          0x05
#define MODE_RX_SINGLE              0x06
#define MODE_CAD                    0x07

/*PA config*/
#define PA_BOOST                    0x80

/*IRQ FLAGS*/
#define IRQ_CAD_DETECTED_MASK       0x01
#define IRQ_FHSS_CHANGE_MASK        0x02
#define IRQ_CAD_DONE_MASK           0x04
#define IRQ_TX_DONE_MASK            0x08
#define IRQ_VALID_HEADER_MASK       0x10
#define IRQ_PAYLOAD_CRC_ERR_MASK    0x20
#define IRQ_RX_DONE_MASK            0x40
#define IRQ_RX_TIMEOUT_MASK         0x80




/*STRUCTS ****************************************************************************************/

typedef struct {

    uint32_t frequency;         /*ex: 915000000UL for 915 MHz*/
    uint8_t spreading_Factor;   /* 7-12 (SF6 will have extra register writes) */
    uint32_t bandwidth;          /* 7800, 10400, 15600, 31250, 41700,
                                 62500, 125000, 250000, 500000 Hz */
    
    uint8_t coding_Rate;        /* 5=4/5, 6=4/6, 7=4/7, 8=4/8 */
    uint8_t tx_Power;           /* dBm — 2..17 (RFO) or 2..20 (PA_BOOST) */
    uint8_t use_Pa_Boost;       /* 1 = PA_BOOST pin, 0 = RFO pin */
    uint16_t preamble_Length;   /* symbols, min 6, default 8 */
    uint8_t sync_Word;          /* 0x12 private, 0x34 LoRaWAN */
    uint8_t enable_CRC;         /* 1 = CRC on payload, 0 = off */
    uint8_t implicit_header;    /* 0 for explicit header mode, 1 for implicit */

} LORA_CONFIG_TYPEDEF;

typedef struct {

    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *nss_port; 
    uint16_t nss_pin; 
    
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;

} SX1276_HandleTypedef;


/*FUNCTIONS **************************************************************************************/

/*Initialization*/

HAL_StatusTypeDef lora_init(SX1276_HandleTypedef *sx1262, 
                            const LORA_CONFIG_TYPEDEF *lora_config);
/*Transmit*/                            
HAL_StatusTypeDef lora_TX(const uint8_t *data, uint8_t length, uint32_t timeout_ms);

/*Recive*/

HAL_StatusTypeDef lora_RX(uint8_t *buff, uint8_t *rx_length, uint8_t max_length, uint32_t timeout_ms);

void lora_receive_cont(void);

HAL_StatusTypeDef lora_receive_cont_poll(uint8_t *buff, uint8_t *rx_length);

/*Utility*/

HAL_StatusTypeDef lora_sleep();
HAL_StatusTypeDef lora_standby();
HAL_StatusTypeDef lora_packet_rssi(int16_t *dbm); /*returns dBm*/
HAL_StatusTypeDef lora_version(uint8_t *lora_version); /*should return 0x12*/
HAL_StatusTypeDef lora_packet_snr(float *snr);



#ifdef __cplusplus
}
#endif

#endif