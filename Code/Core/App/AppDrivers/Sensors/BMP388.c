#include "BMP388.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <math.h>
#include <stdint.h>



HAL_StatusTypeDef BMP388_resetRegister(BMP388Handle_TypeDef *bmp);

HAL_StatusTypeDef BMP388_readRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data);
HAL_StatusTypeDef BMP388_WriteRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data);


HAL_StatusTypeDef BMP388_Init (BMP388Handle_TypeDef *bmp) {

    HAL_StatusTypeDef result;
    uint8_t data;
    
    result = BMP388_readRegister(bmp,CHIP_ID,&data);
    
    if (result != HAL_OK) {

        return HAL_ERROR;
    }

    if (data != BMP3_ADDR) {
        return HAL_ERROR;
    }
    


}

HAL_StatusTypeDef BMP388_readRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data) {

    //Buffers to store what is in the register
    uint8_t txBuff[2];
    uint8_t rxBuff[2];

    //sets the register to read
    txBuff[0] = reg | 0x80;
    txBuff[1] = 0x00;

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(bmp->hspi, txBuff, rxBuff, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);

    *data = rxBuff[1];

    return HAL_OK;

}
