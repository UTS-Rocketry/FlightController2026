#include "BMP388.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <math.h>
#include <stdint.h>

HAL_StatusTypeDef BMP388_readBytes(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef BMP388_GetCalibData(BMP388Handle_TypeDef *bmp);
HAL_StatusTypeDef BMP388_resetRegister(BMP388Handle_TypeDef *bmp);
HAL_StatusTypeDef BMP388_readRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data);
HAL_StatusTypeDef BMP388_WriteRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data);

HAL_StatusTypeDef BMP388_SetTempOS(BMP388Handle_TypeDef *bmp, uint8_t oversample);
HAL_StatusTypeDef BMP388_SetPressOS(BMP388Handle_TypeDef *bmp, uint8_t oversample);
HAL_StatusTypeDef BMP388_SetIIRFilterCoeff(BMP388Handle_TypeDef *bmp, uint8_t filtercoeff);
HAL_StatusTypeDef BMP388_SetOutputDataRate(BMP388Handle_TypeDef *bmp, uint8_t odr);

HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388Handle_TypeDef *bmp, uint32_t *raw_pressure, uint32_t *raw_temperature, uint32_t *time);
void BMP388_CompensateRawPressTemp(BMP388Handle_TypeDef *bmp, uint32_t raw_pressure, uint32_t raw_temperature, float *pressure, float *temperature);
float BMP388_CompensateTemp(BMP388Handle_TypeDef *bmp, uint32_t raw_temp, float *temp);
float BMP388_CompensatePress(BMP388Handle_TypeDef *bmp, float temp, uint32_t raw_press, float *press);

                                                        




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

    result = BMP388_resetRegister(bmp);
    
    if (result != HAL_OK) {

        return HAL_ERROR;
    }

    result = BMP388_GetCalibData(bmp);

     if (result != HAL_OK) {

        return HAL_ERROR;
    }


    BMP388_SetTempOS(bmp, BMP388_OVERSAMPLING_2X);
    BMP388_SetPressOS(bmp, BMP388_OVERSAMPLING_8X);
    BMP388_SetIIRFilterCoeff(bmp, BMP3_IIR_FILTER_COEFF_3);
    BMP388_SetOutputDataRate(bmp, BMP3_ODR_50_HZ);

    data = bmp->osr;

    result = BMP388_WriteRegister(bmp, OSR, &data);

    if (result != HAL_OK) {

        return HAL_ERROR;
    }

    data = bmp->iir;
    result = BMP388_WriteRegister(bmp, CONFIG ,&data);

     if (result != HAL_OK) {

        return HAL_ERROR;
    }

    data = bmp->odr;
    result = BMP388_WriteRegister(bmp, ODR,&data);

     if (result != HAL_OK) {

        return HAL_ERROR;
    }

    data = BMP3_PWR_CTRL_MODE_NORMAL;
    result = BMP388_WriteRegister(bmp, PWR_CTRL, &data);
    
    if (result != HAL_OK) {

        return HAL_ERROR;
    }

    
    return result;
}



HAL_StatusTypeDef BMP388_readRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data) {

    //Buffers to store what is in the register
    uint8_t txBuff[2];
    uint8_t rxBuff[2];

    //sets the register to read
    txBuff[0] = BitSet(reg, 7);
    txBuff[1] = 0x00;

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(bmp->hspi, txBuff, rxBuff, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);

    *data = rxBuff[1];

    return result;

}


HAL_StatusTypeDef BMP388_readBytes(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *buff, uint8_t length) {
    
    //Buffers to store what is in the register
    uint8_t txBuff[2];
    uint8_t rxBuff[length + 2];

    //sets the register to read
    txBuff[0] = BitSet(reg, 7);
    txBuff[1] = 0x00;

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(bmp->hspi, txBuff, rxBuff, length + 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);

    for(int x = 2; x < 2 + length; x++) {

        buff[x - 2] = rxBuff[x];

    }

    return result;

}


HAL_StatusTypeDef BMP388_WriteRegister(BMP388Handle_TypeDef *bmp, BMP388_regs reg, uint8_t *data) {
    
    //Buffers to store what is in the register
    uint8_t txBuff[2];

    //sets the register to write Bit 7 = 0
    txBuff[0] = BitClear(reg, 7);
    txBuff[1] = *data;

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef result = HAL_SPI_Transmit(bmp->hspi, txBuff, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);


    return result;

}


HAL_StatusTypeDef BMP388_resetRegister(BMP388Handle_TypeDef *bmp) {
    
    uint8_t data = BMP388_SOFTRESET;

    HAL_StatusTypeDef result = BMP388_WriteRegister(bmp, CMD, &data);

    return result;
}












HAL_StatusTypeDef BMP388_SetTempOS(BMP388Handle_TypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->osr = (bmp->osr & 0xC7) | (oversample << 3); // == 0b11000111
	return HAL_OK;
}

HAL_StatusTypeDef BMP388_SetPressOS(BMP388Handle_TypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->osr = (bmp->osr & 0xF8) | oversample; // 0xF8 == 0b11111000
	return HAL_OK;
}

HAL_StatusTypeDef BMP388_SetIIRFilterCoeff(BMP388Handle_TypeDef *bmp, uint8_t filtercoeff){
	if(filtercoeff > BMP3_IIR_FILTER_COEFF_127){
		return HAL_ERROR;
	}
	bmp->iir = filtercoeff << 1;
	return HAL_OK;
}

HAL_StatusTypeDef BMP388_SetOutputDataRate(BMP388Handle_TypeDef *bmp, uint8_t odr){
	if(odr > BMP3_ODR_0_001_HZ){
		return HAL_ERROR;
	}
	bmp->odr = odr;
	return HAL_OK;
}















HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388Handle_TypeDef *bmp, uint32_t *raw_pressure, uint32_t *raw_temperature, uint32_t *time){
	
    HAL_StatusTypeDef result;

	uint8_t raw_data[11];
	// Get raw data for pressure and temperature
	result = BMP388_readBytes(bmp, DATA_0, raw_data, 11);
	
    if(result != HAL_OK){
		
        return HAL_ERROR;
	
    }

	// Parsing pressure data
	*raw_pressure = (uint32_t)raw_data[2] << 16 | (uint32_t)raw_data[1] << 8 | (uint32_t)raw_data[0];

	// Parsing temperature data
	*raw_temperature = (uint32_t)raw_data[5] << 16 | (uint32_t)raw_data[4] << 8 | (uint32_t)raw_data[3];

	// Parsing time bytes
	*time = (uint32_t)raw_data[10] << 16 | (uint32_t)raw_data[9] << 8 | (uint32_t)raw_data[8];


	return result;
}


void BMP388_CompensateRawPressTemp(BMP388Handle_TypeDef *bmp, uint32_t raw_pressure, uint32_t raw_temperature,
									  	  	  	  	  	  	  float *pressure, float *temperature){
	float temp;
	float press;

	BMP388_CompensateTemp(bmp, raw_temperature, &temp);
	BMP388_CompensatePress(bmp, temp, raw_pressure, &press);

	*pressure = press;
	*temperature = temp;
}

float BMP388_CompensateTemp(BMP388Handle_TypeDef *bmp, uint32_t raw_temp, float *temp){
    float partial_data1 = (float)(raw_temp - bmp->calib_data.par_t1);;
    float partial_data2 = (float)(partial_data1 * bmp->calib_data.par_t2);

    *temp = partial_data2 + (partial_data1 * partial_data1) * bmp->calib_data.par_t3;

    return *temp;
}

float BMP388_CompensatePress(BMP388Handle_TypeDef *bmp, float temp, uint32_t raw_press, float *press){
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;



    partial_data1 = bmp->calib_data.par_p6 * temp;
    partial_data2 = bmp->calib_data.par_p7 * (temp * temp);
    partial_data3 = bmp->calib_data.par_p8 * (temp * temp * temp);
    partial_out1 = bmp->calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp->calib_data.par_p2 * temp;
    partial_data2 = bmp->calib_data.par_p3 * (temp * temp);
    partial_data3 = bmp->calib_data.par_p4 * (temp * temp * temp);
    partial_out2 = (float)raw_press * (bmp->calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = bmp->calib_data.par_p9 + bmp->calib_data.par_p10 * temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * bmp->calib_data.par_p11;

    *press = partial_out1 + partial_out2 + partial_data4;

    return *press;
}


float BMP388_FindAltitude(float ground_pressure, float pressure){
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return 44330.0 * (1.0 - pow(pressure / ground_pressure, 0.1903));
}

HAL_StatusTypeDef BMP388_GetCalibData(BMP388Handle_TypeDef *bmp){
	HAL_StatusTypeDef rslt;
	uint8_t calib_buff[BMP3_CALIBDATA_LEN] = {0};

	uint16_t	raw_par_t1;
	uint16_t	raw_par_t2;
	int8_t		raw_par_t3;
	int16_t		raw_par_p1;
	int16_t		raw_par_p2;
	int8_t		raw_par_p3;
	int8_t		raw_par_p4;
	uint16_t	raw_par_p5;
	uint16_t	raw_par_p6;
	int8_t		raw_par_p7;
	int8_t		raw_par_p8;
	int16_t		raw_par_p9;
	int8_t		raw_par_p10;
	int8_t		raw_par_p11;

	rslt = BMP388_readBytes(bmp, CALIB_DATA, calib_buff, BMP3_CALIBDATA_LEN);

	float temp_var;
	if(rslt == HAL_OK){
		// PAR_T1
		temp_var = 0.00390625f;
		raw_par_t1 = ((uint16_t)calib_buff[1] << 8) | (uint16_t)calib_buff[0];
		bmp->calib_data.par_t1 = (float)raw_par_t1 / temp_var;
		// PAR_T2
		temp_var = 1073741824.f;
		raw_par_t2 = ((uint16_t)calib_buff[3] << 8) | (uint16_t)calib_buff[2];
		bmp->calib_data.par_t2 = (float)raw_par_t2 / temp_var;
		// PAR_T3
		temp_var = 281474976710656.f;
		raw_par_t3 = calib_buff[4];
		bmp->calib_data.par_t3 = (float)raw_par_t3 / temp_var;
		// PAR_P1
		temp_var = 1048576.f;
		raw_par_p1 = ((int16_t)calib_buff[6] << 8) | (int16_t)calib_buff[5];
		bmp->calib_data.par_p1 = ((float)raw_par_p1 - 16384) / temp_var;
		// PAR_P2
		temp_var = 536870912.f;
		raw_par_p2 = ((int16_t)calib_buff[8] << 8) | (int16_t)calib_buff[7];
		bmp->calib_data.par_p2 = ((float)raw_par_p2 - 16384) / temp_var;
		// PAR_P3
		temp_var = 4294967296.f;
		raw_par_p3 = (int8_t)calib_buff[9];
		bmp->calib_data.par_p3 = (float)raw_par_p3 / temp_var;
		// PAR_P4
		temp_var = 137438953472.f;
		raw_par_p4 = (int8_t)calib_buff[10];
		bmp->calib_data.par_p4 = (float)raw_par_p4 / temp_var;
		// PAR_P5
		temp_var = 0.125f;
		raw_par_p5 = ((uint16_t)calib_buff[12] << 8) | (uint16_t)calib_buff[11];
		bmp->calib_data.par_p5 = (float)raw_par_p5 / temp_var;
		// PAR_P6
		temp_var = 64.f;
		raw_par_p6 = ((uint16_t)calib_buff[14] << 8) | (uint16_t)calib_buff[13];
		bmp->calib_data.par_p6 = (float)raw_par_p6 / temp_var;
		// PAR_P7
		temp_var = 256.f;
		raw_par_p7 = (int8_t)calib_buff[15];
		bmp->calib_data.par_p7 = (float)raw_par_p7 / temp_var;
		// PAR_P8
		temp_var = 32768.f;
		raw_par_p8 = (int8_t)calib_buff[16];
		bmp->calib_data.par_p8 = (float)raw_par_p8 / temp_var;
		// PAR_P9
		temp_var = 281474976710656.f;
		raw_par_p9 = ((int16_t)calib_buff[18] << 8) | (int16_t)calib_buff[17];
		bmp->calib_data.par_p9 = (float)raw_par_p9 / temp_var;
		// PAR_P10
		temp_var = 281474976710656.f;
		raw_par_p10 = (int8_t)calib_buff[19];
		bmp->calib_data.par_p10 = (float)raw_par_p10 / temp_var;
		// PAR_P11
		temp_var = 36893488147419103232.f;
		raw_par_p11 = (int8_t)calib_buff[20];
		bmp->calib_data.par_p11 = (float)raw_par_p11 / temp_var;
	}
	return rslt;
}
