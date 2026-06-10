//Header Guards means it wont double compile if header has been defined before
#ifndef BMP388_H
#define BMP388_H

#include "stm32f405xx.h"
#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/

#define MAX_LEN                                          100
#define BMP3_ADDR                                       0x76

#define BMP3_CHIP_ID                                    0x50

/* BMP 388 commands */
#define BMP3_CMD_RDY                                    0x10
#define BMP388_SOFTRESET                                0xB6

/* Over sampling macros */
#define BMP388_NO_OVERSAMPLING                          0x00
#define BMP388_OVERSAMPLING_2X                          0x01
#define BMP388_OVERSAMPLING_4X                          0x02
#define BMP388_OVERSAMPLING_8X                          0x03
#define BMP388_OVERSAMPLING_16X                         0x04
#define BMP388_OVERSAMPLING_32X                         0x05

/* Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE                         0x00
#define BMP3_IIR_FILTER_COEFF_1                         0x01
#define BMP3_IIR_FILTER_COEFF_3                         0x02
#define BMP3_IIR_FILTER_COEFF_7                         0x03
#define BMP3_IIR_FILTER_COEFF_15                        0x04
#define BMP3_IIR_FILTER_COEFF_31                        0x05
#define BMP3_IIR_FILTER_COEFF_63                        0x06
#define BMP3_IIR_FILTER_COEFF_127                       0x07

/* Output Data Rate macros */
#define BMP3_ODR_200_HZ                                 0x00
#define BMP3_ODR_100_HZ                                 0x01
#define BMP3_ODR_50_HZ                                  0x02
#define BMP3_ODR_25_HZ                                  0x03
#define BMP3_ODR_12_5_HZ                                0x04
#define BMP3_ODR_6_25_HZ                                0x05
#define BMP3_ODR_3_1_HZ                                 0x06
#define BMP3_ODR_1_5_HZ                                 0x07
#define BMP3_ODR_0_78_HZ                                0x08
#define BMP3_ODR_0_39_HZ                                0x09
#define BMP3_ODR_0_2_HZ                                 0x0A
#define BMP3_ODR_0_1_HZ                                 0x0B
#define BMP3_ODR_0_05_HZ                                0x0C
#define BMP3_ODR_0_02_HZ                                0x0D
#define BMP3_ODR_0_01_HZ                                0x0E
#define BMP3_ODR_0_006_HZ                               0x0F
#define BMP3_ODR_0_003_HZ                               0x10
#define BMP3_ODR_0_001_HZ                               0x11

#define BMP3_CALIBDATA_LEN                              21

/* FIFO */
#define BMP3_NORMAL_PRESS_AND_TEMP_FRAME_HEADER         0x94U
#define BMP3_SENSOR_TIME_FRAME_HEADER                   0xA0U
#define BMP3_EMPTY_FRAME_HEADER                         0x80U

/* ----- REGISTER MACROS ----- */

#define BMP3_PWR_CTRL_PRESS_ON                          (1U)
#define BMP3_PWR_CTRL_PRESS_OFF                         0U
#define BMP3_PWR_CTRL_TEMP_ON                           (1U << 1)
#define BMP3_PWR_CTRL_TEMP_OFF                          0U
#define BMP3_PWR_CTRL_MODE_SLEEP                        0U
#define BMP3_PWR_CTRL_MODE_FORCED                       (1U << 4)
#define BMP3_PWR_CTRL_MODE_NORMAL                       (3U << 4)

#define BMP3_FIFO_CONFIG_1_FIFO_MODE_ON                 1U
#define BMP3_FIFO_CONFIG_1_FIFO_MODE_OFF                0U
#define BMP3_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_ON         (1U << 1)
#define BMP3_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_OFF        0U
#define BMP3_FIFO_CONFIG_1_FIFO_TIME_EN_ON              (1U << 2)
#define BMP3_FIFO_CONFIG_1_FIFO_TIME_EN_OFF             0U
#define BMP3_FIFO_CONFIG_1_FIFO_PRESS_EN_ON             (1U << 3)
#define BMP3_FIFO_CONFIG_1_FIFO_PRESS_EN_OFF            0U
#define BMP3_FIFO_CONFIG_1_FIFO_TEMP_EN_ON              (1U << 4)
#define BMP3_FIFO_CONFIG_1_FIFO_TEMP_EN_OFF             0U


/* BMP388 registers */
typedef enum{
    CHIP_ID             = 0x00,
    ERR_REG             = 0x02,
    STATUS              = 0x03,
    DATA_0              = 0x04, // PRESS_XLSB_7_0
    DATA_1              = 0x05, // PRESS_LSB_15_8
    DATA_2              = 0x06, // PRESS_MSB_23_16
    DATA_3              = 0x07, // TEMP_XLSB_7_0
    DATA_4              = 0x08, // TEMP_LSB_15_8
    DATA_5              = 0x09, // TEMP_MSB_23_16
    SENSORTIME_0        = 0x0C,
    SENSORTIME_1        = 0x0D,
    SENSORTIME_2        = 0x0E,
    EVENT               = 0x10,
    INT_STATUS          = 0x11,
    FIFO_LENGTH_0       = 0x12,
    FIFO_LENGTH_1       = 0x13,
    FIFO_DATA           = 0x14,
    FIFO_WTM_0          = 0x15,
    FIFO_WTM_1          = 0x16,
    FIFO_CONFIG_1       = 0x17,
    FIFO_CONFIG_2       = 0x18,
    INT_CTRL            = 0x19,
    IF_CONF             = 0x1A,
    PWR_CTRL            = 0x1B,
    OSR                 = 0x1C,
    ODR                 = 0x1D,
    CONFIG              = 0x1F,
    CALIB_DATA          = 0x31,
    CMD                 = 0x7E,
}BMP388_regs;


//stores the values of the calibration data in the datasheet it shows you how to do the math with examples to convert into a real output
typedef struct{
    float               par_t1;
    float               par_t2;
    float               par_t3;
    float               par_p1;
    float               par_p2;
    float               par_p3;
    float               par_p4;
    float               par_p5;
    float               par_p6;
    float               par_p7;
    float               par_p8;
    float               par_p9;
    float               par_p10;
    float               par_p11;
}Calib_data;

/* BMP388 structure -----------------------------------------------------------------*/

typedef struct{

    //this is the spi struct pass a pointer of it to the struct for bmp388
    SPI_HandleTypeDef *hspi;

    uint8_t osr; //oversampling settings OSR register
    uint8_t iir; // iir filtering coeif  IIR register
    uint8_t odr; //output data rate      ODR register
    uint16_t cs_pin; //pass cs pin to drive the device low
    GPIO_TypeDef *cs_port; // this is the pin port
    Calib_data calib_data;

}BMP388Handle_TypeDef;

HAL_StatusTypeDef BMP388_Init (BMP388Handle_TypeDef *bmp);

HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388Handle_TypeDef *bmp, uint32_t *raw_pressure, uint32_t *raw_temperature, uint32_t *time);
void BMP388_CompensateRawPressTemp(BMP388Handle_TypeDef *bmp, uint32_t raw_pressure, uint32_t raw_temperature, float *pressure, float *temperature);
float BMP388_FindAltitude(float ground_pressure, float pressure);
HAL_StatusTypeDef BMP388_FindGroundPressure (BMP388Handle_TypeDef *bmp, float *ground_pressure);
HAL_StatusTypeDef BMP388_ExternalReadFunction(BMP388Handle_TypeDef *bmp, float *pressure, float *temperature, 
											  float *altitude, float *ground_pressure);



#ifdef __cplusplus
}
#endif

#endif