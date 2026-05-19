#include "flight_sensors.h"
#include <stdio.h>
#include "BMP388.h"
#include "lsm6dsox_reg.h"
#include "h3lis331dl_reg.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
static BMP388Handle_TypeDef bmp;
static lsm6dso_HandleTypedef imu;
static h3lis331dl_HandleTypeDef accel;

/*BAROMETER VARIABLES*/
static float ground_pressure;

/*ACCEL VARIABLES*/
static int16_t accel_val[3] = {0};
static float accel_offset[3] = {0};

/*IMU VARIABLES*/
static int16_t xl_Val[3] = {0};
static int16_t gy_Val[3] = {0};
static float xl_Offset[3] = {0};
static float gy_Offset[3] = {0};

static void BMP388_handleinit (BMP388Handle_TypeDef *bmp);
static void lsm6dso_handleinit(lsm6dso_HandleTypedef *imu);
static void h3lis331dl_handleinit(h3lis331dl_HandleTypeDef *accel);


static void BMP388_handleinit (BMP388Handle_TypeDef *bmp) {
  
  bmp->hspi = &hspi1;  // confirm this is correct
  bmp->cs_port = CSBarometer_GPIO_Port;
  bmp->cs_pin = CSBarometer_Pin;

  if (BMP388_Init(bmp) != HAL_OK) {
      printf("BMP388 init FAILED\r\n");
  } else {
    printf("BMP388 OK\r\n");
  }

}

static void lsm6dso_handleinit(lsm6dso_HandleTypedef *imu) {

  imu->hspi = &hspi1;
  imu->cs_port = CS_IMU_GPIO_Port;
  imu->cs_pin = CS_IMU_Pin;
  if (lsm6dso_init(imu) != HAL_OK) {
    printf("IMU init FAILED\r\n");
  } else {
      printf("IMU OK\r\n");
  }

}

static void h3lis331dl_handleinit(h3lis331dl_HandleTypeDef *accel) {

  accel->hspi = &hspi1;
  accel->cs_port = CSAccelerometer_GPIO_Port;
  accel->cs_pin = CSAccelerometer_Pin;

  if (h3lis331dl_init(accel) != HAL_OK) {
      printf("ACCEL init FAILED\r\n");
  } else {
      printf("ACCEL OK\r\n");
  }

}

HAL_StatusTypeDef flight_sensors_init(void) {
  
  HAL_StatusTypeDef result;

  /*SENSOR INITS*/
  /* BAROMETER INIT */
  BMP388_handleinit(&bmp);
  HAL_Delay(50);
  result = BMP388_FindGroundPressure(&bmp, &ground_pressure);
  if (result != HAL_OK) {

    printf("Ground pressure error\r\n");
    return HAL_ERROR;
    
  } 
  
  /*ACCEL INIT*/
  h3lis331dl_handleinit(&accel);
  HAL_Delay(50);
  result = h3lis331dl_Calibration(accel_offset);
  
  
  /* Beep OK! */
  if (result != HAL_OK) {

    printf("Accelerometer Calibration Error\r\n");
    return HAL_ERROR;
  } 

  /*IMU INIT*/
  lsm6dso_handleinit(&imu);
  HAL_Delay(50);
  result = lsm6dso_Calib(xl_Offset, gy_Offset);
  if (result != HAL_OK) {

    printf("IMU Calibration Error\r\n");
    return HAL_ERROR;

  }

  return result;

  
}


HAL_StatusTypeDef flight_sensors_update(FlightSensorData *sensordata) {
    

  HAL_StatusTypeDef result;
  if (sensordata == NULL) return HAL_ERROR;
  
  result = BMP388_ExternalReadFunction(&bmp, &sensordata->pressure, &sensordata->temperature, &sensordata->altitude, &ground_pressure);
  if (result != HAL_OK) {
    
    printf("BMP388 Error\r\n");
    return result;

  }

  result = h3lis331dl_externalRead(accel_val);
  if (result != HAL_OK) {

    printf("h3lis331dl Error\r\n");
    return result;

  } 

  sensordata->x_mg = h3lis331dl_from_fs200_to_mg(accel_val[0]) - accel_offset[0];
  sensordata->y_mg = h3lis331dl_from_fs200_to_mg(accel_val[1]) - accel_offset[1];
  sensordata->z_mg = h3lis331dl_from_fs200_to_mg(accel_val[2]) - accel_offset[2];

  result = lsm6dso_ExternalReader(xl_Val, gy_Val);
  if (result != HAL_OK) {

    printf("lsm6dso Error\r\n");
    return result;

  } 

  sensordata->x_mg_IMU = lsm6dsox_from_fs16_to_mg(xl_Val[0]) - xl_Offset[0];
  sensordata->y_mg_IMU = lsm6dsox_from_fs16_to_mg(xl_Val[1]) - xl_Offset[1];
  sensordata->z_mg_IMU = lsm6dsox_from_fs16_to_mg(xl_Val[2]) - xl_Offset[2];

  sensordata->x_gy = lsm6dsox_from_fs2000_to_mdps(gy_Val[0]) - gy_Offset[0];
  sensordata->y_gy = lsm6dsox_from_fs2000_to_mdps(gy_Val[1]) - gy_Offset[1];
  sensordata->z_gy = lsm6dsox_from_fs2000_to_mdps(gy_Val[2]) - gy_Offset[2];

  return HAL_OK;
  
}