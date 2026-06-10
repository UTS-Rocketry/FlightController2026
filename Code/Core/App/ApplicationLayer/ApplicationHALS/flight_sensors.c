#include "flight_sensors.h"
#include <stdio.h>
#include "BMP388.h"
#include "lsm6dsox_reg.h"
#include "h3lis331dl_reg.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"

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


/* This function takes initializes the pins for the BMP388 Barometer
   Parameters: BMP388Handle_TypeDef
   returns: void 
   Maybe change later to return a HAL_StatusTypedef for debuging*/
static void BMP388_handleinit (BMP388Handle_TypeDef *bmp) {
  
  /*Set pins and spi*/
  bmp->hspi = &hspi1; 
  bmp->cs_port = CSBarometer_GPIO_Port;
  bmp->cs_pin = CSBarometer_Pin;

  /*This is just for debuging 
    however future implemetation will send status
    over LoRa*/

  if (BMP388_Init(bmp) != HAL_OK) {
    #ifdef DEBUG
      printf("BMP388 init FAILED\r\n");
    #endif
  } else {
    #ifdef DEBUG
      printf("BMP388 OK\r\n");
    #endif
  }

}

static void lsm6dso_handleinit(lsm6dso_HandleTypedef *imu) {
  /* sets the sensor bus and pins */
  imu->hspi = &hspi1;
  imu->cs_port = CS_IMU_GPIO_Port;
  imu->cs_pin = CS_IMU_Pin;

  /* debug statements */
  if (lsm6dso_init(imu) != HAL_OK) {
    #ifdef DEBUG
      printf("IMU init FAILED\r\n");
    #endif
  } else {
    #ifdef DEBUG
      printf("IMU OK\r\n");
    #endif
  }

}

static void h3lis331dl_handleinit(h3lis331dl_HandleTypeDef *accel) {

  /*Sets the struct values from main.h same accoss all senssor and perifierials 
    so we can use the code just need to change the values if ported */
  accel->hspi = &hspi1;
  accel->cs_port = CSAccelerometer_GPIO_Port;
  accel->cs_pin = CSAccelerometer_Pin;

  /* Debug statements */
  if (h3lis331dl_init(accel) != HAL_OK) {
      #ifdef DEBUG
        printf("ACCEL init FAILED\r\n");
      #endif
  } else {
      #ifdef DEBUG
        printf("ACCEL OK\r\n");
      #endif
  }

}

/* This Function is called in main
   Parameters: void
   returns: HAL_StatusTypedef (HAL_OK or HAL_ERROR)
   This function initializes all the sensors on ODIN
   and does calls their calibration functions */

HAL_StatusTypeDef flight_sensors_init(void) {
  

  HAL_StatusTypeDef result;
  
  /*SENSOR INITS*/
  /* BAROMETER INIT */

  BMP388_handleinit(&bmp);
  HAL_Delay(50);


  /* This function is a calibration to find the ground pressure so we dont set altitude to sea level it is to the actuall
     elevation we are at returns a HAL_StatusTypeDef */

  result = BMP388_FindGroundPressure(&bmp, &ground_pressure);
  
  /* Debug statements*/
  if (result != HAL_OK) {  
    #ifdef DEBUG
      printf("Ground pressure error\r\n");
    #endif
    #ifndef HIL_SIM
      return HAL_ERROR;
    #endif
  } 
  
  /*ACCEL INIT*/
  h3lis331dl_handleinit(&accel);
  HAL_Delay(50);
  /* This does the same as barometer however this is due to sensor error and we will average and compensate again to 
     calibrate the sensor */
  result = h3lis331dl_Calibration(accel_offset);
  
  /* debug statements */
  /* Beep OK! */
  if (result != HAL_OK) {
    #ifdef DEBUG
      printf("Accelerometer Calibration Error\r\n");
    #endif
    #ifndef HIL_SIM
      return HAL_ERROR;
    #endif
  } 


  /*IMU INIT*/
  lsm6dso_handleinit(&imu);
  HAL_Delay(50);

  /* creates an offset to be used against sensor values to account for sensor error */
  result = lsm6dso_Calib(xl_Offset, gy_Offset);

  /* debug */
  if (result != HAL_OK) {
    #ifdef DEBUG
      printf("IMU Calibration Error\r\n");
    #endif
    
    #ifndef HIL_SIM
      return HAL_ERROR;
    #endif

  }

  
  /*return HAL_Status Typedef */
  #ifdef HIL_SIM
    return HAL_OK;   /* under sim, ignore sensor init failures */
  #else
    return result;
  #endif
  
}


#ifndef HIL_SIM

HAL_StatusTypeDef flight_sensors_update_baro(FlightSensorData *sensordata) {

  HAL_StatusTypeDef result;
  if (sensordata == NULL) return HAL_ERROR;
  
  result = BMP388_ExternalReadFunction(&bmp, &sensordata->pressure, &sensordata->temperature, &sensordata->altitude, &ground_pressure);
  if (result != HAL_OK) {
    #ifdef DEBUG
      printf("BMP388 Error\r\n");
    #endif
    return result;

  }

  return HAL_OK;

}

HAL_StatusTypeDef flight_sensors_update_IMU_accel(FlightSensorData *sensordata) {
  
  HAL_StatusTypeDef result;
  if (sensordata == NULL) return HAL_ERROR;
  result = h3lis331dl_externalRead(accel_val);
  if (result != HAL_OK) {

    #ifdef DEBUG
      printf("h3lis331dl Error\r\n");
    #endif
    return result;

  } 

  sensordata->x_mg = h3lis331dl_from_fs200_to_mg(accel_val[0]) - accel_offset[0];
  sensordata->y_mg = h3lis331dl_from_fs200_to_mg(accel_val[1]) - accel_offset[1];
  sensordata->z_mg = h3lis331dl_from_fs200_to_mg(accel_val[2]) - accel_offset[2];

  result = lsm6dso_ExternalReader(xl_Val, gy_Val);
  if (result != HAL_OK) {

    #ifdef DEBUG
      printf("lsm6dso Error\r\n");
    #endif
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

#else
#include "sim_profile.h"
#include "flight_state.h"
static uint32_t sim_idx = 0;

HAL_StatusTypeDef flight_sensors_update_IMU_accel(FlightSensorData *d) {
    // hold on the pad sample until armed (FSM reaches PAD), then play the flight
    if (FSM_get_state() >= STATE_PAD && sim_idx < SIM_LEN - 1) {
        sim_idx++;
    }
    d->z_mg_IMU = sim_accel_mg[sim_idx];
    return HAL_OK;
}

HAL_StatusTypeDef flight_sensors_update_baro(FlightSensorData *d) {
    d->altitude = sim_alt[sim_idx];
    return HAL_OK;
}
#endif