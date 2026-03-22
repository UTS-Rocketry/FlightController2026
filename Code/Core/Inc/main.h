/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CSFlashmMemory_Pin GPIO_PIN_0
#define CSFlashmMemory_GPIO_Port GPIOC
#define CS_SD_Card_Pin GPIO_PIN_1
#define CS_SD_Card_GPIO_Port GPIOC
#define RGBLEDControl_Pin GPIO_PIN_2
#define RGBLEDControl_GPIO_Port GPIOA
#define AuxIgnite_Pin GPIO_PIN_3
#define AuxIgnite_GPIO_Port GPIOA
#define BuzzerControl_Pin GPIO_PIN_4
#define BuzzerControl_GPIO_Port GPIOC
#define LoRaNssPin_Pin GPIO_PIN_0
#define LoRaNssPin_GPIO_Port GPIOB
#define LoRaDIO1_Pin GPIO_PIN_1
#define LoRaDIO1_GPIO_Port GPIOB
#define LoRaResetPin_Pin GPIO_PIN_2
#define LoRaResetPin_GPIO_Port GPIOB
#define CSAccelerometer_Pin GPIO_PIN_11
#define CSAccelerometer_GPIO_Port GPIOB
#define PyroIgnite_Pin GPIO_PIN_14
#define PyroIgnite_GPIO_Port GPIOB
#define DrougeIgnite_Pin GPIO_PIN_6
#define DrougeIgnite_GPIO_Port GPIOC
#define CSBarometer_Pin GPIO_PIN_7
#define CSBarometer_GPIO_Port GPIOC
#define GPS2ResetPin_Pin GPIO_PIN_9
#define GPS2ResetPin_GPIO_Port GPIOC
#define GPS1ResetPin_Pin GPIO_PIN_8
#define GPS1ResetPin_GPIO_Port GPIOA
#define CS_IMU_Pin GPIO_PIN_10
#define CS_IMU_GPIO_Port GPIOA
#define LoRaDIO0_Pin GPIO_PIN_6
#define LoRaDIO0_GPIO_Port GPIOB
#define LoRaDIO2_Pin GPIO_PIN_7
#define LoRaDIO2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
