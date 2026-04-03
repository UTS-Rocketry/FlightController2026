#include "CAN.h"
#include "stm32f405xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"



CAN_HandleTypeDef hcan1;

HAL_StatusTypeDef Can_init() {

    HAL_StatusTypeDef result;

    hcan1.Instance = CAN2;
    
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    
    /*prescaler based on 42mhz clock and 500kbps data transfer*/
    hcan1.Init.Prescaler = 6;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

    hcan1.Init.TimeTriggeredMode = DISABLE;
   
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    result = HAL_CAN_Init(&hcan1);

    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    CAN_FilterTypeDef hcan1Filter;

    hcan1Filter.FilterIdHigh = 0x0000;
    hcan1Filter.FilterIdLow = 0x0000;
    hcan1Filter.FilterMaskIdLow = 0x0000;
    hcan1Filter.FilterMaskIdHigh  = 0x0000;

    hcan1Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

    hcan1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    hcan1Filter.FilterScale = CAN_FILTERSCALE_32BIT;

    hcan1Filter.FilterBank = 14;
    hcan1Filter.SlaveStartFilterBank = 14;

    hcan1Filter.FilterActivation = CAN_FILTER_ENABLE;

    result = HAL_CAN_ConfigFilter(&hcan1, &hcan1Filter);
    
    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    result = HAL_CAN_Start(&hcan1);
    
    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    return result;



}