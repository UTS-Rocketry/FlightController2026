#include "CAN.h"
#include "stm32f405xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>



CAN_HandleTypeDef hcan2;

HAL_StatusTypeDef Can_init(void) {

    HAL_StatusTypeDef result;

    hcan2.Instance = CAN2;
    
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.Prescaler = 6;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;  
    hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;

    hcan2.Init.TimeTriggeredMode = DISABLE;
   
    hcan2.Init.AutoBusOff = ENABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = ENABLE;
    
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;

    result = HAL_CAN_Init(&hcan2);

    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    CAN_FilterTypeDef hcan2Filter;

    hcan2Filter.FilterIdHigh = 0x0000;
    hcan2Filter.FilterIdLow = 0x0000;
    hcan2Filter.FilterMaskIdLow = 0x0000;
    hcan2Filter.FilterMaskIdHigh  = 0x0000;

    hcan2Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

    hcan2Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    hcan2Filter.FilterScale = CAN_FILTERSCALE_32BIT;

    hcan2Filter.FilterBank = 14;
    hcan2Filter.SlaveStartFilterBank = 14;

    hcan2Filter.FilterActivation = CAN_FILTER_ENABLE;

    result = HAL_CAN_ConfigFilter(&hcan2, &hcan2Filter);
    
    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    result = HAL_CAN_Start(&hcan2);
    
    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    result =  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    return result;



}

HAL_StatusTypeDef can_transmit(uint8_t nodeID,  uint8_t msgType, uint8_t *payload, uint8_t length) {

    if (length > 8) return HAL_ERROR;

    CAN_TxHeaderTypeDef canTX1;
    uint32_t txMailbox;

    uint32_t canID = CAN_ID(nodeID, msgType); 

    canTX1.StdId = canID;
    canTX1.IDE = CAN_ID_STD;
    canTX1.RTR = CAN_RTR_DATA;
    canTX1.DLC = length;
    canTX1.TransmitGlobalTime = DISABLE;


    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan2, &canTX1, payload, &txMailbox);
    #ifdef DEBUG
        if (result != HAL_OK) {
            printf("CAN err=0x%08lX state=%d\r\n", HAL_CAN_GetError(&hcan2), HAL_CAN_GetState(&hcan2));
        }
    #endif

    return result;

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    CAN_RxHeaderTypeDef canRX1;

    uint8_t rxData[8];
    
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &canRX1, rxData);

    uint8_t nodeID = canRX1.StdId >> 7;
    uint8_t msgType = canRX1.StdId  & 0x7f;

    switch (nodeID) {
        
        case KESTREL :
            switch (msgType) {
            
            }
            break;

        case RAVEN :
            switch (msgType) {
            
            }
            break;

        case HUGINN :
            switch (msgType) {
            
            }
            break;
    
    }

}
