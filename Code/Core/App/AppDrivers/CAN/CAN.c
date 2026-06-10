#include "CAN.h"
#include "stm32f405xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>



CAN_HandleTypeDef hcan1;

HAL_StatusTypeDef Can_init(CAN_HandleTypeDef *hcan) {

    HAL_StatusTypeDef result;

    hcan1.Instance = CAN2;
    
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    
    /*prescaler based on 36mhz clock and 500kbps data transfer*/
    hcan1.Init.Prescaler = 4;
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

    result =  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    return result;



}

HAL_StatusTypeDef can_transmit(uint8_t nodeID,  uint8_t msgType, uint8_t *payload, uint8_t length) {

    CAN_TxHeaderTypeDef canTX1;
    uint32_t txMailbox;

    uint32_t canID = CAN_ID(nodeID, msgType); 

    canTX1.StdId = canID;
    canTX1.IDE = CAN_ID_STD;
    canTX1.RTR = CAN_RTR_DATA;
    canTX1.DLC = length;
    canTX1.TransmitGlobalTime = DISABLE;


    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &canTX1, payload, &txMailbox);

    return result;

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    CAN_RxHeaderTypeDef canRX1;

    uint8_t rxData[8];
    
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canRX1, rxData);

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
