#ifndef CAN_H
#define CAN_H
#include "stm32f405xx.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" { 
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


typedef enum {

    ODIN            = 0x01, // Flight computer
    KESTREL         = 0x02, // Control Board
    RAVEN           = 0x03, // Camera Board
    HUGINN          = 0x04, // CAN Sniffer

} NODE_ID;

typedef enum {

    HEARTBEAT_MSG   = 0x01,
    FSM_STATE_MSG   = 0x02,
    KESTREL_COMMAND = 0x03,
    RAVEN_COMMAND   = 0x04,
    ACK             = 0x05,
    ACTUATOR_STATUS = 0x06,
    CAMERA_STATUS   = 0x07,
    ERROR           = 0x08

} MESSAGE_TYPE;


typedef enum {

    IDLE            = 0x00,
    ARMED           = 0x01,
    POWERED_ASCENT  = 0x02,
    COASTING        = 0x03,
    APOGEE          = 0x04,
    DESCENT         = 0x05,
    LANDED          = 0x06,
    FAULT           = 0x07,
    BENCHTEST       = 0x08

} FLIGHT_STATE;

typedef enum {

    ACTIVATE        = 0x01,
    DEACTIVATE      = 0x02,
    DEPLOY_PARAFOIL = 0x03

}KESTREL_COM_T;

typedef enum {

    RECORD_START = 0x01,
    RECORD_STOP = 0x02

} CAMERA_COM_T;

typedef struct {

    uint8_t CANID;
    uint8_t STATE;

}HEARTBEAT;


typedef struct {

    uint8_t CANID;
    uint8_t STATE;

}FSM_STATE;








#ifdef __cplusplus
 }
#endif


#endif