#ifndef LORA_APP_H
#define LORA_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "flight_sensors.h"
#include "crc16.h"

typedef struct{

    uint8_t sync_word;
    uint8_t packet_type;
    uint8_t sequence_number;
    

}HeaderPacket;

typedef struct{

    HeaderPacket header;
    FlightSensorData sensordata;
    uint8_t flight_State;
    uint16_t CRC;

}TelemetryPacket;

typedef struct{

    HeaderPacket header;
    /* Gps info */
    uint8_t flight_State;
    uint16_t CRC;

}GPSPacket;

typedef enum {

    telemetry_packet = 0x00,
    gps_packet = 0x01

}PacketType;

typedef enum {

    IDLE            = 0x00,
    ARMED           = 0x01,
    POWERED_ASCENT  = 0x02,
    COASTING        = 0x03,
    APOGEE          = 0x04,
    DESCENT         = 0x05,
    LANDED          = 0x06,
    FAULT           = 0x07

} FlightStateLoRa;

extern volatile uint8_t lora_tx_done_flag;

void HAL_GPIOEXTI_Callback(uint16_t GPIO_PIN);
HAL_StatusTypeDef lora_App_Init();

void lora_telemetry_serializer(TelemetryPacket *packet, uint8_t *buff);

#ifdef __cplusplus
}
#endif

#endif