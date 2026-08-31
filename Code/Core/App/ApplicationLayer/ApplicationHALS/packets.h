#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>
#include "flight_sensors.h"
#include "GPS.h"

#define CMD_AUTH_BYTE 0xBE
#define TELEMETRY_PAYLOAD_SIZE 58U
#define GPS_PAYLOAD_SIZE 33U
#define CONTINUITY_PAYLOAD_SIZE 8U
#define COMMAND_PAYLOAD_SIZE 9U
#define LORA_RECEIVER_HEADER_SIZE 4U

typedef struct{

    uint8_t sync_word;
    uint8_t packet_type;
    uint8_t sequence_number;
    

}HeaderPacket;


typedef struct{

    HeaderPacket header;
    FlightSensorData sensordata;
    uint8_t flight_State;
    uint16_t crc;

}TelemetryPacket;

typedef struct{

    HeaderPacket header;
    GPSFix fix;
    uint16_t age_ms;
    uint8_t flight_State;
    uint16_t crc;

}GPSPacket;

typedef struct{

    HeaderPacket header;
    uint8_t main;
    uint8_t drogue;
    uint8_t aux;
    uint16_t crc;

}ContinuityPacket;

typedef struct {
    HeaderPacket header;
    uint8_t cmd_id;
    uint8_t channel;
    uint8_t auth;
    uint16_t crc;
} CommandPacket;


typedef enum {

    telemetry_packet = 0x00,
    gps_packet = 0x01,
    continuity_packet = 0x02,
    command_packet = 0x03

}PacketType;

typedef enum {
    CMD_ARM    = 0x01,
    CMD_FIRE   = 0x02,
    CMD_DISARM = 0x03
} CommandID;

void telemetry_serializer(TelemetryPacket *packet, uint8_t *buff);
void gps_serializer(const GPSPacket *packet, uint8_t *buff);
void continuity_serializer(ContinuityPacket *packet, uint8_t *buff);
uint8_t command_deserializer(uint8_t *buff, uint8_t *cmd_id, uint8_t *channel);

#endif
