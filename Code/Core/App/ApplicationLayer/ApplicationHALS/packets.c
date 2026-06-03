#include "packets.h"
#include "crc16.h"

void telemetry_serializer(TelemetryPacket *packet, uint8_t *buff) {

    buff[0] = packet->header.sync_word;
    buff[1] = packet->header.packet_type;
    buff[2] = packet->header.sequence_number;

    /* start of telemetry */
    uint32_t raw = 0;
    
    /*altitude*/
    memcpy(&raw, &packet->sensordata.kalman_altitude, 4);

    buff[3] = (raw >> 24) & 0xFF;
    buff[4] = (raw >> 16) & 0xFF;
    buff[5] = (raw >> 8) & 0xFF;
    buff[6] = (raw) & 0xFF;
    
    /*Pressure*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.pressure, 4);

    buff[7] = (raw >> 24) & 0xFF;
    buff[8] = (raw >> 16) & 0xFF;
    buff[9] = (raw >> 8) & 0xFF;
    buff[10] = (raw) & 0xFF;

    /*Temperature*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.temperature, 4);

    buff[11] = (raw >> 24) & 0xFF;
    buff[12] = (raw >> 16) & 0xFF;
    buff[13] = (raw >> 8) & 0xFF;
    buff[14] = (raw) & 0xFF;

    
    /*High G accel x_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.x_mg, 4);

    buff[15] = (raw >> 24) & 0xFF;
    buff[16] = (raw >> 16) & 0xFF;
    buff[17] = (raw >> 8) & 0xFF;
    buff[18] = (raw) & 0xFF;

    /*High G accel y_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.y_mg, 4);

    buff[19] = (raw >> 24) & 0xFF;
    buff[20] = (raw >> 16) & 0xFF;
    buff[21] = (raw >> 8) & 0xFF;
    buff[22] = (raw) & 0xFF;

     /*High G accel z_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.z_mg, 4);

    buff[23] = (raw >> 24) & 0xFF;
    buff[24] = (raw >> 16) & 0xFF;
    buff[25] = (raw >> 8) & 0xFF;
    buff[26] = (raw) & 0xFF;

    
    /*IMU x_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.x_mg_IMU, 4);

    buff[27] = (raw >> 24) & 0xFF;
    buff[28] = (raw >> 16) & 0xFF;
    buff[29] = (raw >> 8) & 0xFF;
    buff[30] = (raw) & 0xFF;

    /*IMU y_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.y_mg_IMU, 4);

    buff[31] = (raw >> 24) & 0xFF;
    buff[32] = (raw >> 16) & 0xFF;
    buff[33] = (raw >> 8) & 0xFF;
    buff[34] = (raw) & 0xFF;

    /*IMU z_mg*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.z_mg_IMU, 4);

    buff[35] = (raw >> 24) & 0xFF;
    buff[36] = (raw >> 16) & 0xFF;
    buff[37] = (raw >> 8) & 0xFF;
    buff[38] = (raw) & 0xFF;

    /*IMU x_gy*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.x_gy, 4);

    buff[39] = (raw >> 24) & 0xFF;
    buff[40] = (raw >> 16) & 0xFF;
    buff[41] = (raw >> 8) & 0xFF;
    buff[42] = (raw) & 0xFF;

    /*IMU y_gy*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.y_gy, 4);

    buff[43] = (raw >> 24) & 0xFF;
    buff[44] = (raw >> 16) & 0xFF;
    buff[45] = (raw >> 8) & 0xFF;
    buff[46] = (raw) & 0xFF;

    /*IMU z_gy*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.z_gy, 4);

    buff[47] = (raw >> 24) & 0xFF;
    buff[48] = (raw >> 16) & 0xFF;
    buff[49] = (raw >> 8) & 0xFF;
    buff[50] = (raw) & 0xFF;

    /*Kalman velocity*/
    raw = 0;
    memcpy(&raw, &packet->sensordata.kalman_velocity, 4);

    buff[51] = (raw >> 24) & 0xFF;
    buff[52] = (raw >> 16) & 0xFF;
    buff[53] = (raw >> 8) & 0xFF;
    buff[54] = (raw) & 0xFF;


    buff[55] = packet->flight_State;

    uint16_t crc = crc16(0, buff, 56);

    buff[56] =  (crc >> 8) & 0xFF;
    buff[57] =  (crc) & 0xFF;


}

void continuity_serializer(ContinuityPacket *packet, uint8_t *buff) {
    
    buff[0] = packet->header.sync_word;
    buff[1] = packet->header.packet_type;
    buff[2] = packet->header.sequence_number;

    buff[3] = packet->main;
    buff[4] = packet->drogue;
    buff[5] = 0x00;

    uint16_t crc = crc16(0, buff, 6);

    buff[6] =  (crc >> 8) & 0xFF;
    buff[7] = (crc) & 0xFF;

}

uint8_t command_deserializer(uint8_t *buff, uint8_t *cmd_id, uint8_t *channel) {

    if (buff[0] != 0xAA) return 0;
    if (buff[1] != command_packet) return 0;

    if (buff[6] != CMD_AUTH_BYTE) return 0;
    
    uint16_t calc_crc = crc16(0, buff, 7);  // bytes 0-6
    uint16_t rx_crc = ((uint16_t)buff[7] << 8) | buff[8];
    if (calc_crc != rx_crc) return 0;

    *cmd_id = buff[3];
    *channel = buff[4];

    return 1;

}