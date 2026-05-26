#include "telemetry.h"
#include "Lora_App.h"
#include "LoRa.h"
#include "crc16.h"
#include "flight_sensors.h"
#include "kalman.h"

void serial_print(const FlightSensorData *sensordata)
{
    if (sensordata == NULL) return;

    printf("Alt: %.2fm | H3LIS X:%.1f Y:%.1f Z:%.1f mg | "
           "IMU XL X:%.1f Y:%.1f Z:%.1f mg | "
           "GY X:%.1f Y:%.1f Z:%.1f mdps\r\n",

           sensordata->kalman_altitude,

           sensordata->x_mg,
           sensordata->y_mg,
           sensordata->z_mg,

           sensordata->x_mg_IMU,
           sensordata->y_mg_IMU,
           sensordata->z_mg_IMU,

           sensordata->x_gy,
           sensordata->y_gy,
           sensordata->z_gy);
}

HAL_StatusTypeDef lora_tx_telemetry(FlightSensorData *sensordata) {
    
    static uint8_t seq = 0;
    uint8_t buff[58] = {0};
    HAL_StatusTypeDef result;
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = 0x01;
    packet.header.sequence_number = seq;

    packet.sensordata = *sensordata;

    packet.flight_State = sensorData->flight_state;

    
    // LoRa driver consumes first 4 bytes internally (SX1276 FIFO header)
    // payload starts at buff + 4
    telemetry_serializer(&packet, buff + 4);

    result = lora_TX(buff, 62, 1000);

    seq++;

    return result;


}

HAL_StatusTypeDef flash_log_telemetry(FlightSensorData *sensorData) {
    
    static uint16_t seq = 0;
    uint8_t buff[64] = {0};
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = 0x01;
    packet.header.sequence_number = seq++;
    packet.sensordata = *sensorData;
    packet.flight_State = sensorData->flight_state;

    telemetry_serializer(&packet, buff);

    return flash_log_packet(buff, 64);

}

HAL_StatusTypeDef flash_dump_serial(void) {
    
    uint32_t count = flash_get_record_count();

    if (count == 0) {
        printf("No records logged\r\n");
        return HAL_OK;
    }

    printf("Dumping %lu records\r\n", count);

    uint8_t buff[64] = {0};

    for (uint32_t i = 0; i < count; i++) {
        HAL_StatusTypeDef ret = flash_read_record(i, buff, 64);
        if (ret != HAL_OK) {
            printf("Read failed at record %lu\r\n", i);
            return ret;
        }

        float altitude, pressure, temperature;
        float x_mg, y_mg, z_mg;
        float x_mg_IMU, y_mg_IMU, z_mg_IMU;
        float x_gy, y_gy, z_gy;
        float velocity;

        memcpy(&altitude,    &buff[3],  4);
        memcpy(&pressure,    &buff[7],  4);
        memcpy(&temperature, &buff[11], 4);
        memcpy(&x_mg,        &buff[15], 4);
        memcpy(&y_mg,        &buff[19], 4);
        memcpy(&z_mg,        &buff[23], 4);
        memcpy(&x_mg_IMU,    &buff[27], 4);
        memcpy(&y_mg_IMU,    &buff[31], 4);
        memcpy(&z_mg_IMU,    &buff[35], 4);
        memcpy(&x_gy,        &buff[39], 4);
        memcpy(&y_gy,        &buff[43], 4);
        memcpy(&z_gy,        &buff[47], 4);
        memcpy(&velocity,    &buff[51], 4);

        uint8_t state = buff[55];

        printf("[%lu] alt=%.2f pres=%.2f temp=%.2f velocity=%.2f | "
               "hg=%.1f,%.1f,%.1f | "
               "imu=%.1f,%.1f,%.1f | "
               "gy=%.1f,%.1f,%.1f | state=%u\r\n",
               i, altitude, pressure, temperature, velocity,
               x_mg, y_mg, z_mg,
               x_mg_IMU, y_mg_IMU, z_mg_IMU,
               x_gy, y_gy, z_gy,
               state);
    }

    return HAL_OK;
}