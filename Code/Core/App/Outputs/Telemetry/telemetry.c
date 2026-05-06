#include "telemetry.h"
#include "Lora_App.h"

void serial_print(const FlightSensorData *sensordata)
{
    if (sensordata == NULL) return;

    printf("Alt: %.2fm | H3LIS X:%.1f Y:%.1f Z:%.1f mg | "
           "IMU XL X:%.1f Y:%.1f Z:%.1f mg | "
           "GY X:%.1f Y:%.1f Z:%.1f mdps\r\n",

           sensordata->altitude,

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
    uint8_t buff[54] = {0};
    HAL_StatusTypeDef result;
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = 0x01;
    packet.header.sequence_number = seq;

    packet.sensordata = *sensordata;

    packet.flight_State = 0x01; /*placeholder for now*/



    lora_telemetry_seraializer(&packet, buff);

    result = lora_TX(buff, 54, 1000);

    seq++;

    return result;


}