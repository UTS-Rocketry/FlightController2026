#include "telemetry.h"
#include "Lora_App.h"
#include "LoRa.h"
#include "crc16.h"
#include "flight_sensors.h"
#include "kalman.h"
#include "packets.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "pyro.h"
#include "flight_state.h"
#include "main.h"


static float read_be_float(const uint8_t *b);

void serial_print(const FlightSensorData *sensordata)
{
    if (sensordata == NULL) return;
    #ifdef DEBUG
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
    #endif
}

HAL_StatusTypeDef lora_tx_telemetry(FlightSensorData *sensordata) {
    
    static uint8_t seq = 0;
    uint8_t buff[62] = {0};
    HAL_StatusTypeDef result;
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = 0x01;
    packet.header.sequence_number = seq;

    packet.sensordata = *sensordata;

    packet.flight_State = sensordata->flight_state;

    
    // LoRa driver consumes first 4 bytes internally (SX1276 FIFO header)
    // payload starts at buff + 4
    telemetry_serializer(&packet, buff + 4);

    lora_tx_done_flag = 0;

    result = lora_TX(buff, 62, 200);

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
#ifdef MEMORY_DUMP
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

        altitude    = read_be_float(&buff[3]);
        pressure    = read_be_float(&buff[7]);
        temperature = read_be_float(&buff[11]);
        x_mg        = read_be_float(&buff[15]);
        y_mg        = read_be_float(&buff[19]);
        z_mg        = read_be_float(&buff[23]);
        x_mg_IMU    = read_be_float(&buff[27]);
        y_mg_IMU    = read_be_float(&buff[31]);
        z_mg_IMU    = read_be_float(&buff[35]);
        x_gy        = read_be_float(&buff[39]);
        y_gy        = read_be_float(&buff[43]);
        z_gy        = read_be_float(&buff[47]);
        velocity    = read_be_float(&buff[51]);
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

#endif

HAL_StatusTypeDef lora_rx_command() {
    HAL_StatusTypeDef result;

    uint8_t buff[13] = {0};
    uint8_t rxLength = 0;

    result = lora_RX(buff, &rxLength, 13, 150);


    if(result == HAL_TIMEOUT) {
        return HAL_OK;
    }

    if(result == HAL_OK) {
    
    #ifdef DEBUG
        printf("RX ok: %02X %02X id=%02X ch=%02X auth=%02X\r\n",
            buff[4], buff[5], buff[7], buff[8], buff[10]);
    #endif
        

        uint8_t cmd_id;
        uint8_t channel;
        if(command_deserializer(buff+4, &cmd_id, &channel)) {
            switch (cmd_id) {
                
                case CMD_ARM:
                     #ifdef DEBUG
                        printf("CMD_ARM rx, before=%d\r\n", FSM_get_state());
                    #endif
                    FSM_arm();
                    #ifdef DEBUG
                        printf("after=%d\r\n", FSM_get_state());
                    #endif
                    break;
                
                case CMD_FIRE:
                    
                    if (FSM_get_state() != STATE_PAD) break;   // only fire on the pad, post-arm
                            switch (channel) {
                                case 1:
                                    pyro_fire_drogue_ground();
                                    FSM_disarm();
                                    break;
                                case 2: 
                                    pyro_fire_main_ground();
                                    FSM_disarm();   
                                    break;
                            }
                    
                    break;
                  
                case CMD_DISARM:
                    FSM_disarm();
                    /*disarm rocket*/
                    break;

            }
            
        }
    }

    return result;


}

HAL_StatusTypeDef lora_tx_continuity() {
    
    static uint8_t seq = 0;
    uint8_t buff[12] = {0};
    HAL_StatusTypeDef result;
    ContinuityPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = continuity_packet;
    packet.header.sequence_number = seq;

    packet.main = pyro_check_main();
    packet.drogue = pyro_check_drogue();
    
    // LoRa driver consumes first 4 bytes internally (SX1276 FIFO header)
    // payload starts at buff + 4
    continuity_serializer(&packet, buff + 4);

    result = lora_TX(buff, 12, 100);

    seq++;

    return result;

}

static float read_be_float(const uint8_t *b)
{
    uint32_t raw = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                   ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    float f;
    memcpy(&f, &raw, 4);
    return f;
}