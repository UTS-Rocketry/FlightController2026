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
#include <string.h>


static float read_be_float(const uint8_t *b);
static uint8_t command_rx_buffer[LORA_RECEIVER_HEADER_SIZE + COMMAND_PAYLOAD_SIZE];
static uint8_t command_rx_length = 0U;
static uint8_t command_rx_pending = 0U;

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
    if (sensordata == NULL) return HAL_ERROR;
    
    static uint8_t seq = 0;
    uint8_t buff[LORA_RECEIVER_HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE] = {0};
    HAL_StatusTypeDef result;
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = telemetry_packet;
    packet.header.sequence_number = seq;

    packet.sensordata = *sensordata;

    packet.flight_State = sensordata->flight_state;

    
    // Four leading zero bytes are the receiver-routing header used by this protocol.
    // The application payload starts after that header.
    telemetry_serializer_lora(&packet, buff + LORA_RECEIVER_HEADER_SIZE);

    result = lora_TX(buff, sizeof(buff), 200);

    if (result == HAL_OK) {
        seq++;
    }

    return result;


}

HAL_StatusTypeDef lora_tx_gps(const GPSFix *fix, uint8_t flight_state)
{
    if (fix == NULL) return HAL_ERROR;

    static uint8_t seq = 0U;
    uint8_t buff[LORA_RECEIVER_HEADER_SIZE + GPS_PAYLOAD_SIZE] = {0};
    GPSPacket packet = {0};

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = gps_packet;
    packet.header.sequence_number = seq;
    packet.fix = *fix;
    packet.flight_State = flight_state;

    uint32_t age_ms = HAL_GetTick() - fix->last_update_ms;
    packet.age_ms = age_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)age_ms;

    gps_serializer(&packet, buff + LORA_RECEIVER_HEADER_SIZE);
    HAL_StatusTypeDef result = lora_TX(buff, sizeof(buff), 150U);
    if (result == HAL_OK) {
        seq++;
    }
    return result;
}

HAL_StatusTypeDef flash_log_telemetry(FlightSensorData *sensorData) {
    if (sensorData == NULL) return HAL_ERROR;
    
    static uint16_t seq = 0;
    uint8_t buff[64] = {0};
    TelemetryPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = telemetry_packet;
    packet.header.sequence_number = seq++;
    packet.sensordata = *sensorData;
    packet.flight_State = sensorData->flight_state;
    packet.timestamp = HAL_GetTick();

    telemetry_serializer_memory(&packet, buff);

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
        uint32_t timestamp_ms;

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
        timestamp_ms = ((uint32_t)buff[56] << 24) | ((uint32_t)buff[57] << 16) |
                       ((uint32_t)buff[58] << 8)  |  (uint32_t)buff[59];
 


         printf("[%lu] t=%lums alt=%.2f pres=%.2f temp=%.2f velocity=%.2f | "
                "hg=%.1f,%.1f,%.1f | "
                "imu=%.1f,%.1f,%.1f | "
                "gy=%.1f,%.1f,%.1f | state=%u\r\n",
                i, timestamp_ms, altitude, pressure, temperature, velocity,
                x_mg, y_mg, z_mg,
                x_mg_IMU, y_mg_IMU, z_mg_IMU,
                x_gy, y_gy, z_gy,
                state);
    }

    return HAL_OK;
}

#endif

HAL_StatusTypeDef lora_rx_command(void) {
    if (command_rx_pending != 0U) {
        return HAL_BUSY;
    }

    memset(command_rx_buffer, 0, sizeof(command_rx_buffer));
    command_rx_length = 0U;

    HAL_StatusTypeDef result = lora_RX(command_rx_buffer,
                                        &command_rx_length,
                                        sizeof(command_rx_buffer),
                                        150U);
    if (result == HAL_OK) {
        command_rx_pending = 1U;
    }

    return result;
}

HAL_StatusTypeDef lora_rx_command_service(void) {
    if (command_rx_pending == 0U) {
        return HAL_OK;
    }

    HAL_StatusTypeDef result = lora_get_status();
    if (result == HAL_BUSY) {
        return HAL_BUSY;
    }

    command_rx_pending = 0U;
    if (result == HAL_TIMEOUT) {
        return HAL_OK;
    }
    if (result != HAL_OK) {
        return result;
    }
    if (command_rx_length != sizeof(command_rx_buffer)) {
        return HAL_ERROR;
    }
    
#ifdef DEBUG
        printf("RX ok: %02X %02X id=%02X ch=%02X auth=%02X\r\n",
            command_rx_buffer[4], command_rx_buffer[5], command_rx_buffer[7],
            command_rx_buffer[8], command_rx_buffer[10]);
#endif
        

    uint8_t cmd_id;
    uint8_t channel;
    if(command_deserializer(command_rx_buffer + LORA_RECEIVER_HEADER_SIZE,
                            &cmd_id, &channel)) {
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
                    default:
                        break;
                }
                    
                break;
                  
            case CMD_DISARM:
                FSM_disarm();
                break;

            default:
                break;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef lora_tx_continuity() {
    
    static uint8_t seq = 0;
    /* Buffer needs to be inserted due to groundstation driver code  4 bytes */
    uint8_t buff[LORA_RECEIVER_HEADER_SIZE + CONTINUITY_PAYLOAD_SIZE] = {0};
    
    HAL_StatusTypeDef result;
    ContinuityPacket packet;

    packet.header.sync_word = 0xAA;
    packet.header.packet_type = continuity_packet;
    packet.header.sequence_number = seq;

    packet.main = pyro_check_main();
    packet.drogue = pyro_check_drogue();
    
    // Four leading zero bytes are the receiver-routing header used by this protocol.
    // The application payload starts after that header.
    continuity_serializer(&packet, buff + LORA_RECEIVER_HEADER_SIZE);

    result = lora_TX(buff, sizeof(buff), 100);

    if (result == HAL_OK) {
        seq++;
    }

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
