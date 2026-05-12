#include "Lora_App.h"
#include "LoRa.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"

extern SPI_HandleTypeDef hspi2;
static LORA_CONFIG_TYPEDEF lora_t;
static SX1276_HandleTypedef sx_t;
volatile uint8_t lora_tx_done_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == LoRaDIO0_Pin) {
        lora_tx_done_flag = 1;
    }
}


HAL_StatusTypeDef lora_App_Init() {

    sx_t.hspi = &hspi2;
    sx_t.nss_port = LoRaNssPin_GPIO_Port;
    sx_t.nss_pin = LoRaNssPin_Pin;
    sx_t.rst_port = LoRaResetPin_GPIO_Port;
    sx_t.rst_pin = LoRaResetPin_Pin;

    lora_t.frequency = 915000000;
    lora_t.spreading_Factor = 7;
    lora_t.bandwidth = 125000;

    lora_t.coding_Rate     = 5; 
    lora_t.tx_Power        = 17;      
    lora_t.use_Pa_Boost    = 1;
    lora_t.enable_CRC      = 1;
    lora_t.implicit_header = 0;
    lora_t.sync_Word       = 0x12;
    lora_t.preamble_Length = 8;

    
    
    return lora_init(&sx_t, &lora_t);
    
}

void lora_telemetry_serializer(TelemetryPacket *packet, uint8_t *buff) {

    buff[0] = packet->header.sync_word;
    buff[1] = packet->header.packet_type;
    buff[2] = packet->header.sequence_number;

    /* start of telemetry */
    uint32_t raw = 0;
    
    /*altitude*/
    memcpy(&raw, &packet->sensordata.altitude, 4);

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

    buff[51] = packet->flight_State;

    uint16_t crc = crc16(0, buff, 51);

    buff[52] =  (crc >> 8) & 0xFF;
    buff[53] = (crc) & 0xFF;



}