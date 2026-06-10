#include "Lora_App.h"
#include "LoRa.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "packets.h"

extern SPI_HandleTypeDef hspi2;
static LORA_CONFIG_TYPEDEF lora_t;
static SX1276_HandleTypedef sx_t;
volatile uint8_t lora_tx_done_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == LoRaDIO0_Pin) {
        lora_tx_done_flag = 1;
    }
}

/* This function Stores all the LoRa radio functions if you change values here make sure the values
   at the ground station are the same or it wont work together */
HAL_StatusTypeDef lora_App_Init() {

    /* Use a struct to organize and set values so it is portable */
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