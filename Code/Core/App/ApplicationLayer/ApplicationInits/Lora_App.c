#include "Lora_App.h"


volatile uint8_t lora_tx_done_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DIO0_Pin) {
        lora_tx_done_flag = 1;
    }
}
