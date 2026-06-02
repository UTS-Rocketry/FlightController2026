#include "pyro.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void pyro_init(void) {
    
    HAL_GPIO_WritePin(DrougeIgnite_GPIO_Port, DrougeIgnite_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port,   PyroIgnite_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AuxIgnite_GPIO_Port,    AuxIgnite_Pin,    GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BuzzerControl_GPIO_Port, BuzzerControl_Pin, GPIO_PIN_RESET);


}

uint8_t pyro_check_drogue(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);  // 10ms timeout
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return (val > PYRO_CONTINUITY_THRESHOLD) ? 1 : 0;
}          
uint8_t pyro_check_main(void) {
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);  // 10ms timeout
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    return (val > PYRO_CONTINUITY_THRESHOLD) ? 1 : 0;
}

void pyro_fire_drogue(void) {
    HAL_GPIO_WritePin(DrougeIgnite_GPIO_Port, DrougeIgnite_Pin, GPIO_PIN_SET);
    HAL_Delay(PYRO_FIRE_DURATION_MS);
    HAL_GPIO_WritePin(DrougeIgnite_GPIO_Port, DrougeIgnite_Pin, GPIO_PIN_RESET);
}     
void pyro_fire_main(void) {
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port, PyroIgnite_Pin, GPIO_PIN_SET);
    HAL_Delay(PYRO_FIRE_DURATION_MS);
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port, PyroIgnite_Pin, GPIO_PIN_RESET);
}        
void pyro_fire_aux(void) {
    HAL_GPIO_WritePin(AuxIgnite_GPIO_Port, AuxIgnite_Pin, GPIO_PIN_SET);
    HAL_Delay(PYRO_FIRE_DURATION_MS);
    HAL_GPIO_WritePin(AuxIgnite_GPIO_Port, AuxIgnite_Pin, GPIO_PIN_RESET);
}

void pyro_buzzer_on(void) {
    HAL_GPIO_WritePin(BuzzerControl_GPIO_Port, BuzzerControl_Pin, GPIO_PIN_SET);
}

void pyro_buzzer_off(void) {
    HAL_GPIO_WritePin(BuzzerControl_GPIO_Port, BuzzerControl_Pin, GPIO_PIN_RESET);
}