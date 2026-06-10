#include "pyro.h"
#include "flight_state.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static uint32_t drogue_fire_start = 0;
static uint32_t main_fire_start = 0;
static uint32_t aux_fire_start = 0;



void pyro_init(void) {
    
    HAL_GPIO_WritePin(DrogueIgnite_GPIO_Port, DrogueIgnite_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port,   PyroIgnite_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AuxIgnite_GPIO_Port,    AuxIgnite_Pin,    GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BuzzerControl_GPIO_Port, BuzzerControl_Pin, GPIO_PIN_RESET);


}



uint8_t pyro_check_drogue(void) {
    HAL_StatusTypeDef st;

    HAL_ADC_Start(&hadc1);
    st = HAL_ADC_PollForConversion(&hadc1, 10);  // 10mss timeout
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    #ifdef DEBUG
        printf("drogue ADC=%lu thresh=%d\r\n", val, (int)PYRO_CONTINUITY_THRESHOLD);
    #endif

    return (st == HAL_OK && val > PYRO_CONTINUITY_THRESHOLD) ? 1 : 0;
}          
uint8_t pyro_check_main(void) {
    HAL_StatusTypeDef st;

    HAL_ADC_Start(&hadc2);
    st = HAL_ADC_PollForConversion(&hadc2, 10);  // 10ms timeout
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    #ifdef DEBUG
        printf("Main ADC=%lu thresh=%d\r\n", val, (int)PYRO_CONTINUITY_THRESHOLD);
    #endif
    return (st == HAL_OK && val > PYRO_CONTINUITY_THRESHOLD) ? 1 : 0;
}

void pyro_fire_drogue(void) {

    if (FSM_get_state() != STATE_APOGEE) return;
    HAL_GPIO_WritePin(DrogueIgnite_GPIO_Port, DrogueIgnite_Pin, GPIO_PIN_SET);
    drogue_fire_start = HAL_GetTick();
   
}     
void pyro_fire_main(void) {
    
    FlightState_t s = FSM_get_state();
    if (s != STATE_APOGEE && s != STATE_DROGUE) return;
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port, PyroIgnite_Pin, GPIO_PIN_SET);
    main_fire_start = HAL_GetTick();
   
}

void pyro_fire_drogue_ground(void) {
    if (FSM_get_state() != STATE_PAD) return;   // armed-on-pad only
    HAL_GPIO_WritePin(DrogueIgnite_GPIO_Port, DrogueIgnite_Pin, GPIO_PIN_SET);
    drogue_fire_start = HAL_GetTick();
}

void pyro_fire_main_ground(void) {
    if (FSM_get_state() != STATE_PAD) return;
    HAL_GPIO_WritePin(PyroIgnite_GPIO_Port, PyroIgnite_Pin, GPIO_PIN_SET);
    main_fire_start = HAL_GetTick();
}

/*
void pyro_fire_aux(void) {
    HAL_GPIO_WritePin(AuxIgnite_GPIO_Port, AuxIgnite_Pin, GPIO_PIN_SET);
    aux_fire_start = HAL_GetTick();
}
*/

void pyro_service(void) {
    uint32_t now = HAL_GetTick();
    if (drogue_fire_start && now - drogue_fire_start >= PYRO_FIRE_DURATION_MS) {
        HAL_GPIO_WritePin(DrogueIgnite_GPIO_Port, DrogueIgnite_Pin, GPIO_PIN_RESET);
        drogue_fire_start = 0;
    }
    if (main_fire_start && now - main_fire_start >= PYRO_FIRE_DURATION_MS) {
        HAL_GPIO_WritePin(PyroIgnite_GPIO_Port, PyroIgnite_Pin, GPIO_PIN_RESET);
        main_fire_start = 0;
    }
    
    /*if (aux_fire_start && now - aux_fire_start >= PYRO_FIRE_DURATION_MS) {
        HAL_GPIO_WritePin(AuxIgnite_GPIO_Port, AuxIgnite_Pin, GPIO_PIN_RESET);
        aux_fire_start = 0;
    }*/
}
