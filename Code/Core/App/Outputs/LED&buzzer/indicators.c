#include "indicators.h"
#include "flight_state.h"
#include "pyro.h"
#include "main.h"

Buzzer_t buzzer;

void buzzer_init(void) {
    buzzer.port = BuzzerControl_GPIO_Port;
    buzzer.pin  = BuzzerControl_Pin;
    buzzer.active = 0;
    HAL_GPIO_WritePin(buzzer.port, buzzer.pin, GPIO_PIN_RESET);
}

void buzzer_Set(uint32_t on, uint32_t off, uint32_t count, uint32_t pause) {
    buzzer.on_ms = on;
    buzzer.off_ms = off;
    buzzer.beep_count = count;
    buzzer.pause_ms = pause;
    buzzer.active = (count > 0 || on > 0);
    buzzer.is_on = 0;
    buzzer.beeps_done = 0;
    buzzer.in_pause = 0;
    buzzer.last_toggle = HAL_GetTick();
    HAL_GPIO_WritePin(buzzer.port, buzzer.pin, GPIO_PIN_RESET);
}

void buzzer_Service(void) {
    if (!buzzer.active) return;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - buzzer.last_toggle;

    if (buzzer.in_pause) {
        if (elapsed >= buzzer.pause_ms) {
            buzzer.in_pause = 0;
            buzzer.beeps_done = 0;
            buzzer.last_toggle = now;
        }
        return;
    }

    if (buzzer.is_on) {
        if (elapsed >= buzzer.on_ms) {
            HAL_GPIO_WritePin(buzzer.port, buzzer.pin, GPIO_PIN_RESET);
            buzzer.is_on = 0;
            buzzer.last_toggle = now;
            buzzer.beeps_done++;
            if (buzzer.beep_count && buzzer.beeps_done >= buzzer.beep_count && buzzer.pause_ms) {
                buzzer.in_pause = 1;
            }
        }
    } else {
        if (elapsed >= buzzer.off_ms) {
            HAL_GPIO_WritePin(buzzer.port, buzzer.pin, GPIO_PIN_SET);
            buzzer.is_on = 1;
            buzzer.last_toggle = now;
        }
    }
}

void indicators_service(void) {
    static uint32_t last_cont = 0;
    FlightState_t s = FSM_get_state();

    if (s == STATE_PAD) {
        uint32_t now = HAL_GetTick();
        if (now - last_cont >= 1000) {
            last_cont = now;
            uint32_t beeps = 0;
            if (pyro_check_drogue()) beeps++;
            if (pyro_check_main())   beeps++;
            buzzer_Set(150, 150, beeps, 1500);
        }
    } else if (buzzer.active) {
        buzzer.active = 0;
        HAL_GPIO_WritePin(buzzer.port, buzzer.pin, GPIO_PIN_RESET);
    }

    buzzer_Service();
}