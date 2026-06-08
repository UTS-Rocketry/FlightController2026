#ifndef INDICATORS_H
#define INDICATORS_H
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t on_ms, off_ms, beep_count, pause_ms;
    uint8_t active, is_on, in_pause;
    uint32_t beeps_done, last_toggle;
} Buzzer_t;

extern Buzzer_t buzzer;

void buzzer_init(void);
void buzzer_Set(uint32_t on, uint32_t off, uint32_t count, uint32_t pause);
void buzzer_Service(void);
void indicators_service(void);

#endif