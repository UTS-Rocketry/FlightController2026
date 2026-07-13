#ifndef WATCHDOG_H
#define WATCHDOG_H
#include <stdint.h>
void reset_cause_check(void);
uint16_t reset_get_iwdg_count(void);


#endif