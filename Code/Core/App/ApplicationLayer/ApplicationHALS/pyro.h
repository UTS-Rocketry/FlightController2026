#ifndef PYRO_H
#define PYRO_H
#include <stdint.h>

#define PYRO_CONTINUITY_THRESHOLD   3000
#define PYRO_FIRE_DURATION_MS       500

void pyro_init(void);                  
void pyro_fire_drogue(void);           
void pyro_fire_main(void);          
void pyro_fire_aux(void);              
uint8_t pyro_check_drogue(void);          
uint8_t pyro_check_main(void);            
void pyro_service(void);
void pyro_fire_drogue_ground(void);
void pyro_fire_main_ground(void);

#endif