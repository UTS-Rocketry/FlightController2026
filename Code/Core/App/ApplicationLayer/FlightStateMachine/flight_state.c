#include "flight_state.h"
#include "pyro.h"
#include "stm32f4xx_hal.h"
#include "telemetry.h"
#include "flight_config.h"
#include <stdint.h>
#include <math.h>


static FSM_Context_t ctx;

static void FSM_transition(FlightState_t new_state);

void FSM_init(void) {
    memset(&ctx, 0, sizeof(FSM_Context_t));
    ctx.state = STATE_IDLE;
    ctx.entry = 1;
}

HAL_StatusTypeDef FSM_update(FlightSensorData *sensorData, uint8_t imu_read, uint8_t baro_read) {
    /*********************************************************************** */
    switch(ctx.state) {

        case STATE_IDLE:
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: IDLE\r\n");
                #endif
            }
            
            break;
    /*********************************************************************** */

        case STATE_PAD:
            
            /* This runs the first time state entry */
            if(ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: STATE-PAD\r\n");
                #endif
            }
            
            /* If imu detects more threshold */

            if(imu_read) {
                if(sensorData->z_mg_IMU >= LAUNCH_ACCEL_THRESHOLD_MG) {
                    ctx.launch_count++;
                }
                else {
                    ctx.launch_count = 0;
                }

                if (ctx.launch_count >= LAUNCH_CONFIRM_SAMPLES) {
                    ctx.launch_count = 0;  
                    FSM_transition(STATE_BOOST);
                }
            }

            break;
    /*********************************************************************** */

        case STATE_BOOST:
            
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: BOOST\r\n");
                #endif
            }

            //lock pyro

            // change when accel = around < 2 gs
            if(imu_read) {
                
                if(sensorData->z_mg_IMU <= BURNOUT_ACCEL_THRESHOLD_MG) {
                ctx.burnout_count++;
                
                }
                else {
                    ctx.burnout_count = 0;
                }

                if (ctx.burnout_count >= BURNOUT_CONFIRM_SAMPLES) {
                    ctx.burnout_count = 0;  
                    FSM_transition(STATE_COAST);
                }


            }
            
            if (HAL_GetTick() - ctx.state_entry_time > BOOST_TIMEOUT_MS) {
                FSM_transition(STATE_COAST);
            }

            break;
    
    /*********************************************************************** */
        case STATE_COAST:
            
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: COAST\r\n");
                #endif
            }
            
            
            /*Apogee detected*/

            if(baro_read) {
                if(sensorData->kalman_velocity < APOGEE_VELOCITY_THRESHOLD){
                ctx.apogee_count++;
                }
                else {
                    ctx.apogee_count = 0;
                }
                if(ctx.apogee_count >= APOGEE_CONFIRM_SAMPLES) {
                    ctx.apogee_count = 0; 
                    FSM_transition(STATE_APOGEE);
                }

            }
            
            if(HAL_GetTick() - ctx.state_entry_time > COAST_TIMEOUT_MS) {
                FSM_transition(STATE_APOGEE);
            }
                
            break;
    
    /*********************************************************************** */

        case STATE_APOGEE:
            
            if (ctx.entry) {
                ctx.entry = 0;
                ctx.apogee_alt = sensorData->kalman_altitude;
                
                #ifdef DEBUG
                    printf("FSM: APOGEE\r\n");
                #endif
                

                /* ADD LORA TRANSMISSION */
                /* If apogee is less than Main deployment alt */
                if (ctx.apogee_alt < MAIN_DEPLOY_ALT_M) {
                    
                    pyro_fire_drogue();
                    ctx.drogue_fired = 1;
                    
                    pyro_fire_main();
                    ctx.main_fired = 1;
                    FSM_transition(STATE_PARAFOIL);
                    break;

                } 
                
                pyro_fire_drogue();
                ctx.drogue_fired = 1;
                FSM_transition(STATE_DROGUE);

            }
            //APPOGEE DETECTED FIRE PYRO
            break;

    /*********************************************************************** */        
        case STATE_DROGUE:
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: DROGUE\r\n");
                #endif
                /* ADD LORA TRANSMISSION */
            }
            
            
            if(baro_read) {
                
                if (sensorData->kalman_altitude < MAIN_DEPLOY_ALT_M && ctx.main_fired != 1) {
                ctx.main_alt_count++;
                }
                else {
                    ctx.main_alt_count = 0;
                }

                if(ctx.main_alt_count > MAIN_ALT_CONFIRM_SAMPLES) {
                    pyro_fire_main();
                    ctx.main_fired = 1;
                    FSM_transition(STATE_PARAFOIL);
                }
            }
          
            if (HAL_GetTick() - ctx.state_entry_time > DROGUE_TIMEOUT_MS && ctx.main_fired != 1) {
                /*pyro fire main*/
                pyro_fire_main();
                ctx.main_fired = 1;
                FSM_transition(STATE_PARAFOIL);
            }

            break;

    /*********************************************************************** */

        case STATE_PARAFOIL:
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: PARAFOIL\r\n");
                #endif
                /* ADD LORA TRANSMISSION */
            }

            if(baro_read) {
                if (sensorData->kalman_altitude < LAND_ALT_THRESHOLD_M && 
                    fabsf(sensorData->kalman_velocity) < LAND_VELOCITY_THRESHOLD) {
                    
                    FSM_transition(STATE_LAND);
                }
            }

            if(HAL_GetTick() - ctx.state_entry_time > PARAFOIL_TIMEOUT_MS) {
                FSM_transition(STATE_LAND);
            }

            break;
    
    /*********************************************************************** */

         case STATE_LAND:
            if (ctx.entry) {
                ctx.entry = 0;
                #ifdef DEBUG
                    printf("FSM: LANDED\r\n");
                #endif
                /* ADD LORA TRANSMISSION */
            }
            //LANDED
            break;

    }

    return HAL_OK;

}

static void FSM_transition(FlightState_t new_state) {
    ctx.state = new_state;
    ctx.state_entry_time = HAL_GetTick();
    ctx.entry = 1;
}

FlightState_t FSM_get_state(void) {

    return ctx.state;

}
void FSM_arm(void) {
    if(ctx.state == STATE_IDLE ) {
        FSM_transition(STATE_PAD);
    }
}
void FSM_disarm(void) {
    if(ctx.state == STATE_PAD ) {
        FSM_transition(STATE_IDLE);
    }
}
