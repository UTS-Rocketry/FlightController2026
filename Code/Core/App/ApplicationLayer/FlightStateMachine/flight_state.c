#include "flight_state.h"
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

HAL_StatusTypeDef FSM_update(FlightSensorData *sensorData) {

    switch(ctx.state) {
        case STATE_IDLE:
                
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: IDLE\r\n");
            }

            /* Time base transition will move to lora arm later */
            if (HAL_GetTick() > ARM_AUTO_DELAY_MS) {
                FSM_transition(STATE_PAD);
            }
            
            break;

        case STATE_PAD:
            
            /* This runs the first time state entry */
            if(ctx.entry) {
                ctx.entry = 0;
                printf("FSM: ARMED\r\n");
            }
            
            /* If imu detects more threshold */
            static uint8_t launch_count = 0;
            if(sensorData->z_mg_IMU >= LAUNCH_ACCEL_THRESHOLD_MG) {
                launch_count++;
                
            }
            else {
                launch_count = 0;
            }

            if (launch_count >= LAUNCH_CONFIRM_SAMPLES) {
                launch_count = 0;  
                FSM_transition(STATE_BOOST);
            }


            break;

        case STATE_BOOST:
            
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: BOOST\r\n");
            }

            //lock pyro

            // change when accel = around < 2 gs
            static uint8_t burnout_count = 0;
            if(sensorData->z_mg_IMU <= BURNOUT_ACCEL_THRESHOLD_MG) {
                burnout_count++;
                
            }
            else {
                burnout_count = 0;
            }

            if (burnout_count >= BURNOUT_CONFIRM_SAMPLES) {
                burnout_count = 0;  
                FSM_transition(STATE_COAST);
            }

            if (HAL_GetTick() - ctx.state_entry_time > BOOST_TIMEOUT_MS) {
                FSM_transition(STATE_COAST);
            }

            break;
        case STATE_COAST:
            
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: COAST\r\n");
                /* ADD LORA TRANSMISSION */
            }
            
            
            /*Apogee detected*/

            static uint8_t apogee_count = 0;
            if(sensorData->kalman_velocity < APOGEE_VELOCITY_THRESHOLD){
                apogee_count++;
            }
            else {
                apogee_count = 0;
            }
            if(apogee_count >= APOGEE_CONFIRM_SAMPLES) {
                apogee_count = 0; 
                FSM_transition(STATE_APOGEE);
            }

            if(HAL_GetTick() - ctx.state_entry_time > COAST_TIMEOUT_MS) {
                FSM_transition(STATE_APOGEE);
            }
            
                
            break;

        case STATE_APOGEE:
            
            if (ctx.entry) {
                ctx.entry = 0;
                ctx.apogee_alt = sensorData->kalman_altitude;
                printf("FSM: APOGEE\r\n");
                /* ADD LORA TRANSMISSION */
                  if (ctx.apogee_alt < MAIN_DEPLOY_ALT_M) {
                // pyro_fire_main();
                ctx.main_fired = 1;
                } else {
                    // pyro_fire_drogue();
                    ctx.drogue_fired = 1;
                }
                FSM_transition(STATE_DROGUE);

            }

            //APPOGEE DETECTED FIRE PYRO
            break;
        case STATE_DROGUE:
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: DROUGE\r\n");
                /* ADD LORA TRANSMISSION */
            }

            if (sensorData->kalman_altitude < MAIN_DEPLOY_ALT_M && ctx.main_fired != 1) {
                /*pyro fire main*/
                ctx.main_fired = 1;
                FSM_transition(STATE_PARAFOIL);
            }
            if (HAL_GetTick() - ctx.state_entry_time > DROGUE_TIMEOUT_MS && ctx.main_fired != 1) {
                /*pyro fire main*/
                ctx.main_fired = 1;
                FSM_transition(STATE_PARAFOIL);
            }

            break;
        case STATE_PARAFOIL:
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: PARAFOIL\r\n");
                /* ADD LORA TRANSMISSION */
            }

           
            if (sensorData->kalman_altitude < LAND_ALT_THRESHOLD_M && 
                fabsf(sensorData->kalman_velocity) < LAND_VELOCITY_THRESHOLD) {
                
                FSM_transition(STATE_LAND);

            }
            if(HAL_GetTick() - ctx.state_entry_time > PARAFOIL_TIMEOUT_MS) {
                FSM_transition(STATE_LAND);
            }

            break;
         case STATE_LAND:
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: LANDED\r\n");
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