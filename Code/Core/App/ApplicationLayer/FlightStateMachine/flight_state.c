#include "flight_state.h"
#include "pyro.h"
#include "stm32f4xx_hal.h"
#include "telemetry.h"
#include "flight_config.h"
#include <stdint.h>
#include <math.h>
#include <string.h>


static FSM_Context_t ctx;

static void FSM_transition(FlightState_t new_state);
static uint8_t drogue_fail_check(FSM_Context_t *ctx, float current_alt, uint32_t now_ms);

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
                if(sensorData->kalman_velocity < APOGEE_VELOCITY_THRESHOLD && HAL_GetTick() - ctx.state_entry_time > SONIC_TIMOUT_MS){
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
                

                pyro_fire_drogue();
                ctx.drogue_fired = 1;

                if (ctx.apogee_alt < MAIN_DEPLOY_ALT_M) {
                    ctx.main_pending = 1;
                } else {
                    FSM_transition(STATE_DROGUE);
                }

            }

            if (ctx.main_pending && !ctx.main_fired && 
                (HAL_GetTick() - ctx.state_entry_time) >= UNDERSHOOT_TIME_DELAY) {
                    
                pyro_fire_main();
                ctx.main_fired = 1;
                ctx.main_pending = 0;
                FSM_transition(STATE_PARAFOIL);

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

                if (drogue_fail_check(&ctx, sensorData->altitude, HAL_GetTick()) && ctx.main_fired != 1) {
                    ctx.main_backup_count++;
                } else {
                    ctx.main_backup_count = 0;
                }
                if (ctx.main_backup_count >= MAIN_BACKUP_CONFIRM_SAMPLES) {
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


static uint8_t drogue_fail_check(FSM_Context_t *ctx, float current_alt, uint32_t now_ms) {
    uint8_t oldest_idx = ctx->hist_index;   // about to be overwritten == oldest sample
    uint8_t have_full_window = ctx->hist_filled;

    // Store current sample, advance ring buffer
    ctx->alt_history[ctx->hist_index]  = current_alt;
    ctx->time_history[ctx->hist_index] = now_ms;
    ctx->hist_index = (ctx->hist_index + 1) % DROGUE_FAIL_WINDOW_SAMPLES;
    if (ctx->hist_index == 0) ctx->hist_filled = 1;

    if (!have_full_window) return 0;   // not enough history yet, don't judge early

    float dt = (now_ms - ctx->time_history[oldest_idx]) / 1000.0f;
    if (dt <= 0.0f) return 0;

    float descent_rate = (current_alt - ctx->alt_history[oldest_idx]) / dt;
    return (descent_rate < DROGUE_FAIL_RATE_MPS) ? 1 : 0;
}   