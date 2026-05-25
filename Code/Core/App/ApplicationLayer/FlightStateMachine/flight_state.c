#include "flight_state.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_def.h"
#include "telemetry.h"
#include "flight_config.h"
#include <stdint.h>


static FSM_Context_t ctx;

static void FSM_transiton(FlightState_t new_state);

void FSM_init() {
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

            flash_log_telemetry(sensorData);
            
            static uint8_t launch_count = 0;
            if(sensorData->z_mg >= LAUNCH_ACCEL_THRESHOLD_MG) {
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
            if(sensorData->z_mg <= BURNOUT_ACCEL_THRESHOLD_MG) {
                burnout_count++;
                
            }
            else {
                burnout_count = 0;
            }

            if (burnout_count >= BURNOUT_CONFIRM_SAMPLES) {
                burnout_count = 0;  
                FSM_transition(STATE_COAST);
            }

            break;
        case STATE_COAST:
            
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: BOOST\r\n");
                /* ADD LORA TRANSMISSION */
            }
            //apogee detected
            
                
            break;
        case STATE_APOGEE:
            
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: APOGEE\r\n");
                /* ADD LORA TRANSMISSION */
            }
            //APPOGEE DETECTED FIRE PYRO
            break;
        case STATE_DROGUE:
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: DROUGE\r\n");
                /* ADD LORA TRANSMISSION */
            }
            //PYRO DEPLOYED
            //WAIT UNTILL CERTAIN HIEGT TO DEPLOY MAIN/PARAFOIL
            break;
        case STATE_PARAFOIL:
            if (ctx.entry) {
                ctx.entry = 0;
                printf("FSM: PARAFOIL\r\n");
                /* ADD LORA TRANSMISSION */
            }
            //PYRO DEPLOYED
            //WAIT UNTIL LAND TO CHANGE STATE
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

static void FSM_transiton(FlightState_t new_state) {
    ctx.state = new_state;
    ctx.state_entry_time = HAL_GetTick();
    ctx.entry = 1;
}