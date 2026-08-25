#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H
#include "flight_sensors.h"
#include <stdint.h>


typedef enum{
    STATE_IDLE = 0,
    STATE_PAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_APOGEE,
    STATE_DROGUE,
    STATE_PARAFOIL,
    STATE_LAND
} FlightState_t;

typedef struct {
    FlightState_t state;        // current state
    uint32_t state_entry_time;  // HAL_GetTick() when we entered this state
    float apogee_alt;           // recorded peak altitude
    float prev_altitude;        // for velocity estimate
    float vert_velocity;        // m/s, derived from baro
    uint8_t armed;              // set when arm command received
    uint8_t drogue_fired;       // pyro flags
    uint8_t main_fired;
    uint8_t entry;              // true on first tick of a new state
    
    uint8_t launch_count;
    uint8_t burnout_count;
    uint8_t apogee_count;        
    uint8_t main_alt_count;
    uint8_t main_backup_count;

} FSM_Context_t;

HAL_StatusTypeDef FSM_update(FlightSensorData *sensorData, uint8_t imu_read, uint8_t baro_read);
void FSM_init(void);
FlightState_t FSM_get_state(void);

void FSM_arm(void);
void FSM_disarm(void);

#endif