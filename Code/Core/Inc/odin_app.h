#ifndef ODIN_APP_H
#define ODIN_APP_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "GPS.h"
#include "flight_sensors.h"

#define ODIN_EVENT_START BIT(0)

#define ODIN_IMU_PERIOD_MS 10U
#define ODIN_BARO_PERIOD_MS 40U
#define ODIN_LOG_PERIOD_MS 40U
#define ODIN_RADIO_GUARD_MS 25U
#define ODIN_COMMAND_RX_PERIOD_MS 400U
#define ODIN_FLIGHT_TX_PERIOD_MS 300U
#define ODIN_GPS_TX_PERIOD_MS 1000U
#define ODIN_CONTINUITY_PERIOD_MS 2000U
#define ODIN_CAN_HEARTBEAT_PERIOD_MS 500U

typedef enum {
	ODIN_COMMAND_ARM = 1,
	ODIN_COMMAND_FIRE = 2,
	ODIN_COMMAND_DISARM = 3,
} OdinCommandId;

typedef struct {
	uint8_t id;
	uint8_t channel;
} OdinCommand;

extern struct k_event odin_events;
extern struct k_msgq odin_log_queue;
extern struct k_msgq odin_command_queue;
extern atomic_t odin_sensor_heartbeat_ms;
extern atomic_t odin_log_drop_count;

void odin_wait_for_start(void);
void odin_snapshot_publish(const FlightSensorData *sample);
bool odin_snapshot_get(FlightSensorData *sample);
void odin_log_enqueue(const FlightSensorData *sample);
void odin_command_submit(uint8_t id, uint8_t channel);

int odin_gps_init(void);
bool odin_gps_get_fix(GPSFix *fix);
void odin_gps_thread(void *, void *, void *);

int odin_radio_init(void);
void odin_radio_thread(void *, void *, void *);

int odin_storage_init(void);
void odin_storage_thread(void *, void *, void *);

int odin_can_init(void);
void odin_can_thread(void *, void *, void *);

int odin_watchdog_init(void);
void odin_watchdog_thread(void *, void *, void *);

#endif
