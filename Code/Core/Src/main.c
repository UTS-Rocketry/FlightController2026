#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "flight_state.h"
#include "kalman.h"
#include "odin_app.h"
#include "pyro.h"

LOG_MODULE_REGISTER(odin_main, LOG_LEVEL_INF);

K_EVENT_DEFINE(odin_events);
K_MSGQ_DEFINE(odin_log_queue, sizeof(FlightSensorData), 64, 4);
K_MSGQ_DEFINE(odin_command_queue, sizeof(OdinCommand), 8, 1);
K_MUTEX_DEFINE(snapshot_lock);

atomic_t odin_sensor_heartbeat_ms;
atomic_t odin_log_drop_count;

static FlightSensorData latest_sample;
static bool latest_sample_valid;

#define WDT_NODE DT_NODELABEL(iwdg)
BUILD_ASSERT(DT_NODE_HAS_STATUS(WDT_NODE, okay), "iwdg must be enabled");
static const struct device *const watchdog = DEVICE_DT_GET(WDT_NODE);
static int watchdog_channel = -1;

static void flight_thread(void *, void *, void *);

#if defined(CONFIG_ODIN_HIL_SIM)
#define ODIN_SIM_REPORT_PERIOD_MS 1000U

static const char *sim_state_name(FlightState_t state)
{
	switch (state) {
	case STATE_IDLE:
		return "IDLE";
	case STATE_PAD:
		return "PAD";
	case STATE_BOOST:
		return "BOOST";
	case STATE_COAST:
		return "COAST";
	case STATE_APOGEE:
		return "APOGEE";
	case STATE_DROGUE:
		return "DROGUE";
	case STATE_PARAFOIL:
		return "PARAFOIL";
	case STATE_LAND:
		return "LAND";
	default:
		return "UNKNOWN";
	}
}
#endif

void odin_wait_for_start(void)
{
	(void)k_event_wait(&odin_events, ODIN_EVENT_START, false, K_FOREVER);
}

void odin_snapshot_publish(const FlightSensorData *sample)
{
	k_mutex_lock(&snapshot_lock, K_FOREVER);
	latest_sample = *sample;
	latest_sample_valid = true;
	k_mutex_unlock(&snapshot_lock);
}

bool odin_snapshot_get(FlightSensorData *sample)
{
	bool valid;

	if (sample == NULL) {
		return false;
	}

	k_mutex_lock(&snapshot_lock, K_FOREVER);
	valid = latest_sample_valid;
	if (valid) {
		*sample = latest_sample;
	}
	k_mutex_unlock(&snapshot_lock);
	return valid;
}

void odin_log_enqueue(const FlightSensorData *sample)
{
	FlightSensorData discarded;

	if (k_msgq_put(&odin_log_queue, sample, K_NO_WAIT) == 0) {
		return;
	}

	/* Never delay the flight thread. Keep the most recent 2.56 seconds. */
	(void)k_msgq_get(&odin_log_queue, &discarded, K_NO_WAIT);
	if (k_msgq_put(&odin_log_queue, sample, K_NO_WAIT) != 0) {
		LOG_WRN("log queue remained full");
	}
	atomic_inc(&odin_log_drop_count);
}

void odin_command_submit(uint8_t id, uint8_t channel)
{
	OdinCommand command = {.id = id, .channel = channel};

	if (k_msgq_put(&odin_command_queue, &command, K_NO_WAIT) != 0) {
		LOG_WRN("command queue full; command %u dropped", id);
	}
}

static uint32_t advance_period(uint32_t deadline, uint32_t period,
			       uint32_t now)
{
	do {
		deadline += period;
	} while ((int32_t)(now - deadline) >= 0);
	return deadline;
}

static void process_commands(void)
{
	OdinCommand command;

	while (k_msgq_get(&odin_command_queue, &command, K_NO_WAIT) == 0) {
		switch (command.id) {
		case ODIN_COMMAND_ARM:
			FSM_arm();
			break;
		case ODIN_COMMAND_DISARM:
			FSM_disarm();
			break;
		case ODIN_COMMAND_FIRE:
			if (FSM_get_state() != STATE_PAD) {
				break;
			}
			if (command.channel == 1U) {
				pyro_fire_drogue_ground();
			} else if (command.channel == 2U) {
				pyro_fire_main_ground();
			}
			FSM_disarm();
			break;
		default:
			LOG_WRN("unknown command %u", command.id);
			break;
		}
	}
}

static void flight_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	FlightSensorData sample = {0};
	uint32_t now;
	uint32_t last_imu;
	uint32_t next_imu;
	uint32_t next_baro;
	uint32_t next_log;
#if defined(CONFIG_ODIN_HIL_SIM)
	uint32_t sim_start;
	uint32_t next_sim_report;
	int sim_previous_state = -1;
#endif

	odin_wait_for_start();
	now = k_uptime_get_32();
	last_imu = now - ODIN_IMU_PERIOD_MS;
	next_imu = now;
	next_baro = now;
	next_log = now;
#if defined(CONFIG_ODIN_HIL_SIM)
	sim_start = now;
	next_sim_report = now;
#endif

	for (;;) {
		now = k_uptime_get_32();
		process_commands();

		bool imu_fresh = false;
		bool baro_fresh = false;

		if ((int32_t)(now - next_imu) >= 0) {
			float dt = (float)(now - last_imu) / 1000.0f;
			last_imu = now;
			next_imu = advance_period(next_imu, ODIN_IMU_PERIOD_MS, now);

			if (odin_sensors_read_imu(&sample) == 0) {
				kalman_predict(sample.z_mg_IMU, dt);
				imu_fresh = true;
			}
		}

		now = k_uptime_get_32();
		if ((int32_t)(now - next_baro) >= 0) {
			next_baro = advance_period(next_baro, ODIN_BARO_PERIOD_MS, now);
			if (odin_sensors_read_baro(&sample) == 0) {
				kalman_update(sample.altitude);
				baro_fresh = true;
			}
		}

		sample.kalman_altitude = kalman_get_altitude();
		sample.kalman_velocity = kalman_get_velocity();

		if (imu_fresh || baro_fresh) {
			(void)FSM_update(&sample, imu_fresh, baro_fresh);
		}
		sample.flight_state = (uint8_t)FSM_get_state();
		odin_snapshot_publish(&sample);

#if defined(CONFIG_ODIN_HIL_SIM)
		FlightState_t sim_state = FSM_get_state();

		if ((int)sim_state != sim_previous_state) {
			LOG_INF("SIM state -> %s at %d m", sim_state_name(sim_state),
				(int)sample.altitude);
			sim_previous_state = (int)sim_state;
		}

		now = k_uptime_get_32();
		if ((int32_t)(now - next_sim_report) >= 0) {
			next_sim_report = advance_period(next_sim_report,
				ODIN_SIM_REPORT_PERIOD_MS, now);
			LOG_INF("SIM t=%u s state=%s alt=%d m est=%d m vel=%d m/s",
				(unsigned int)((now - sim_start) / 1000U),
				sim_state_name(sim_state), (int)sample.altitude,
				(int)sample.kalman_altitude,
				(int)sample.kalman_velocity);
		}
#endif

		now = k_uptime_get_32();
		if ((int32_t)(now - next_log) >= 0) {
			next_log = advance_period(next_log, ODIN_LOG_PERIOD_MS, now);
			if (FSM_get_state() >= STATE_PAD) {
				odin_log_enqueue(&sample);
			}
		}

		atomic_set(&odin_sensor_heartbeat_ms, (atomic_val_t)now);
		k_sleep(K_TIMEOUT_ABS_MS(next_imu));
	}
}

int odin_watchdog_init(void)
{
	if (!device_is_ready(watchdog)) {
		return -ENODEV;
	}

	const struct wdt_timeout_cfg timeout = {
		.window = {.min = 0U, .max = 2050U},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	watchdog_channel = wdt_install_timeout(watchdog, &timeout);
	if (watchdog_channel < 0) {
		return watchdog_channel;
	}

	int ret = wdt_setup(watchdog, WDT_OPT_PAUSE_HALTED_BY_DBG);

	if (ret == -ENOTSUP) {
		/* Some STM32 watchdog instances cannot pause while the debugger halts. */
		ret = wdt_setup(watchdog, 0U);
	}

	return ret;
}

void odin_watchdog_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	for (;;) {
		uint32_t now = k_uptime_get_32();
		uint32_t heartbeat = (uint32_t)atomic_get(&odin_sensor_heartbeat_ms);

		if (watchdog_channel >= 0 && (now - heartbeat) <= 250U) {
			(void)wdt_feed(watchdog, watchdog_channel);
		}
		k_msleep(500);
	}
}

int main(void)
{
	int ret;
	bool sensors_ok = false;

	LOG_INF("ODIN Zephyr RTOS starting");
#if defined(CONFIG_ODIN_HIL_SIM)
	LOG_WRN("HIL SIMULATION BUILD: profile sensors active; pyro locked out");
#endif

	ret = odin_pyro_init();
	if (ret != 0) {
		LOG_ERR("pyro IO init failed: %d", ret);
		return ret;
	}

	ret = odin_sensors_init();
	if (ret != 0) {
		LOG_ERR("flight sensor init failed: %d", ret);
	} else {
		sensors_ok = true;
	}

	ret = odin_storage_init();
	if (ret != 0) {
		LOG_ERR("flash logging disabled: %d", ret);
	}
	ret = odin_radio_init();
	if (ret != 0) {
		LOG_ERR("radio disabled: %d", ret);
	}
	ret = odin_gps_init();
	if (ret != 0) {
		LOG_ERR("GPS disabled: %d", ret);
	}
	ret = odin_can_init();
	if (ret != 0) {
		LOG_ERR("CAN disabled: %d", ret);
	}

	kalman_init();
	FSM_init();
	atomic_set(&odin_sensor_heartbeat_ms, (atomic_val_t)k_uptime_get_32());

	ret = odin_watchdog_init();
	if (ret != 0) {
		LOG_ERR("watchdog init failed: %d", ret);
	}

	if (!sensors_ok) {
		odin_pyro_all_safe();
		LOG_ERR("critical sensors unavailable; scheduler not released");
		return -ENODEV;
	}

#if defined(CONFIG_ODIN_SIM_AUTO_START)
	FSM_arm();
	LOG_WRN("HIL auto-start armed; profile begins with scheduler release");
#endif

	(void)k_event_set(&odin_events, ODIN_EVENT_START);
	LOG_INF("scheduler released: IMU 100 Hz, baro/log 25 Hz");
	return 0;
}

K_THREAD_DEFINE(odin_flight, 3072, flight_thread, NULL, NULL, NULL,
		1, 0, 0);
K_THREAD_DEFINE(odin_watchdog, 768, odin_watchdog_thread, NULL, NULL, NULL,
		2, 0, 0);
K_THREAD_DEFINE(odin_gps, 2048, odin_gps_thread, NULL, NULL, NULL,
		4, 0, 0);
K_THREAD_DEFINE(odin_can, 1024, odin_can_thread, NULL, NULL, NULL,
		5, 0, 0);
K_THREAD_DEFINE(odin_radio, 2560, odin_radio_thread, NULL, NULL, NULL,
		6, 0, 0);
K_THREAD_DEFINE(odin_storage, 2048, odin_storage_thread, NULL, NULL, NULL,
		7, 0, 0);
K_THREAD_DEFINE(odin_indicator, 1024, odin_indicator_thread, NULL, NULL, NULL,
		8, 0, 0);
