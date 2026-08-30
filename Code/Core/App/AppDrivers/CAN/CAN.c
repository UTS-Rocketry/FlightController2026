#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "CAN.h"
#include "flight_state.h"
#include "odin_app.h"

LOG_MODULE_REGISTER(odin_can, LOG_LEVEL_INF);

#define CAN_NODE DT_CHOSEN(zephyr_canbus)
#define ODIN_NODE_ID 0x01U
#define HEARTBEAT_MESSAGE 0x01U
#define ODIN_CAN_ID(node, message) (((node) << 7) | (message))

static const struct device *const can_device = DEVICE_DT_GET(CAN_NODE);
static bool can_enabled;

int odin_can_init(void)
{
	if (!device_is_ready(can_device)) {
		return -ENODEV;
	}
	int ret = can_set_mode(can_device, CAN_MODE_NORMAL);
	if (ret == 0) {
		ret = can_start(can_device);
	}
	if (ret == 0) {
		can_enabled = true;
		LOG_INF("CAN2 started at 500 kbit/s");
	}
	return ret;
}

void odin_can_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	uint32_t deadline = k_uptime_get_32();

	for (;;) {
		if (!can_enabled) {
			k_sleep(K_FOREVER);
		}

		FlightSensorData sample;
		if (odin_snapshot_get(&sample) && sample.flight_state <= STATE_PAD) {
			const struct can_frame heartbeat = {
				.id = ODIN_CAN_ID(ODIN_NODE_ID, HEARTBEAT_MESSAGE),
				.dlc = 0U,
				.flags = 0U,
			};
			int ret = can_send(can_device, &heartbeat, K_MSEC(20), NULL, NULL);
			if (ret != 0) {
				LOG_WRN("heartbeat TX failed: %d", ret);
			}
		}

		uint32_t now = k_uptime_get_32();
		do {
			deadline += ODIN_CAN_HEARTBEAT_PERIOD_MS;
		} while ((int32_t)(now - deadline) >= 0);
		k_sleep(K_TIMEOUT_ABS_MS(deadline));
	}
}
