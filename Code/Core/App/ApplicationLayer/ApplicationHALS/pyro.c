#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "flight_state.h"
#include "odin_app.h"
#include "pyro.h"

LOG_MODULE_REGISTER(odin_pyro, LOG_LEVEL_INF);

#define ODIN_IO_NODE DT_NODELABEL(odin_io)

static const struct gpio_dt_spec drogue_output =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, drogue_pyro_gpios);
static const struct gpio_dt_spec main_output =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, main_pyro_gpios);
static const struct gpio_dt_spec aux_output =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, aux_pyro_gpios);
static const struct gpio_dt_spec buzzer_output =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, buzzer_gpios);

static const struct adc_dt_spec drogue_continuity =
	ADC_DT_SPEC_GET_BY_NAME(ODIN_IO_NODE, drogue_continuity);
static const struct adc_dt_spec main_continuity =
	ADC_DT_SPEC_GET_BY_NAME(ODIN_IO_NODE, main_continuity);

K_MUTEX_DEFINE(continuity_lock);

static void drogue_off(struct k_work *work);
static void main_off(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(drogue_off_work, drogue_off);
K_WORK_DELAYABLE_DEFINE(main_off_work, main_off);

static void drogue_off(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)gpio_pin_set_dt(&drogue_output, 0);
}

static void main_off(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)gpio_pin_set_dt(&main_output, 0);
}

void odin_pyro_all_safe(void)
{
	(void)gpio_pin_set_dt(&drogue_output, 0);
	(void)gpio_pin_set_dt(&main_output, 0);
	(void)gpio_pin_set_dt(&aux_output, 0);
	(void)gpio_pin_set_dt(&buzzer_output, 0);
}

int odin_pyro_init(void)
{
	const struct gpio_dt_spec *outputs[] = {
		&drogue_output, &main_output, &aux_output, &buzzer_output,
	};

	for (size_t i = 0U; i < ARRAY_SIZE(outputs); ++i) {
		if (!gpio_is_ready_dt(outputs[i])) {
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(outputs[i], GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	if (!adc_is_ready_dt(&drogue_continuity) ||
	    !adc_is_ready_dt(&main_continuity)) {
		return -ENODEV;
	}
	int ret = adc_channel_setup_dt(&drogue_continuity);
	if (ret == 0) {
		ret = adc_channel_setup_dt(&main_continuity);
	}
	odin_pyro_all_safe();
	return ret;
}

static uint8_t continuity_read(const struct adc_dt_spec *channel)
{
	int16_t raw = 0;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	if (adc_sequence_init_dt(channel, &sequence) != 0) {
		return 0U;
	}

	k_mutex_lock(&continuity_lock, K_FOREVER);
	int ret = adc_read_dt(channel, &sequence);
	k_mutex_unlock(&continuity_lock);
	return (ret == 0 && raw > PYRO_CONTINUITY_THRESHOLD) ? 1U : 0U;
}

uint8_t pyro_check_drogue(void)
{
	return continuity_read(&drogue_continuity);
}

uint8_t pyro_check_main(void)
{
	return continuity_read(&main_continuity);
}

static void fire_drogue(void)
{
	(void)gpio_pin_set_dt(&drogue_output, 1);
	(void)k_work_reschedule(&drogue_off_work,
				K_MSEC(PYRO_FIRE_DURATION_MS));
}

static void fire_main(void)
{
	(void)gpio_pin_set_dt(&main_output, 1);
	(void)k_work_reschedule(&main_off_work,
				K_MSEC(PYRO_FIRE_DURATION_MS));
}

void pyro_fire_drogue(void)
{
	if (FSM_get_state() == STATE_APOGEE) {
		fire_drogue();
	}
}

void pyro_fire_main(void)
{
	FlightState_t state = FSM_get_state();

	if (state == STATE_APOGEE || state == STATE_DROGUE) {
		fire_main();
	}
}

void pyro_fire_drogue_ground(void)
{
	if (FSM_get_state() == STATE_PAD) {
		fire_drogue();
	}
}

void pyro_fire_main_ground(void)
{
	if (FSM_get_state() == STATE_PAD) {
		fire_main();
	}
}

static bool still_on_pad(void)
{
	FlightSensorData sample;

	return odin_snapshot_get(&sample) && sample.flight_state == STATE_PAD;
}

void odin_indicator_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	uint32_t next_check = k_uptime_get_32();

	for (;;) {
		uint32_t now = k_uptime_get_32();
		if (!still_on_pad()) {
			(void)gpio_pin_set_dt(&buzzer_output, 0);
			next_check = now + 1000U;
			k_msleep(100);
			continue;
		}

		if ((int32_t)(now - next_check) < 0) {
			k_sleep(K_TIMEOUT_ABS_MS(next_check));
			continue;
		}
		next_check += 1000U;
		if ((int32_t)(now - next_check) >= 0) {
			next_check = now + 1000U;
		}

		uint8_t beep_count = pyro_check_drogue() + pyro_check_main();
		for (uint8_t i = 0U; i < beep_count && still_on_pad(); ++i) {
			(void)gpio_pin_set_dt(&buzzer_output, 1);
			k_msleep(150);
			(void)gpio_pin_set_dt(&buzzer_output, 0);
			k_msleep(150);
		}
	}
}
