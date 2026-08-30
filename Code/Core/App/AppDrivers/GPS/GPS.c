#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "GPS.h"
#include "odin_app.h"

LOG_MODULE_REGISTER(odin_gps, LOG_LEVEL_INF);

#define GPS_UART_NODE DT_NODELABEL(uart5)
#define ODIN_IO_NODE DT_NODELABEL(odin_io)
#define GPS_SENTENCE_SIZE 128U
#define GPS_MAX_FIELDS 20U

static const struct device *const gps_uart = DEVICE_DT_GET(GPS_UART_NODE);
static const struct gpio_dt_spec gps_reset =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, gps_reset_gpios);

RING_BUF_DECLARE(gps_rx_ring, 256);
K_SEM_DEFINE(gps_rx_sem, 0, 1);
K_MUTEX_DEFINE(gps_fix_lock);

static atomic_t gps_rx_overflow;
static bool gps_enabled;
static bool have_nmea_data;
static GPSFix latest_fix;

static bool checksum_valid(const char *line);
static uint32_t parse_unsigned(const char *text);
static int32_t parse_fixed(const char *text, uint32_t scale);
static bool parse_coordinate(const char *text, char hemisphere,
			     int32_t *degrees_e7);
static uint32_t parse_utc_ms(const char *text);

static void gps_uart_isr(const struct device *device, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t bytes[32];

	while (uart_irq_update(device) != 0 && uart_irq_rx_ready(device) != 0) {
		int received = uart_fifo_read(device, bytes, sizeof(bytes));
		if (received <= 0) {
			break;
		}
		uint32_t stored = ring_buf_put(&gps_rx_ring, bytes, (uint32_t)received);
		if (stored != (uint32_t)received) {
			atomic_set(&gps_rx_overflow, 1);
		}
		k_sem_give(&gps_rx_sem);
	}
}

int odin_gps_init(void)
{
	if (!device_is_ready(gps_uart) || !gpio_is_ready_dt(&gps_reset)) {
		return -ENODEV;
	}
	int ret = gpio_pin_configure_dt(&gps_reset, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	/* Active-low reset: logical active for 10 ms, then release. */
	(void)gpio_pin_set_dt(&gps_reset, 1);
	k_msleep(10);
	(void)gpio_pin_set_dt(&gps_reset, 0);
	k_msleep(10);

	ret = uart_irq_callback_user_data_set(gps_uart, gps_uart_isr, NULL);
	if (ret != 0) {
		return ret;
	}
	uart_irq_rx_enable(gps_uart);
	gps_enabled = true;
	LOG_INF("GPS UART5 enabled at 9600 baud");
	return 0;
}

bool odin_gps_get_fix(GPSFix *fix)
{
	if (fix == NULL || !gps_enabled) {
		return false;
	}

	k_mutex_lock(&gps_fix_lock, K_FOREVER);
	bool valid = have_nmea_data;
	if (valid) {
		*fix = latest_fix;
		if (atomic_get(&gps_rx_overflow) != 0) {
			fix->status_flags |= GPS_STATUS_RX_OVERFLOW;
		}
	}
	k_mutex_unlock(&gps_fix_lock);
	return valid;
}

static void parse_sentence(char *line)
{
	if (!checksum_valid(line)) {
		return;
	}

	char *asterisk = strchr(line, '*');
	if (asterisk == NULL) {
		return;
	}
	*asterisk = '\0';

	char *fields[GPS_MAX_FIELDS] = {0};
	uint8_t count = 0U;
	char *field = line;
	while (field != NULL && count < GPS_MAX_FIELDS) {
		fields[count++] = field;
		char *comma = strchr(field, ',');
		if (comma == NULL) {
			break;
		}
		*comma = '\0';
		field = comma + 1;
	}
	if (count == 0U) {
		return;
	}

	bool is_gga = strcmp(fields[0], "$GPGGA") == 0 ||
		      strcmp(fields[0], "$GNGGA") == 0;
	bool is_rmc = strcmp(fields[0], "$GPRMC") == 0 ||
		      strcmp(fields[0], "$GNRMC") == 0;

	k_mutex_lock(&gps_fix_lock, K_FOREVER);
	if (is_gga && count > 9U) {
		uint8_t quality = (uint8_t)parse_unsigned(fields[6]);
		int32_t latitude;
		int32_t longitude;

		latest_fix.utc_ms = parse_utc_ms(fields[1]);
		latest_fix.fix_quality = quality;
		latest_fix.satellites = (uint8_t)parse_unsigned(fields[7]);
		latest_fix.altitude_mm = parse_fixed(fields[9], 1000U);
		latest_fix.status_flags |= GPS_STATUS_GGA_SEEN;

		if (quality > 0U &&
		    parse_coordinate(fields[2], fields[3][0], &latitude) &&
		    parse_coordinate(fields[4], fields[5][0], &longitude)) {
			latest_fix.latitude_e7 = latitude;
			latest_fix.longitude_e7 = longitude;
			latest_fix.status_flags |= GPS_STATUS_FIX_VALID;
		} else {
			latest_fix.status_flags &= (uint8_t)~GPS_STATUS_FIX_VALID;
		}
		latest_fix.last_update_ms = k_uptime_get_32();
		have_nmea_data = true;
	} else if (is_rmc && count > 8U) {
		bool valid = fields[2][0] == 'A';
		int32_t latitude;
		int32_t longitude;

		latest_fix.utc_ms = parse_utc_ms(fields[1]);
		latest_fix.ground_speed_cms =
			(uint32_t)(((uint64_t)(uint32_t)parse_fixed(fields[7], 1000U) *
			51444U + 500000U) / 1000000U);
		latest_fix.course_cdeg = (uint16_t)parse_fixed(fields[8], 100U);
		if (valid &&
		    parse_coordinate(fields[3], fields[4][0], &latitude) &&
		    parse_coordinate(fields[5], fields[6][0], &longitude)) {
			latest_fix.latitude_e7 = latitude;
			latest_fix.longitude_e7 = longitude;
			latest_fix.status_flags |= GPS_STATUS_RMC_VALID |
						  GPS_STATUS_FIX_VALID;
		} else {
			latest_fix.status_flags &= (uint8_t)~GPS_STATUS_RMC_VALID;
			if (latest_fix.fix_quality == 0U) {
				latest_fix.status_flags &= (uint8_t)~GPS_STATUS_FIX_VALID;
			}
		}
		latest_fix.last_update_ms = k_uptime_get_32();
		have_nmea_data = true;
	}
	k_mutex_unlock(&gps_fix_lock);
}

void odin_gps_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	char sentence[GPS_SENTENCE_SIZE];
	size_t sentence_length = 0U;

	for (;;) {
		if (!gps_enabled) {
			k_sleep(K_FOREVER);
		}
		(void)k_sem_take(&gps_rx_sem, K_FOREVER);

		uint8_t bytes[64];
		uint32_t count;
		while ((count = ring_buf_get(&gps_rx_ring, bytes, sizeof(bytes))) > 0U) {
			for (uint32_t i = 0U; i < count; ++i) {
				char byte = (char)bytes[i];
				if (byte == '$') {
					sentence_length = 0U;
					sentence[sentence_length++] = byte;
				} else if (byte == '\n') {
					if (sentence_length > 0U) {
						sentence[sentence_length] = '\0';
						parse_sentence(sentence);
					}
					sentence_length = 0U;
				} else if (byte != '\r' && sentence_length > 0U) {
					if (sentence_length < GPS_SENTENCE_SIZE - 1U) {
						sentence[sentence_length++] = byte;
					} else {
						sentence_length = 0U;
					}
				}
			}
		}
	}
}

static bool checksum_valid(const char *line)
{
	if (line == NULL || line[0] != '$') {
		return false;
	}
	const char *asterisk = strchr(line, '*');
	if (asterisk == NULL || asterisk[1] == '\0' || asterisk[2] == '\0') {
		return false;
	}

	uint8_t checksum = 0U;
	for (const char *p = line + 1; p < asterisk; ++p) {
		checksum ^= (uint8_t)*p;
	}

	uint8_t received = 0U;
	for (uint8_t i = 1U; i <= 2U; ++i) {
		char c = asterisk[i];
		uint8_t nibble;
		if (c >= '0' && c <= '9') {
			nibble = (uint8_t)(c - '0');
		} else if (c >= 'A' && c <= 'F') {
			nibble = (uint8_t)(c - 'A' + 10);
		} else if (c >= 'a' && c <= 'f') {
			nibble = (uint8_t)(c - 'a' + 10);
		} else {
			return false;
		}
		received = (uint8_t)((received << 4) | nibble);
	}
	return checksum == received;
}

static uint32_t parse_unsigned(const char *text)
{
	uint32_t value = 0U;
	if (text == NULL) {
		return 0U;
	}
	while (*text >= '0' && *text <= '9') {
		value = value * 10U + (uint32_t)(*text - '0');
		++text;
	}
	return value;
}

static int32_t parse_fixed(const char *text, uint32_t scale)
{
	if (text == NULL || *text == '\0') {
		return 0;
	}
	bool negative = false;
	if (*text == '-') {
		negative = true;
		++text;
	} else if (*text == '+') {
		++text;
	}

	uint32_t whole = parse_unsigned(text);
	while (*text >= '0' && *text <= '9') {
		++text;
	}
	uint32_t fraction = 0U;
	uint32_t divisor = 1U;
	if (*text == '.') {
		++text;
		while (*text >= '0' && *text <= '9' && divisor < scale) {
			fraction = fraction * 10U + (uint32_t)(*text - '0');
			divisor *= 10U;
			++text;
		}
	}
	int64_t value = (int64_t)whole * scale;
	if (divisor > 1U) {
		value += ((int64_t)fraction * scale + divisor / 2U) / divisor;
	}
	return negative ? (int32_t)-value : (int32_t)value;
}

static bool parse_coordinate(const char *text, char hemisphere,
			     int32_t *degrees_e7)
{
	if (text == NULL || *text == '\0' || degrees_e7 == NULL ||
	    (hemisphere != 'N' && hemisphere != 'S' &&
	     hemisphere != 'E' && hemisphere != 'W')) {
		return false;
	}
	const char *decimal = strchr(text, '.');
	size_t whole_digits = decimal == NULL ? strlen(text) :
					      (size_t)(decimal - text);
	if (whole_digits < 3U) {
		return false;
	}
	size_t degree_digits = whole_digits - 2U;
	uint32_t degrees = 0U;
	for (size_t i = 0U; i < degree_digits; ++i) {
		if (text[i] < '0' || text[i] > '9') {
			return false;
		}
		degrees = degrees * 10U + (uint32_t)(text[i] - '0');
	}
	int32_t minutes_e6 = parse_fixed(text + degree_digits, 1000000U);
	int64_t result = (int64_t)degrees * 10000000LL +
			 ((int64_t)minutes_e6 * 10LL + 30LL) / 60LL;
	if (hemisphere == 'S' || hemisphere == 'W') {
		result = -result;
	}
	*degrees_e7 = (int32_t)result;
	return true;
}

static uint32_t parse_utc_ms(const char *text)
{
	if (text == NULL || strlen(text) < 6U) {
		return 0U;
	}
	for (uint8_t i = 0U; i < 6U; ++i) {
		if (text[i] < '0' || text[i] > '9') {
			return 0U;
		}
	}
	uint32_t hours = (uint32_t)(text[0] - '0') * 10U +
			 (uint32_t)(text[1] - '0');
	uint32_t minutes = (uint32_t)(text[2] - '0') * 10U +
			   (uint32_t)(text[3] - '0');
	uint32_t seconds = (uint32_t)(text[4] - '0') * 10U +
			   (uint32_t)(text[5] - '0');
	uint32_t milliseconds = 0U;
	if (text[6] == '.') {
		uint32_t place = 100U;
		for (uint8_t i = 7U; text[i] >= '0' && text[i] <= '9' && place > 0U;
		     ++i) {
			milliseconds += (uint32_t)(text[i] - '0') * place;
			place /= 10U;
		}
	}
	if (hours > 23U || minutes > 59U || seconds > 60U) {
		return 0U;
	}
	return ((hours * 60U + minutes) * 60U + seconds) * 1000U +
	       milliseconds;
}
