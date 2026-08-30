#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "Lora_App.h"
#include "flight_state.h"
#include "odin_app.h"
#include "packets.h"
#include "pyro.h"

LOG_MODULE_REGISTER(odin_radio, LOG_LEVEL_INF);

#define ODIN_IO_NODE DT_NODELABEL(odin_io)
#define RADIO_OP (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static const struct spi_dt_spec radio_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(odin_lora), RADIO_OP, 0);
static const struct gpio_dt_spec radio_reset =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, lora_reset_gpios);
static const struct gpio_dt_spec radio_dio0 =
	GPIO_DT_SPEC_GET(ODIN_IO_NODE, lora_dio0_gpios);

K_SEM_DEFINE(radio_irq_sem, 0, 1);
static struct gpio_callback dio0_callback;
static bool radio_enabled;

enum {
	REG_FIFO = 0x00,
	REG_OPMODE = 0x01,
	REG_FR_MSB = 0x06,
	REG_FR_MID = 0x07,
	REG_FR_LSB = 0x08,
	REG_PA_CONFIG = 0x09,
	REG_OCP = 0x0B,
	REG_LNA = 0x0C,
	REG_FIFO_ADDR_PTR = 0x0D,
	REG_FIFO_TX_BASE_ADDR = 0x0E,
	REG_FIFO_RX_BASE_ADDR = 0x0F,
	REG_FIFO_RX_CURRENT_ADDR = 0x10,
	REG_IRQ_FLAGS = 0x12,
	REG_RX_NB_BYTES = 0x13,
	REG_MODEM_CONFIG_1 = 0x1D,
	REG_MODEM_CONFIG_2 = 0x1E,
	REG_PREAMBLE_MSB = 0x20,
	REG_PREAMBLE_LSB = 0x21,
	REG_PAYLOAD_LENGTH = 0x22,
	REG_MODEM_CONFIG_3 = 0x26,
	REG_SYNC_WORD = 0x39,
	REG_DIO_MAPPING_1 = 0x40,
	REG_VERSION = 0x42,
	REG_PA_DAC = 0x4D,
};

enum {
	MODE_LONG_RANGE = 0x80,
	MODE_SLEEP = 0x00,
	MODE_STANDBY = 0x01,
	MODE_TRANSMIT = 0x03,
	MODE_RX_SINGLE = 0x06,
	IRQ_TX_DONE = 0x08,
	IRQ_PAYLOAD_CRC_ERROR = 0x20,
	IRQ_RX_DONE = 0x40,
};

static int radio_write(uint8_t reg, const uint8_t *data, size_t length)
{
	uint8_t address = reg | 0x80U;
	struct spi_buf buffers[2] = {
		{.buf = &address, .len = 1U},
		{.buf = (void *)data, .len = length},
	};
	const struct spi_buf_set tx = {.buffers = buffers, .count = 2U};

	return spi_write_dt(&radio_spi, &tx);
}

static int radio_write_byte(uint8_t reg, uint8_t value)
{
	return radio_write(reg, &value, 1U);
}

static int radio_read(uint8_t reg, uint8_t *data, size_t length)
{
	uint8_t address = reg & 0x7FU;
	struct spi_buf tx_buffers[2] = {
		{.buf = &address, .len = 1U},
		{.buf = NULL, .len = length},
	};
	struct spi_buf rx_buffers[2] = {
		{.buf = NULL, .len = 1U},
		{.buf = data, .len = length},
	};
	const struct spi_buf_set tx = {.buffers = tx_buffers, .count = 2U};
	const struct spi_buf_set rx = {.buffers = rx_buffers, .count = 2U};

	return spi_transceive_dt(&radio_spi, &tx, &rx);
}

static void radio_dio0_callback(const struct device *port,
				struct gpio_callback *callback,
				gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);
	k_sem_give(&radio_irq_sem);
}

static int radio_standby(void)
{
	return radio_write_byte(REG_OPMODE, MODE_LONG_RANGE | MODE_STANDBY);
}

static int radio_set_frequency(uint32_t frequency)
{
	uint32_t frf = (uint32_t)(((uint64_t)frequency << 19) / 32000000ULL);
	uint8_t bytes[3] = {
		(uint8_t)(frf >> 16),
		(uint8_t)(frf >> 8),
		(uint8_t)frf,
	};

	int ret = radio_write_byte(REG_FR_MSB, bytes[0]);
	if (ret == 0) {
		ret = radio_write_byte(REG_FR_MID, bytes[1]);
	}
	if (ret == 0) {
		ret = radio_write_byte(REG_FR_LSB, bytes[2]);
	}
	return ret;
}

int odin_radio_init(void)
{
	if (!spi_is_ready_dt(&radio_spi) || !gpio_is_ready_dt(&radio_reset) ||
	    !gpio_is_ready_dt(&radio_dio0)) {
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&radio_reset, GPIO_OUTPUT_INACTIVE);
	if (ret == 0) {
		ret = gpio_pin_configure_dt(&radio_dio0, GPIO_INPUT);
	}
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(&dio0_callback, radio_dio0_callback, BIT(radio_dio0.pin));
	ret = gpio_add_callback(radio_dio0.port, &dio0_callback);
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&radio_dio0,
						      GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (ret != 0) {
		return ret;
	}

	(void)gpio_pin_set_dt(&radio_reset, 1);
	k_msleep(1);
	(void)gpio_pin_set_dt(&radio_reset, 0);
	k_msleep(5);

	uint8_t version = 0U;
	ret = radio_read(REG_VERSION, &version, 1U);
	if (ret != 0 || version != 0x12U) {
		return -ENODEV;
	}

	ret = radio_write_byte(REG_OPMODE, MODE_LONG_RANGE | MODE_SLEEP);
	if (ret == 0) ret = radio_write_byte(REG_FIFO_RX_BASE_ADDR, 0U);
	if (ret == 0) ret = radio_write_byte(REG_FIFO_TX_BASE_ADDR, 0U);
	if (ret == 0) ret = radio_set_frequency(915000000U);
	if (ret == 0) ret = radio_write_byte(REG_LNA, 0x23U);
	/* BW=125 kHz, CR=4/5, explicit header. */
	if (ret == 0) ret = radio_write_byte(REG_MODEM_CONFIG_1, 0x72U);
	/* SF7 and payload CRC enabled. */
	if (ret == 0) ret = radio_write_byte(REG_MODEM_CONFIG_2, 0x74U);
	if (ret == 0) ret = radio_write_byte(REG_MODEM_CONFIG_3, 0x04U);
	if (ret == 0) ret = radio_write_byte(REG_SYNC_WORD, 0x12U);
	if (ret == 0) ret = radio_write_byte(REG_PREAMBLE_MSB, 0U);
	if (ret == 0) ret = radio_write_byte(REG_PREAMBLE_LSB, 8U);
	/* PA_BOOST, 17 dBm. */
	if (ret == 0) ret = radio_write_byte(REG_PA_CONFIG, 0x8FU);
	if (ret == 0) ret = radio_write_byte(REG_PA_DAC, 0x84U);
	if (ret == 0) ret = radio_standby();
	if (ret != 0) {
		return ret;
	}

	radio_enabled = true;
	LOG_INF("SX1276 ready at 915 MHz, SF7/BW125");
	return 0;
}

static int radio_transmit(const uint8_t *data, uint8_t length,
			  uint32_t timeout_ms)
{
	int ret = radio_standby();
	if (ret == 0) ret = radio_write_byte(REG_DIO_MAPPING_1, 0x40U);
	if (ret == 0) ret = radio_write_byte(REG_IRQ_FLAGS, 0xFFU);

	uint8_t base = 0U;
	if (ret == 0) ret = radio_read(REG_FIFO_TX_BASE_ADDR, &base, 1U);
	if (ret == 0) ret = radio_write_byte(REG_FIFO_ADDR_PTR, base);
	if (ret == 0) ret = radio_write(REG_FIFO, data, length);
	if (ret == 0) ret = radio_write_byte(REG_PAYLOAD_LENGTH, length);
	if (ret != 0) {
		return ret;
	}

	k_sem_reset(&radio_irq_sem);
	ret = radio_write_byte(REG_OPMODE, MODE_LONG_RANGE | MODE_TRANSMIT);
	if (ret == 0 && k_sem_take(&radio_irq_sem, K_MSEC(timeout_ms)) != 0) {
		ret = -ETIMEDOUT;
	}

	uint8_t flags = 0U;
	if (radio_read(REG_IRQ_FLAGS, &flags, 1U) != 0) {
		ret = -EIO;
	} else if (ret == 0 && (flags & IRQ_TX_DONE) == 0U) {
		ret = -EIO;
	}
	(void)radio_write_byte(REG_IRQ_FLAGS, 0xFFU);
	(void)radio_standby();
	return ret;
}

static int radio_receive(uint8_t *data, uint8_t *received_length,
			 size_t capacity, uint32_t timeout_ms)
{
	int ret = radio_standby();
	if (ret == 0) ret = radio_write_byte(REG_DIO_MAPPING_1, 0x00U);
	if (ret == 0) ret = radio_write_byte(REG_IRQ_FLAGS, 0xFFU);

	uint8_t base = 0U;
	if (ret == 0) ret = radio_read(REG_FIFO_RX_BASE_ADDR, &base, 1U);
	if (ret == 0) ret = radio_write_byte(REG_FIFO_ADDR_PTR, base);
	if (ret != 0) {
		return ret;
	}

	k_sem_reset(&radio_irq_sem);
	ret = radio_write_byte(REG_OPMODE, MODE_LONG_RANGE | MODE_RX_SINGLE);
	if (ret == 0 && k_sem_take(&radio_irq_sem, K_MSEC(timeout_ms)) != 0) {
		ret = -ETIMEDOUT;
	}

	uint8_t flags = 0U;
	if (ret == 0) {
		ret = radio_read(REG_IRQ_FLAGS, &flags, 1U);
	}
	if (ret == 0 && ((flags & IRQ_RX_DONE) == 0U ||
			 (flags & IRQ_PAYLOAD_CRC_ERROR) != 0U)) {
		ret = -EBADMSG;
	}

	uint8_t length = 0U;
	if (ret == 0) ret = radio_read(REG_RX_NB_BYTES, &length, 1U);
	if (ret == 0 && length > capacity) ret = -EMSGSIZE;
	uint8_t current = 0U;
	if (ret == 0) ret = radio_read(REG_FIFO_RX_CURRENT_ADDR, &current, 1U);
	if (ret == 0) ret = radio_write_byte(REG_FIFO_ADDR_PTR, current);
	if (ret == 0) ret = radio_read(REG_FIFO, data, length);
	if (ret == 0) *received_length = length;

	(void)radio_write_byte(REG_IRQ_FLAGS, 0xFFU);
	(void)radio_standby();
	return ret;
}

static int send_telemetry(void)
{
	static uint8_t sequence;
	FlightSensorData sample;
	if (!odin_snapshot_get(&sample)) {
		return -ENODATA;
	}

	uint8_t buffer[LORA_RECEIVER_HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE] = {0};
	TelemetryPacket packet = {0};
	packet.header.sync_word = 0xAAU;
	packet.header.packet_type = telemetry_packet;
	packet.header.sequence_number = sequence++;
	packet.sensordata = sample;
	packet.flight_State = sample.flight_state;
	telemetry_serializer(&packet, buffer + LORA_RECEIVER_HEADER_SIZE);
	return radio_transmit(buffer, sizeof(buffer), 200U);
}

static int send_gps(const GPSFix *fix, uint8_t flight_state)
{
	static uint8_t sequence;
	uint8_t buffer[LORA_RECEIVER_HEADER_SIZE + GPS_PAYLOAD_SIZE] = {0};
	GPSPacket packet = {0};
	packet.header.sync_word = 0xAAU;
	packet.header.packet_type = gps_packet;
	packet.header.sequence_number = sequence++;
	packet.fix = *fix;
	packet.flight_State = flight_state;
	uint32_t age = k_uptime_get_32() - fix->last_update_ms;
	packet.age_ms = age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
	gps_serializer(&packet, buffer + LORA_RECEIVER_HEADER_SIZE);
	return radio_transmit(buffer, sizeof(buffer), 150U);
}

static int send_continuity(void)
{
	static uint8_t sequence;
	uint8_t buffer[LORA_RECEIVER_HEADER_SIZE + CONTINUITY_PAYLOAD_SIZE] = {0};
	ContinuityPacket packet = {0};
	packet.header.sync_word = 0xAAU;
	packet.header.packet_type = continuity_packet;
	packet.header.sequence_number = sequence++;
	packet.main = pyro_check_main();
	packet.drogue = pyro_check_drogue();
	continuity_serializer(&packet, buffer + LORA_RECEIVER_HEADER_SIZE);
	return radio_transmit(buffer, sizeof(buffer), 100U);
}

static int receive_command(void)
{
	uint8_t buffer[13] = {0};
	uint8_t length = 0U;
	int ret = radio_receive(buffer, &length, sizeof(buffer), 150U);
	if (ret == -ETIMEDOUT) {
		return 0;
	}
	if (ret != 0 || length < 13U) {
		return ret;
	}

	uint8_t command_id;
	uint8_t channel;
	if (command_deserializer(buffer + LORA_RECEIVER_HEADER_SIZE,
				 &command_id, &channel) != 0U) {
		odin_command_submit(command_id, channel);
	}
	return 0;
}

static uint32_t advance_period(uint32_t deadline, uint32_t period,
			       uint32_t now)
{
	do {
		deadline += period;
	} while ((int32_t)(now - deadline) >= 0);
	return deadline;
}

void odin_radio_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	uint32_t now = k_uptime_get_32();
	uint32_t next_rx = now;
	uint32_t next_flight_tx = now;
	uint32_t next_gps = now;
	uint32_t next_continuity = now;
	uint32_t last_radio_end = now - ODIN_RADIO_GUARD_MS;

	for (;;) {
		if (!radio_enabled) {
			k_sleep(K_FOREVER);
		}

		FlightSensorData sample = {0};
		(void)odin_snapshot_get(&sample);
		now = k_uptime_get_32();
		bool dispatched = false;

		if ((now - last_radio_end) >= ODIN_RADIO_GUARD_MS) {
			if (sample.flight_state <= STATE_PAD &&
			    (int32_t)(now - next_rx) >= 0) {
				next_rx = advance_period(next_rx,
						 ODIN_COMMAND_RX_PERIOD_MS, now);
				(void)receive_command();
				dispatched = true;
			} else if (sample.flight_state >= STATE_PAD &&
				   (int32_t)(now - next_flight_tx) >= 0) {
				next_flight_tx = advance_period(next_flight_tx,
							ODIN_FLIGHT_TX_PERIOD_MS, now);
				(void)send_telemetry();
				dispatched = true;
			} else if ((int32_t)(now - next_gps) >= 0) {
				next_gps = advance_period(next_gps,
						  ODIN_GPS_TX_PERIOD_MS, now);
				GPSFix fix;
				if (odin_gps_get_fix(&fix)) {
					(void)send_gps(&fix, sample.flight_state);
					dispatched = true;
				}
			} else if (sample.flight_state <= STATE_PAD &&
				   (int32_t)(now - next_continuity) >= 0) {
				next_continuity = advance_period(next_continuity,
							 ODIN_CONTINUITY_PERIOD_MS, now);
				(void)send_continuity();
				dispatched = true;
			}
		}

		if (dispatched) {
			last_radio_end = k_uptime_get_32();
		}
		k_msleep(5);
	}
}
