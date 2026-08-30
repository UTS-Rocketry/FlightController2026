#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "W25Q128_HAL.h"
#include "odin_app.h"
#include "packets.h"

LOG_MODULE_REGISTER(odin_storage, LOG_LEVEL_INF);

#define FLASH_NODE DT_NODELABEL(odin_flash)
#define FLASH_LOG_START 0U
#define FLASH_RECORD_SIZE 64U
#define FLASH_SECTOR_SIZE 4096U
#define FLASH_TOTAL_SIZE (16U * 1024U * 1024U)
#define FLASH_MAX_RECORDS (FLASH_TOTAL_SIZE / FLASH_RECORD_SIZE)
#define FLASH_PREERASE_SECTORS 150U
#define FLASH_SYNC_WORD 0xAAU

static const struct device *const flash_device = DEVICE_DT_GET(FLASH_NODE);
static bool storage_enabled;
static uint32_t write_address;
static uint32_t record_count;
static uint32_t prepared_start;
static uint32_t prepared_end;

static bool record_is_written(uint32_t index)
{
	uint8_t sync = 0xFFU;
	off_t address = (off_t)(FLASH_LOG_START + index * FLASH_RECORD_SIZE);

	return flash_read(flash_device, address, &sync, sizeof(sync)) == 0 &&
	       sync == FLASH_SYNC_WORD;
}

static int recover_write_pointer(void)
{
	uint32_t low = 0U;
	uint32_t high = FLASH_MAX_RECORDS;

	while (low < high) {
		uint32_t middle = low + (high - low) / 2U;
		if (record_is_written(middle)) {
			low = middle + 1U;
		} else {
			high = middle;
		}
	}
	record_count = low;
	write_address = FLASH_LOG_START + low * FLASH_RECORD_SIZE;
	return 0;
}

static int prepare_log_region(uint32_t sectors)
{
	uint32_t address;

	if ((write_address % FLASH_SECTOR_SIZE) == 0U) {
		address = write_address;
	} else {
		address = ROUND_UP(write_address, FLASH_SECTOR_SIZE);
	}
	prepared_start = address;
	prepared_end = address;

	for (uint32_t i = 0U; i < sectors && address < FLASH_TOTAL_SIZE; ++i) {
		int ret = flash_erase(flash_device, (off_t)address, FLASH_SECTOR_SIZE);
		if (ret != 0) {
			return ret;
		}
		address += FLASH_SECTOR_SIZE;
		prepared_end = address;
	}
	return 0;
}

int odin_storage_init(void)
{
	if (!device_is_ready(flash_device)) {
		return -ENODEV;
	}

	int ret = recover_write_pointer();
	if (ret == 0) {
		ret = prepare_log_region(FLASH_PREERASE_SECTORS);
	}
	if (ret != 0) {
		return ret;
	}

	storage_enabled = true;
	LOG_INF("flash ready at record %u; %u sectors prepared",
		record_count, FLASH_PREERASE_SECTORS);
	return 0;
}

static int write_record(const uint8_t record[FLASH_RECORD_SIZE])
{
	if (record_count >= FLASH_MAX_RECORDS) {
		return -ENOSPC;
	}

	if ((write_address % FLASH_SECTOR_SIZE) == 0U &&
	    !(write_address >= prepared_start && write_address < prepared_end)) {
		int ret = flash_erase(flash_device, (off_t)write_address,
				      FLASH_SECTOR_SIZE);
		if (ret != 0) {
			return ret;
		}
	}

	int ret = flash_write(flash_device, (off_t)write_address, record,
			      FLASH_RECORD_SIZE);
	if (ret == 0) {
		write_address += FLASH_RECORD_SIZE;
		record_count++;
	}
	return ret;
}

void odin_storage_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	odin_wait_for_start();
	uint8_t sequence = 0U;

	for (;;) {
		FlightSensorData sample;
		(void)k_msgq_get(&odin_log_queue, &sample, K_FOREVER);
		if (!storage_enabled) {
			continue;
		}

		uint8_t record[FLASH_RECORD_SIZE] = {0};
		TelemetryPacket packet = {0};
		packet.header.sync_word = FLASH_SYNC_WORD;
		packet.header.packet_type = telemetry_packet;
		packet.header.sequence_number = sequence++;
		packet.sensordata = sample;
		packet.flight_State = sample.flight_state;
		telemetry_serializer(&packet, record);

		int ret = write_record(record);
		if (ret != 0) {
			LOG_ERR("flash log failed at 0x%06x: %d", write_address, ret);
			storage_enabled = false;
		}
	}
}
