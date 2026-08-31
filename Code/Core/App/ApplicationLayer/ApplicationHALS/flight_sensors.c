#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "flight_sensors.h"
#include "h3lis331dl_reg.h"
#include "lsm6dsox_reg.h"

#if defined(CONFIG_ODIN_HIL_SIM)
#include "flight_state.h"
#include "sim_profile.h"
#endif

LOG_MODULE_REGISTER(odin_sensors, LOG_LEVEL_INF);

#if !defined(CONFIG_ODIN_HIL_SIM)

#define SPI_MODE_3_OP (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | \
		       SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

static const struct spi_dt_spec imu_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(odin_imu), SPI_MODE_3_OP, 0);
static const struct spi_dt_spec baro_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(odin_baro), SPI_MODE_3_OP, 0);
static const struct spi_dt_spec highg_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(odin_highg), SPI_MODE_3_OP, 0);

static stmdev_ctx_t imu_ctx;
static stmdev_ctx_t highg_ctx;
static float imu_accel_offset[3];
static float imu_gyro_offset[3];
static float highg_offset[3];
static float ground_pressure;

typedef struct {
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
} BmpCalibration;

static BmpCalibration bmp_cal;

enum {
	BMP_CHIP_ID = 0x00,
	BMP_STATUS = 0x03,
	BMP_DATA = 0x04,
	BMP_PWR_CTRL = 0x1B,
	BMP_OSR = 0x1C,
	BMP_ODR = 0x1D,
	BMP_CONFIG = 0x1F,
	BMP_CALIB_DATA = 0x31,
	BMP_CMD = 0x7E,
};

static int spi_write_register(const struct spi_dt_spec *spec, uint8_t reg,
			      const uint8_t *data, size_t length)
{
	struct spi_buf buffers[2] = {
		{.buf = &reg, .len = 1U},
		{.buf = (void *)data, .len = length},
	};
	const struct spi_buf_set tx = {.buffers = buffers, .count = 2U};

	return spi_write_dt(spec, &tx);
}

static int spi_read_register(const struct spi_dt_spec *spec, uint8_t reg,
			     uint8_t *data, size_t length, uint8_t read_mask,
			     size_t dummy_bytes)
{
	uint8_t tx_data[32] = {0};
	uint8_t rx_data[32] = {0};
	size_t total = 1U + dummy_bytes + length;

	if (total > sizeof(tx_data)) {
		return -EMSGSIZE;
	}

	tx_data[0] = reg | read_mask;
	const struct spi_buf tx_buf = {.buf = tx_data, .len = total};
	struct spi_buf rx_buf = {.buf = rx_data, .len = total};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1U};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1U};

	int ret = spi_transceive_dt(spec, &tx, &rx);
	if (ret == 0) {
		memcpy(data, &rx_data[1U + dummy_bytes], length);
	}
	return ret;
}

static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *data,
			 uint16_t length)
{
	return spi_write_register(handle, reg & 0x7FU, data, length);
}

static int32_t imu_read(void *handle, uint8_t reg, uint8_t *data,
			uint16_t length)
{
	return spi_read_register(handle, reg, data, length, 0x80U, 0U);
}

static int32_t highg_write(void *handle, uint8_t reg, const uint8_t *data,
			   uint16_t length)
{
	return spi_write_register(handle, reg & 0x3FU, data, length);
}

static int32_t highg_read(void *handle, uint8_t reg, uint8_t *data,
			  uint16_t length)
{
	return spi_read_register(handle, reg, data, length, 0xC0U, 0U);
}

static void sensor_delay(uint32_t milliseconds)
{
	k_msleep(milliseconds);
}

static int wait_for_imu_data(uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();
	lsm6dsox_status_reg_t status = {0};

	do {
		if (lsm6dsox_status_reg_get(&imu_ctx, &status) != 0) {
			return -EIO;
		}
		if (status.xlda && status.gda) {
			return 0;
		}
		k_msleep(1);
	} while ((k_uptime_get_32() - start) < timeout_ms);

	return -ETIMEDOUT;
}

static int wait_for_highg_data(uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();
	uint8_t status = 0U;

	do {
		if (h3lis331dl_read_reg(&highg_ctx, H3LIS331DL_STATUS_REG,
					  &status, 1U) != 0) {
			return -EIO;
		}
		if ((status & 0x08U) != 0U) {
			return 0;
		}
		k_msleep(1);
	} while ((k_uptime_get_32() - start) < timeout_ms);

	return -ETIMEDOUT;
}

static int imu_init(void)
{
	uint8_t id = 0U;
	uint8_t reset = 0U;
	uint32_t start;

	imu_ctx.write_reg = imu_write;
	imu_ctx.read_reg = imu_read;
	imu_ctx.mdelay = sensor_delay;
	imu_ctx.handle = (void *)&imu_spi;
	k_msleep(50);

	if (lsm6dsox_device_id_get(&imu_ctx, &id) != 0 || id != LSM6DSOX_ID) {
		return -ENODEV;
	}
	if (lsm6dsox_reset_set(&imu_ctx, PROPERTY_ENABLE) != 0) {
		return -EIO;
	}

	start = k_uptime_get_32();
	do {
		if (lsm6dsox_reset_get(&imu_ctx, &reset) != 0) {
			return -EIO;
		}
	} while (reset != 0U && (k_uptime_get_32() - start) < 100U);
	if (reset != 0U) {
		return -ETIMEDOUT;
	}

	if (lsm6dsox_i3c_disable_set(&imu_ctx, LSM6DSOX_I3C_DISABLE) != 0 ||
	    lsm6dsox_xl_data_rate_set(&imu_ctx, LSM6DSOX_XL_ODR_104Hz) != 0 ||
	    lsm6dsox_xl_full_scale_set(&imu_ctx, LSM6DSOX_16g) != 0 ||
	    lsm6dsox_gy_full_scale_set(&imu_ctx, LSM6DSOX_2000dps) != 0 ||
	    lsm6dsox_gy_data_rate_set(&imu_ctx, LSM6DSOX_GY_ODR_104Hz) != 0 ||
	    lsm6dsox_gy_lp1_bandwidth_set(&imu_ctx, LSM6DSOX_MEDIUM) != 0) {
		return -EIO;
	}
	return 0;
}

static int imu_calibrate(void)
{
	int32_t accel_sum[3] = {0};
	int32_t gyro_sum[3] = {0};

	for (unsigned int i = 0U; i < 100U; ++i) {
		int16_t accel[3];
		int16_t gyro[3];

		int ret = wait_for_imu_data(100U);
		if (ret != 0) {
			return ret;
		}
		if (lsm6dsox_acceleration_raw_get(&imu_ctx, accel) != 0 ||
		    lsm6dsox_angular_rate_raw_get(&imu_ctx, gyro) != 0) {
			return -EIO;
		}
		for (unsigned int axis = 0U; axis < 3U; ++axis) {
			accel_sum[axis] += accel[axis];
			gyro_sum[axis] += gyro[axis];
		}
	}

	for (unsigned int axis = 0U; axis < 3U; ++axis) {
		imu_accel_offset[axis] =
			lsm6dsox_from_fs16_to_mg((int16_t)(accel_sum[axis] / 100));
		imu_gyro_offset[axis] =
			lsm6dsox_from_fs2000_to_mdps((int16_t)(gyro_sum[axis] / 100));
	}
	imu_accel_offset[2] -= 1000.0f;
	return 0;
}

static int highg_init(void)
{
	uint8_t id = 0U;
	uint8_t boot = 0x80U;

	highg_ctx.write_reg = highg_write;
	highg_ctx.read_reg = highg_read;
	highg_ctx.mdelay = sensor_delay;
	highg_ctx.handle = (void *)&highg_spi;
	k_msleep(50);

	if (h3lis331dl_device_id_get(&highg_ctx, &id) != 0 ||
	    id != H3LIS331DL_ID) {
		return -ENODEV;
	}
	if (h3lis331dl_write_reg(&highg_ctx, H3LIS331DL_CTRL_REG2, &boot, 1U) != 0) {
		return -EIO;
	}
	k_msleep(10);
	if (h3lis331dl_full_scale_set(&highg_ctx, H3LIS331DL_200g) != 0 ||
	    h3lis331dl_hp_path_set(&highg_ctx, H3LIS331DL_HP_DISABLE) != 0 ||
	    h3lis331dl_data_rate_set(&highg_ctx, H3LIS331DL_ODR_100Hz) != 0) {
		return -EIO;
	}
	k_msleep(100);
	return 0;
}

static int highg_calibrate(void)
{
	int32_t sum[3] = {0};

	for (unsigned int i = 0U; i < 100U; ++i) {
		int16_t raw[3];
		int ret = wait_for_highg_data(100U);

		if (ret != 0) {
			return ret;
		}
		if (h3lis331dl_acceleration_raw_get(&highg_ctx, raw) != 0) {
			return -EIO;
		}
		for (unsigned int axis = 0U; axis < 3U; ++axis) {
			sum[axis] += raw[axis];
		}
	}

	for (unsigned int axis = 0U; axis < 3U; ++axis) {
		highg_offset[axis] =
			h3lis331dl_from_fs200_to_mg((int16_t)(sum[axis] / 100));
	}
	highg_offset[2] -= 1000.0f;
	return 0;
}

static int bmp_read(uint8_t reg, uint8_t *data, size_t length)
{
	return spi_read_register(&baro_spi, reg, data, length, 0x80U, 1U);
}

static int bmp_write(uint8_t reg, uint8_t value)
{
	reg &= 0x7FU;
	return spi_write_register(&baro_spi, reg, &value, 1U);
}

static int bmp_read_calibration(void)
{
	uint8_t b[21] = {0};

	if (bmp_read(BMP_CALIB_DATA, b, sizeof(b)) != 0) {
		return -EIO;
	}

	uint16_t t1 = ((uint16_t)b[1] << 8) | b[0];
	uint16_t t2 = ((uint16_t)b[3] << 8) | b[2];
	int8_t t3 = (int8_t)b[4];
	int16_t p1 = (int16_t)(((uint16_t)b[6] << 8) | b[5]);
	int16_t p2 = (int16_t)(((uint16_t)b[8] << 8) | b[7]);
	int8_t p3 = (int8_t)b[9];
	int8_t p4 = (int8_t)b[10];
	uint16_t p5 = ((uint16_t)b[12] << 8) | b[11];
	uint16_t p6 = ((uint16_t)b[14] << 8) | b[13];
	int8_t p7 = (int8_t)b[15];
	int8_t p8 = (int8_t)b[16];
	int16_t p9 = (int16_t)(((uint16_t)b[18] << 8) | b[17]);
	int8_t p10 = (int8_t)b[19];
	int8_t p11 = (int8_t)b[20];

	bmp_cal.par_t1 = (float)t1 / 0.00390625f;
	bmp_cal.par_t2 = (float)t2 / 1073741824.0f;
	bmp_cal.par_t3 = (float)t3 / 281474976710656.0f;
	bmp_cal.par_p1 = ((float)p1 - 16384.0f) / 1048576.0f;
	bmp_cal.par_p2 = ((float)p2 - 16384.0f) / 536870912.0f;
	bmp_cal.par_p3 = (float)p3 / 4294967296.0f;
	bmp_cal.par_p4 = (float)p4 / 137438953472.0f;
	bmp_cal.par_p5 = (float)p5 / 0.125f;
	bmp_cal.par_p6 = (float)p6 / 64.0f;
	bmp_cal.par_p7 = (float)p7 / 256.0f;
	bmp_cal.par_p8 = (float)p8 / 32768.0f;
	bmp_cal.par_p9 = (float)p9 / 281474976710656.0f;
	bmp_cal.par_p10 = (float)p10 / 281474976710656.0f;
	bmp_cal.par_p11 = (float)p11 / 36893488147419103232.0f;
	return 0;
}

static void bmp_compensate(uint32_t raw_pressure, uint32_t raw_temperature,
			   float *pressure, float *temperature)
{
	float dtemp = (float)raw_temperature - bmp_cal.par_t1;
	float temp = dtemp * bmp_cal.par_t2 +
		     dtemp * dtemp * bmp_cal.par_t3;
	float temp2 = temp * temp;
	float temp3 = temp2 * temp;
	float raw = (float)raw_pressure;
	float out1 = bmp_cal.par_p5 + bmp_cal.par_p6 * temp +
		     bmp_cal.par_p7 * temp2 + bmp_cal.par_p8 * temp3;
	float out2 = raw * (bmp_cal.par_p1 + bmp_cal.par_p2 * temp +
		     bmp_cal.par_p3 * temp2 + bmp_cal.par_p4 * temp3);
	float raw2 = raw * raw;
	float out3 = raw2 * (bmp_cal.par_p9 + bmp_cal.par_p10 * temp) +
		     raw2 * raw * bmp_cal.par_p11;

	*temperature = temp;
	*pressure = out1 + out2 + out3;
}

static int bmp_read_sample(float *pressure, float *temperature)
{
	uint8_t status = 0U;
	uint8_t raw[6];
	uint32_t start = k_uptime_get_32();

	do {
		if (bmp_read(BMP_STATUS, &status, 1U) != 0) {
			return -EIO;
		}
		if ((status & (BIT(5) | BIT(6))) == (BIT(5) | BIT(6))) {
			break;
		}
		k_msleep(1);
	} while ((k_uptime_get_32() - start) < 250U);

	if ((status & (BIT(5) | BIT(6))) != (BIT(5) | BIT(6))) {
		return -ETIMEDOUT;
	}
	if (bmp_read(BMP_DATA, raw, sizeof(raw)) != 0) {
		return -EIO;
	}

	uint32_t raw_pressure = ((uint32_t)raw[2] << 16) |
				((uint32_t)raw[1] << 8) | raw[0];
	uint32_t raw_temperature = ((uint32_t)raw[5] << 16) |
				   ((uint32_t)raw[4] << 8) | raw[3];
	bmp_compensate(raw_pressure, raw_temperature, pressure, temperature);
	return 0;
}

static int bmp_init(void)
{
	uint8_t id = 0U;

	if (bmp_read(BMP_CHIP_ID, &id, 1U) != 0 || id != 0x50U) {
		return -ENODEV;
	}
	if (bmp_write(BMP_CMD, 0xB6U) != 0) {
		return -EIO;
	}
	k_msleep(10);
	if (bmp_read_calibration() != 0 ||
	    bmp_write(BMP_OSR, 0x0BU) != 0 ||
	    bmp_write(BMP_CONFIG, 0x04U) != 0 ||
	    bmp_write(BMP_ODR, 0x03U) != 0 ||
	    bmp_write(BMP_PWR_CTRL, 0x33U) != 0) {
		return -EIO;
	}

	float pressure_sum = 0.0f;
	for (unsigned int i = 0U; i < 100U; ++i) {
		float pressure;
		float temperature;
		int ret = bmp_read_sample(&pressure, &temperature);

		if (ret != 0) {
			return ret;
		}
		pressure_sum += pressure;
	}
	ground_pressure = pressure_sum / 100.0f;
	return (isfinite(ground_pressure) && ground_pressure > 1000.0f) ? 0 : -ERANGE;
}

#endif /* !CONFIG_ODIN_HIL_SIM */

#if defined(CONFIG_ODIN_HIL_SIM)

static uint32_t sim_index;
static bool sim_complete_logged;

int odin_sensors_init(void)
{
	sim_index = 0U;
	sim_complete_logged = false;
	LOG_WRN("HIL simulation profile %s: %u samples, %u ms period",
		ODIN_SIM_PROFILE_NAME, (unsigned int)SIM_LEN,
		(unsigned int)SIM_DT_MS);
	return 0;
}

int odin_sensors_read_imu(FlightSensorData *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	/* Match the old simulator: start advancing only after an ARM command. */
	if (FSM_get_state() >= STATE_PAD && sim_index < (SIM_LEN - 1U)) {
		sim_index++;
	}

	data->x_mg = 0.0f;
	data->y_mg = 0.0f;
	data->z_mg = 0.0f;
	data->x_mg_IMU = 0.0f;
	data->y_mg_IMU = 0.0f;
	data->z_mg_IMU = sim_accel_mg[sim_index];
	data->x_gy = 0.0f;
	data->y_gy = 0.0f;
	data->z_gy = 0.0f;

	if (sim_index == (SIM_LEN - 1U) && !sim_complete_logged) {
		LOG_INF("HIL simulation profile complete; holding final sample");
		sim_complete_logged = true;
	}
	return 0;
}

int odin_sensors_read_baro(FlightSensorData *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	data->altitude = sim_alt[sim_index];
	data->pressure = 0.0f;
	data->temperature = 0.0f;
	return 0;
}

#else

int odin_sensors_init(void)
{
	if (!spi_is_ready_dt(&imu_spi) || !spi_is_ready_dt(&baro_spi) ||
	    !spi_is_ready_dt(&highg_spi)) {
		return -ENODEV;
	}

	int ret = bmp_init();
	if (ret != 0) {
		LOG_ERR("BMP388 init/calibration failed: %d", ret);
		return ret;
	}
	k_msleep(50);

	ret = highg_init();
	if (ret == 0) {
		ret = highg_calibrate();
	}
	if (ret != 0) {
		LOG_ERR("H3LIS331DL init/calibration failed: %d", ret);
		return ret;
	}
	k_msleep(50);

	ret = imu_init();
	if (ret == 0) {
		ret = imu_calibrate();
	}
	if (ret != 0) {
		LOG_ERR("LSM6DSOX init/calibration failed: %d", ret);
		return ret;
	}

	LOG_INF("sensors ready; ground pressure calibrated");
	return 0;
}

int odin_sensors_read_imu(FlightSensorData *data)
{
	int16_t highg[3];
	int16_t accel[3];
	int16_t gyro[3];

	if (data == NULL) {
		return -EINVAL;
	}
	int ret = wait_for_highg_data(50U);
	if (ret != 0) {
		return ret;
	}
	if (h3lis331dl_acceleration_raw_get(&highg_ctx, highg) != 0) {
		return -EIO;
	}
	ret = wait_for_imu_data(50U);
	if (ret != 0) {
		return ret;
	}
	if (lsm6dsox_acceleration_raw_get(&imu_ctx, accel) != 0 ||
	    lsm6dsox_angular_rate_raw_get(&imu_ctx, gyro) != 0) {
		return -EIO;
	}

	data->x_mg = h3lis331dl_from_fs200_to_mg(highg[0]) - highg_offset[0];
	data->y_mg = h3lis331dl_from_fs200_to_mg(highg[1]) - highg_offset[1];
	data->z_mg = h3lis331dl_from_fs200_to_mg(highg[2]) - highg_offset[2];
	data->x_mg_IMU = lsm6dsox_from_fs16_to_mg(accel[0]) - imu_accel_offset[0];
	data->y_mg_IMU = lsm6dsox_from_fs16_to_mg(accel[1]) - imu_accel_offset[1];
	data->z_mg_IMU = lsm6dsox_from_fs16_to_mg(accel[2]) - imu_accel_offset[2];
	data->x_gy = lsm6dsox_from_fs2000_to_mdps(gyro[0]) - imu_gyro_offset[0];
	data->y_gy = lsm6dsox_from_fs2000_to_mdps(gyro[1]) - imu_gyro_offset[1];
	data->z_gy = lsm6dsox_from_fs2000_to_mdps(gyro[2]) - imu_gyro_offset[2];
	return 0;
}

int odin_sensors_read_baro(FlightSensorData *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	int ret = bmp_read_sample(&data->pressure, &data->temperature);
	if (ret == 0) {
		data->altitude = 44330.0f *
			(1.0f - powf(data->pressure / ground_pressure, 0.1903f));
	}
	return ret;
}

#endif /* CONFIG_ODIN_HIL_SIM */
