/**
 * @brief Driver for the Bosch BMX160 9-axis IMU (BMI160 accel/gyro + BMM150 magnetometer)
 *
 * Port of flight/hal/drivers/bmx160.py. The accel/gyro register map is the BMI160's. The
 * BMM150 magnetometer sits behind the BMX160's secondary interface: the driver talks to it
 * through the MAG_IF registers during init, then the BMX160 polls it on its own and
 * mirrors X, Y, Z and RHALL into DATA_MAG (0x04).
 *
 * Python names the MAG_IF registers one lower than the datasheet (its _MAG_IF_0 is 0x4C,
 * which the datasheet calls MAG_IF[1]). This file uses the datasheet names.
 */

#define DT_DRV_COMPAT bosch_bmx160

#include "argus/drivers/bmx160.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bmx160, CONFIG_SENSOR_LOG_LEVEL);

/* BMX160 registers */
#define BMX160_REG_CHIP_ID   0x00
#define BMX160_REG_ERR       0x02
#define BMX160_REG_DATA_MAG  0x04 /* X, Y, Z, RHALL, then gyro at 0x0C, accel at 0x12 */
#define BMX160_REG_DATA_GYR  0x0C
#define BMX160_REG_DATA_ACC  0x12
#define BMX160_REG_STATUS    0x1B
#define BMX160_REG_TEMP      0x20
#define BMX160_REG_ACC_CONF  0x40
#define BMX160_REG_ACC_RANGE 0x41
#define BMX160_REG_GYR_CONF  0x42
#define BMX160_REG_GYR_RANGE 0x43
#define BMX160_REG_MAG_CONF  0x44
#define BMX160_REG_MAG_IF1   0x4C /* manual mode enable, read burst length */
#define BMX160_REG_MAG_IF2   0x4D /* BMM150 read address; writing it starts the read */
#define BMX160_REG_MAG_IF3   0x4E /* BMM150 write address; writing it starts the write */
#define BMX160_REG_MAG_IF4   0x4F /* BMM150 write data */
#define BMX160_REG_CMD       0x7E

#define BMX160_CHIP_ID           0xD8
#define BMX160_STATUS_MAG_MAN_OP BIT(2)
#define BMX160_MAG_IF1_MANUAL    BIT(7)

/* CMD register values */
#define BMX160_CMD_SOFT_RESET    0xB6
#define BMX160_CMD_ACC_NORMAL    0x11
#define BMX160_CMD_GYR_NORMAL    0x15
#define BMX160_CMD_MAG_NORMAL    0x19
#define BMX160_CMD_MAG_LOW_POWER 0x1A

/*
 * What Python actually writes to the CONF registers. Its ODR setters write the bare ODR
 * code, which also clears the bandwidth bits (normal filter -> OSR4):
 *   accel_odr = 25 -> ACC_CONF 0x06: 25 Hz
 *   gyro_odr = 25  -> GYR_CONF 0x07: 50 Hz (25 Hz isn't valid for the gyro, rounds to 50)
 */
#define BMX160_ACC_CONF_VAL 0x06
#define BMX160_GYR_CONF_VAL 0x07
#define BMX160_MAG_CONF_VAL 0x05 /* 12.5 Hz */

/* BMM150 registers, reached through MAG_IF */
#define BMM150_REG_DATA_X         0x42
#define BMM150_REG_POWER          0x4B
#define BMM150_REG_OPMODE         0x4C
#define BMM150_REG_REP_XY         0x51
#define BMM150_REG_REP_Z          0x52
#define BMM150_POWER_SLEEP        0x01
#define BMM150_OPMODE_FORCED      0x02
#define BMM150_REP_XY_HIGH_ACC    0x17 /* 47 repetitions, the high-accuracy preset */
#define BMM150_REP_Z_HIGH_ACC     0x52 /* 83 repetitions */
#define BMM150_OVERFLOW_XY        (-4096)
#define BMM150_OVERFLOW_Z         (-16384)

/*
 * Timing. Python's waits, except the gyro: Python sleeps 8.1 ms after the normal-mode
 * command, but the gyro takes up to 80 ms to start (BMI160_GYRO_DELAY_MS in Bosch's API).
 */
#define BMX160_RESET_DELAY_MS      3
#define BMX160_MAG_NORMAL_DELAY_US 650
#define BMX160_MAG_WARMUP_MS       100
#define BMX160_ACC_NORMAL_DELAY_MS 5
#define BMX160_GYR_NORMAL_DELAY_MS 80
/* The chip needs 450 us between writes while any sensor is suspended, as during init */
#define BMX160_WRITE_GAP_US        450
/* Python gives a BMM150 transfer 1 ms. Poll for up to 2 ms. */
#define BMX160_MAN_OP_POLLS        20
#define BMX160_MAN_OP_POLL_US      100

#define BMX160_PI          3.14159265358979f
#define BMX160_STANDARD_G  9.80665f
#define BMX160_UT_PER_GAUSS 100.0f

static const uint16_t gyro_range_dps[] = {2000, 1000, 500, 250, 125};

/* BMM150 factory trim, read once at init. Python: _read_trim(). */
struct bmx160_trim {
	int8_t x1, y1, x2, y2, xy2;
	uint8_t xy1;
	int16_t z2, z3, z4;
	uint16_t z1, xyz1;
};

/* Read-only, one per device, built at compile time from devicetree */
struct bmx160_config {
	struct i2c_dt_spec i2c;
	uint8_t gyro_range;  /* GYR_RANGE code */
	uint8_t accel_range; /* ACC_RANGE code */
};

/* Mutable, one per device */
struct bmx160_data {
	int16_t gyro[3];
	int16_t accel[3];
	int16_t mag[3]; /* raw registers; the ADC value is in the upper 13 (X, Y) or 15 (Z) bits */
	uint16_t rhall; /* raw register; the ADC value is in the upper 14 bits */
	int16_t temp;
	struct bmx160_trim trim;
	uint8_t gyro_range;  /* as read back from the chip */
	uint8_t accel_g;     /* full scale in g, from the ACC_RANGE read-back */
};

/* --- Register access (Python: read_u8, read_bytes, write_u8) --- */

static int bmx160_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	const struct bmx160_config *cfg = dev->config;
	int ret = i2c_burst_read_dt(&cfg->i2c, reg, buf, len);

	if (ret < 0) {
		LOG_ERR("%s: read 0x%02x failed (%d)", dev->name, reg, ret);
	}
	return ret;
}

static int bmx160_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct bmx160_config *cfg = dev->config;
	int ret = i2c_reg_write_byte_dt(&cfg->i2c, reg, val);

	if (ret < 0) {
		LOG_ERR("%s: write 0x%02x failed (%d)", dev->name, reg, ret);
		return ret;
	}
	k_usleep(BMX160_WRITE_GAP_US);
	return 0;
}

/* Python: query_error / error_code checks after a setting change */
static int bmx160_check_err(const struct device *dev, uint8_t mask, const char *what)
{
	uint8_t err;
	int ret = bmx160_read(dev, BMX160_REG_ERR, &err, 1);

	if (ret < 0) {
		return ret;
	}
	if (err & mask) {
		LOG_ERR("%s: ERR_REG 0x%02x after %s", dev->name, err, what);
		return -EIO;
	}
	return 0;
}

/* --- BMM150 through the secondary interface (Python: _bmm150_write, _bmm150_read) --- */

static int bmx160_mag_wait_idle(const struct device *dev)
{
	uint8_t status;

	for (int i = 0; i < BMX160_MAN_OP_POLLS; i++) {
		int ret = bmx160_read(dev, BMX160_REG_STATUS, &status, 1);

		if (ret < 0) {
			return ret;
		}
		if (!(status & BMX160_STATUS_MAG_MAN_OP)) {
			return 0;
		}
		k_busy_wait(BMX160_MAN_OP_POLL_US);
	}

	LOG_ERR("%s: BMM150 transfer timed out", dev->name);
	return -ETIMEDOUT;
}

static int bmm150_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	int ret = bmx160_write(dev, BMX160_REG_MAG_IF4, val);

	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_IF3, reg);
	}
	if (ret == 0) {
		ret = bmx160_mag_wait_idle(dev);
	}
	return ret;
}

/* len must be 1, 2, 6 or 8 */
static int bmm150_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	uint8_t burst;
	int ret;

	switch (len) {
	case 1:
		burst = 0;
		break;
	case 2:
		burst = 1;
		break;
	case 6:
		burst = 2;
		break;
	case 8:
		burst = 3;
		break;
	default:
		return -EINVAL;
	}

	ret = bmx160_write(dev, BMX160_REG_MAG_IF1, BMX160_MAG_IF1_MANUAL | burst);
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_IF2, reg);
	}
	if (ret == 0) {
		ret = bmx160_mag_wait_idle(dev);
	}
	if (ret == 0) {
		ret = bmx160_read(dev, BMX160_REG_DATA_MAG, buf, len);
	}
	return ret;
}

/* Python: _read_trim(). BMM150 trim registers 0x5D-0x71. */
static int bmx160_read_trim(const struct device *dev)
{
	struct bmx160_trim *t = &((struct bmx160_data *)dev->data)->trim;
	uint8_t d[8];
	int ret;

	ret = bmm150_read(dev, 0x5D, d, 2);
	if (ret < 0) {
		return ret;
	}
	t->x1 = (int8_t)d[0];
	t->y1 = (int8_t)d[1];

	ret = bmm150_read(dev, 0x62, d, 2);
	if (ret < 0) {
		return ret;
	}
	t->z4 = (int16_t)sys_get_le16(d);

	ret = bmm150_read(dev, 0x64, d, 2);
	if (ret < 0) {
		return ret;
	}
	t->x2 = (int8_t)d[0];
	t->y2 = (int8_t)d[1];

	ret = bmm150_read(dev, 0x68, d, 8);
	if (ret < 0) {
		return ret;
	}
	t->z2 = (int16_t)sys_get_le16(&d[0]);
	t->z1 = sys_get_le16(&d[2]);
	t->xyz1 = sys_get_le16(&d[4]) & 0x7FFF;
	t->z3 = (int16_t)sys_get_le16(&d[6]);

	ret = bmm150_read(dev, 0x70, d, 2);
	if (ret < 0) {
		return ret;
	}
	t->xy2 = (int8_t)d[0];
	t->xy1 = d[1];

	return 0;
}

/* --- Init steps (Python: init_mag, init_accel, init_gyro) --- */

/* Bosch's setup sequence for the BMM150 behind the BMX160, as in Python's init_mag() */
static int bmx160_init_mag(const struct device *dev)
{
	int ret;

	ret = bmx160_write(dev, BMX160_REG_CMD, BMX160_CMD_MAG_NORMAL);
	if (ret < 0) {
		return ret;
	}
	k_usleep(BMX160_MAG_NORMAL_DELAY_US);

	ret = bmx160_write(dev, BMX160_REG_MAG_IF1, BMX160_MAG_IF1_MANUAL);
	if (ret == 0) {
		ret = bmm150_write(dev, BMM150_REG_POWER, BMM150_POWER_SLEEP);
	}
	if (ret == 0) {
		ret = bmx160_read_trim(dev);
	}
	/* _read_trim() left a longer burst selected; back to single-byte manual mode */
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_IF1, BMX160_MAG_IF1_MANUAL);
	}
	if (ret == 0) {
		ret = bmm150_write(dev, BMM150_REG_REP_XY, BMM150_REP_XY_HIGH_ACC);
	}
	if (ret == 0) {
		ret = bmm150_write(dev, BMM150_REG_REP_Z, BMM150_REP_Z_HIGH_ACC);
	}
	if (ret == 0) {
		ret = bmm150_write(dev, BMM150_REG_OPMODE, BMM150_OPMODE_FORCED);
	}
	/* Where the BMX160 reads from when it polls the BMM150 on its own */
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_IF2, BMM150_REG_DATA_X);
	}
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_CONF, BMX160_MAG_CONF_VAL);
	}
	/* Data mode, burst of 8 so RHALL comes with every sample */
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_MAG_IF1, 0x03);
	}
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_CMD, BMX160_CMD_MAG_LOW_POWER);
	}
	if (ret < 0) {
		LOG_ERR("%s: magnetometer setup failed (%d)", dev->name, ret);
		return ret;
	}

	k_msleep(BMX160_MAG_WARMUP_MS);
	return 0;
}

/* Write a range register and read it back, like the Python range setters */
static int bmx160_write_range(const struct device *dev, uint8_t reg, uint8_t code,
			      uint8_t *readback)
{
	int ret = bmx160_write(dev, reg, code);

	if (ret == 0) {
		ret = bmx160_read(dev, reg, readback, 1);
	}
	if (ret == 0 && *readback != code) {
		LOG_ERR("%s: wrote 0x%02x to 0x%02x, read back 0x%02x", dev->name, code, reg,
			*readback);
		ret = -EIO;
	}
	return ret;
}

static int accel_range_g(uint8_t code)
{
	switch (code) {
	case 0x03:
		return 2;
	case 0x05:
		return 4;
	case 0x08:
		return 8;
	case 0x0C:
		return 16;
	default:
		return -EINVAL;
	}
}

static int bmx160_init_accel(const struct device *dev)
{
	const struct bmx160_config *cfg = dev->config;
	struct bmx160_data *data = dev->data;
	uint8_t readback;
	int ret;

	ret = bmx160_write_range(dev, BMX160_REG_ACC_RANGE, cfg->accel_range, &readback);
	if (ret < 0) {
		return ret;
	}
	data->accel_g = accel_range_g(readback);

	ret = bmx160_write(dev, BMX160_REG_ACC_CONF, BMX160_ACC_CONF_VAL);
	if (ret == 0) {
		ret = bmx160_check_err(dev, BMX160_ERR_CODE, "accel ODR");
	}
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_CMD, BMX160_CMD_ACC_NORMAL);
	}
	if (ret < 0) {
		return ret;
	}
	k_msleep(BMX160_ACC_NORMAL_DELAY_MS);
	return bmx160_check_err(dev, 0xFF, "accel normal mode");
}

static int bmx160_init_gyro(const struct device *dev)
{
	const struct bmx160_config *cfg = dev->config;
	int ret;

	ret = bmx160_set_gyro_range(dev, cfg->gyro_range);
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_GYR_CONF, BMX160_GYR_CONF_VAL);
	}
	if (ret == 0) {
		ret = bmx160_check_err(dev, BMX160_ERR_CODE, "gyro ODR");
	}
	if (ret == 0) {
		ret = bmx160_write(dev, BMX160_REG_CMD, BMX160_CMD_GYR_NORMAL);
	}
	if (ret < 0) {
		return ret;
	}
	k_msleep(BMX160_GYR_NORMAL_DELAY_MS);
	return bmx160_check_err(dev, 0xFF, "gyro normal mode");
}

/* --- Public API --- */

int bmx160_set_gyro_range(const struct device *dev, enum bmx160_gyro_range range)
{
	struct bmx160_data *data = dev->data;
	uint8_t readback;
	int ret;

	if ((unsigned int)range >= ARRAY_SIZE(gyro_range_dps)) {
		return -EINVAL;
	}

	ret = bmx160_write(dev, BMX160_REG_GYR_RANGE, range);
	if (ret == 0) {
		ret = bmx160_read(dev, BMX160_REG_GYR_RANGE, &readback, 1);
	}
	if (ret < 0) {
		return ret;
	}

	/* Scale with what the chip holds, even if it isn't what was asked for */
	if (readback < ARRAY_SIZE(gyro_range_dps)) {
		data->gyro_range = readback;
	}
	if (readback != range) {
		LOG_ERR("%s: gyro range read back 0x%02x, wrote 0x%02x", dev->name, readback,
			range);
		return -EIO;
	}
	return 0;
}

enum bmx160_gyro_range bmx160_get_gyro_range(const struct device *dev)
{
	const struct bmx160_data *data = dev->data;

	return data->gyro_range;
}

int bmx160_read_errors(const struct device *dev, uint8_t *err)
{
	return bmx160_read(dev, BMX160_REG_ERR, err, 1);
}

/* --- Sensor API --- */

static void get_xyz(const uint8_t *buf, int16_t out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = (int16_t)sys_get_le16(&buf[2 * i]);
	}
}

static int bmx160_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bmx160_data *data = dev->data;
	uint8_t buf[20];
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
		/* MAG, RHALL, GYR, ACC in one burst so all three come from the same moment */
		ret = bmx160_read(dev, BMX160_REG_DATA_MAG, buf, 20);
		if (ret < 0) {
			return ret;
		}
		get_xyz(&buf[0], data->mag);
		data->rhall = sys_get_le16(&buf[6]);
		get_xyz(&buf[8], data->gyro);
		get_xyz(&buf[14], data->accel);
		ret = bmx160_read(dev, BMX160_REG_TEMP, buf, 2);
		if (ret < 0) {
			return ret;
		}
		data->temp = (int16_t)sys_get_le16(buf);
		return 0;
	case SENSOR_CHAN_GYRO_XYZ:
		ret = bmx160_read(dev, BMX160_REG_DATA_GYR, buf, 6);
		if (ret == 0) {
			get_xyz(buf, data->gyro);
		}
		return ret;
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = bmx160_read(dev, BMX160_REG_DATA_ACC, buf, 6);
		if (ret == 0) {
			get_xyz(buf, data->accel);
		}
		return ret;
	case SENSOR_CHAN_MAGN_XYZ:
		ret = bmx160_read(dev, BMX160_REG_DATA_MAG, buf, 8);
		if (ret == 0) {
			get_xyz(buf, data->mag);
			data->rhall = sys_get_le16(&buf[6]);
		}
		return ret;
	case SENSOR_CHAN_DIE_TEMP:
		ret = bmx160_read(dev, BMX160_REG_TEMP, buf, 2);
		if (ret == 0) {
			data->temp = (int16_t)sys_get_le16(buf);
		}
		return ret;
	default:
		return -ENOTSUP;
	}
}

/*
 * BMM150 floating-point compensation, from Python's _compensate_x/_y/_z (Bosch BMM150
 * datasheet). Inputs are ADC values, output is uT. Overflow or missing trim gives 0.
 */
static float bmm150_compensate_xy(const struct bmx160_trim *t, int16_t adc, uint16_t rhall,
				  int8_t dig1, int8_t dig2)
{
	if (adc == BMM150_OVERFLOW_XY || rhall == 0 || t->xyz1 == 0) {
		return 0.0f;
	}
	float p0 = (float)t->xyz1 * 16384.0f / (float)rhall - 16384.0f;
	float p1 = (float)t->xy2 * (p0 * p0 / 268435456.0f);
	float p2 = p1 + p0 * (float)t->xy1 / 16384.0f;
	float p4 = (float)adc * ((p2 + 256.0f) * ((float)dig2 + 160.0f));

	return (p4 / 8192.0f + (float)dig1 * 8.0f) / 16.0f;
}

static float bmm150_compensate_z(const struct bmx160_trim *t, int16_t adc, uint16_t rhall)
{
	if (adc == BMM150_OVERFLOW_Z || t->z2 == 0 || t->z1 == 0 || rhall == 0 ||
	    t->xyz1 == 0) {
		return 0.0f;
	}
	float p0 = (float)adc - (float)t->z4;
	float p1 = (float)rhall - (float)t->xyz1;
	float p2 = (float)t->z3 * p1;
	float p3 = (float)t->z2 + ((float)t->z1 * (float)rhall / 32768.0f);

	if (p3 == 0.0f) {
		return 0.0f;
	}
	return (p0 * 131072.0f - p2) / (p3 * 64.0f);
}

/* Python: mag(). Falls back to ADC / 16 if the trim wasn't read. */
static void bmx160_mag_ut(const struct bmx160_data *data, float out[3])
{
	const struct bmx160_trim *t = &data->trim;
	/* Arithmetic shifts, as in Python, drop the status bits under each value */
	int16_t x = data->mag[0] >> 3;
	int16_t y = data->mag[1] >> 3;
	int16_t z = data->mag[2] >> 1;
	uint16_t rhall = data->rhall >> 2;

	if (t->xyz1 == 0) {
		out[0] = x / 16.0f;
		out[1] = y / 16.0f;
		out[2] = z / 16.0f;
		return;
	}
	out[0] = bmm150_compensate_xy(t, x, rhall, t->x1, t->x2);
	out[1] = bmm150_compensate_xy(t, y, rhall, t->y1, t->y2);
	out[2] = bmm150_compensate_z(t, z, rhall);
}

/* Fill val[] for an X, Y, Z or XYZ channel from three converted values */
static int put_axes(enum sensor_channel chan, enum sensor_channel x_chan, const float v[3],
		    struct sensor_value *val)
{
	int first = chan - x_chan; /* 0, 1, 2 for X, Y, Z; 3 for XYZ */
	int count = 1;

	if (first == 3) {
		first = 0;
		count = 3;
	}
	for (int i = 0; i < count; i++) {
		int ret = sensor_value_from_float(&val[i], v[first + i]);

		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}

static int bmx160_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct bmx160_data *data = dev->data;
	float v[3];

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ: {
		/* Python: GYR_SCALAR = range / 32768 deg/s per LSB, in rad/s */
		float scale = gyro_range_dps[data->gyro_range] / 32768.0f * BMX160_PI / 180.0f;

		for (int i = 0; i < 3; i++) {
			v[i] = data->gyro[i] * scale;
		}
		return put_axes(chan, SENSOR_CHAN_GYRO_X, v, val);
	}
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ: {
		/* Python: ACC_SCALAR = range / 32768 g per LSB, in m/s^2 */
		float scale = data->accel_g / 32768.0f * BMX160_STANDARD_G;

		for (int i = 0; i < 3; i++) {
			v[i] = data->accel[i] * scale;
		}
		return put_axes(chan, SENSOR_CHAN_ACCEL_X, v, val);
	}
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		bmx160_mag_ut(data, v);
		for (int i = 0; i < 3; i++) {
			v[i] /= BMX160_UT_PER_GAUSS;
		}
		return put_axes(chan, SENSOR_CHAN_MAGN_X, v, val);
	case SENSOR_CHAN_DIE_TEMP:
		/* Python: temperature(). 0 is 23 degC, 1/512 K per LSB. */
		return sensor_value_from_float(val, data->temp / 512.0f + 23.0f);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, bmx160_api) = {
	.sample_fetch = bmx160_sample_fetch,
	.channel_get = bmx160_channel_get,
};

/* Python: __init__(). Runs once per device at boot; a non-zero return marks it not ready. */
static int bmx160_init(const struct device *dev)
{
	const struct bmx160_config *cfg = dev->config;
	struct bmx160_data *data = dev->data;
	uint8_t id;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	/* Zeroed trim makes mag readings fall back to ADC / 16 */
	*data = (struct bmx160_data){0};

	ret = bmx160_write(dev, BMX160_REG_CMD, BMX160_CMD_SOFT_RESET);
	if (ret < 0) {
		return ret;
	}
	/* Datasheet says ~1 ms; Python found 1 ms sometimes isn't enough */
	k_msleep(BMX160_RESET_DELAY_MS);

	ret = bmx160_read(dev, BMX160_REG_CHIP_ID, &id, 1);
	if (ret < 0) {
		return ret;
	}
	if (id != BMX160_CHIP_ID) {
		LOG_ERR("%s: chip id 0x%02x, expected 0x%02x", dev->name, id, BMX160_CHIP_ID);
		return -ENODEV;
	}

	ret = bmx160_init_mag(dev);
	if (ret == 0) {
		ret = bmx160_init_accel(dev);
	}
	if (ret == 0) {
		ret = bmx160_init_gyro(dev);
	}
	return ret;
}

#define BMX160_DEFINE(inst)                                                                  \
	static struct bmx160_data bmx160_data_##inst;                                        \
	static const struct bmx160_config bmx160_config_##inst = {                          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                          \
		.gyro_range = DT_INST_PROP(inst, gyro_range),                               \
		.accel_range = DT_INST_PROP(inst, accel_range),                             \
	};                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bmx160_init, NULL, &bmx160_data_##inst,           \
				     &bmx160_config_##inst, POST_KERNEL,                     \
				     CONFIG_SENSOR_INIT_PRIORITY, &bmx160_api);

DT_INST_FOREACH_STATUS_OKAY(BMX160_DEFINE)
