/**
 * @brief BMX160 driver tests.
 *
 * The real driver runs unchanged against a fake BMX160 (bmx160_emul.c) on an emulated I2C
 * bus. Expected magnetometer values come from the compensation code in
 * flight/hal/drivers/bmx160.py, run on the same raw registers and trim.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include <argus/drivers/bmx160.h>

#include "bmx160_emul.h"

#define IMU_NODE DT_NODELABEL(imu)

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct emul *const imu_emul = EMUL_DT_GET(IMU_NODE);

#define PI_F 3.14159265f
#define DEG_TO_RAD(dps) ((dps) * PI_F / 180.0f)

static float get1(const struct device *dev, enum sensor_channel chan)
{
	struct sensor_value val;

	zassert_ok(sensor_channel_get(dev, chan, &val));
	return sensor_value_to_float(&val);
}

static void get3(const struct device *dev, enum sensor_channel chan, float out[3])
{
	struct sensor_value val[3];

	zassert_ok(sensor_channel_get(dev, chan, val));
	for (int i = 0; i < 3; i++) {
		out[i] = sensor_value_to_float(&val[i]);
	}
}

static void set_xyz(uint8_t reg, int16_t x, int16_t y, int16_t z)
{
	bmx160_emul_set_reg16(imu_emul, reg, x);
	bmx160_emul_set_reg16(imu_emul, reg + 2, y);
	bmx160_emul_set_reg16(imu_emul, reg + 4, z);
}

/* Undo what tests change: faults, and the gyro range (back to the devicetree default) */
static void bmx160_before(void *fixture)
{
	ARG_UNUSED(fixture);

	bmx160_emul_set_fail(imu_emul, false);
	bmx160_emul_force_err(imu_emul, 0);
	bmx160_emul_ignore_writes(imu_emul, BMX160_EMUL_NO_REG);
	zassert_ok(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_500DPS));
}

ZTEST_SUITE(bmx160, NULL, NULL, bmx160_before, NULL, NULL);

/* Boot ran bmx160_init() with the register values Python writes */
ZTEST(bmx160, test_init_registers)
{
	static const uint8_t expected_cmds[] = {0xB6, 0x19, 0x1A, 0x11, 0x15};
	const uint8_t *cmds;
	size_t num_cmds = bmx160_emul_get_cmds(imu_emul, &cmds);

	zassert_true(device_is_ready(imu));

	/* Reset, mag normal, mag low power, accel normal, gyro normal, in Python's order */
	zassert_equal(num_cmds, ARRAY_SIZE(expected_cmds), "%zu commands", num_cmds);
	zassert_mem_equal(cmds, expected_cmds, sizeof(expected_cmds));

	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_ACC_RANGE), 0x08); /* 8 g */
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_ACC_CONF), 0x06);
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x02); /* 500 */
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_CONF), 0x07);
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_MAG_CONF), 0x05);
	/* Accel normal, gyro normal, mag interface low power */
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_PMU_STATUS), 0x16);

	/* Magnetometer left in data mode, burst 8, polling BMM150 DATA_X */
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_MAG_IF1), 0x03);
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_MAG_IF2), 0x42);

	/* BMM150: sleep mode, high-accuracy repetitions, forced mode */
	zassert_equal(bmx160_emul_get_bmm150_reg(imu_emul, 0x4B), 0x01);
	zassert_equal(bmx160_emul_get_bmm150_reg(imu_emul, 0x51), 0x17);
	zassert_equal(bmx160_emul_get_bmm150_reg(imu_emul, 0x52), 0x52);
	zassert_equal(bmx160_emul_get_bmm150_reg(imu_emul, 0x4C), 0x02);
}

/* Python: gyro(), with GYR_SCALAR = 500 / 32768 deg/s per LSB */
ZTEST(bmx160, test_gyro_500dps)
{
	float g[3];

	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, -16384, 1000);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ));
	get3(imu, SENSOR_CHAN_GYRO_XYZ, g);

	zassert_equal(bmx160_get_gyro_range(imu), BMX160_GYRO_RANGE_500DPS);
	zassert_within(g[0], DEG_TO_RAD(250.0f), 1e-4f);
	zassert_within(g[1], DEG_TO_RAD(-250.0f), 1e-4f);
	zassert_within(g[2], DEG_TO_RAD(1000.0f * 500.0f / 32768.0f), 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_GYRO_Y), DEG_TO_RAD(-250.0f), 1e-4f);
}

/* The two ranges the ADCS task uses: 2000 deg/s detumbling, 125 deg/s nominal */
ZTEST(bmx160, test_gyro_range_change)
{
	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, 0, 0);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ));

	zassert_ok(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_2000DPS));
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x00);
	zassert_equal(bmx160_get_gyro_range(imu), BMX160_GYRO_RANGE_2000DPS);
	zassert_within(get1(imu, SENSOR_CHAN_GYRO_X), DEG_TO_RAD(1000.0f), 1e-4f);

	zassert_ok(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_125DPS));
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x04);
	zassert_within(get1(imu, SENSOR_CHAN_GYRO_X), DEG_TO_RAD(62.5f), 1e-4f);
}

/*
 * After a range change the data registers still hold an old-range sample (seen on
 * hardware: 16x off for 2000 <-> 125), so a change waits two 50 Hz gyro samples.
 */
#define RANGE_SETTLE_MS 40

static int64_t time_set_range(enum bmx160_gyro_range range, int expected_ret)
{
	int64_t start = k_uptime_get();

	zassert_equal(bmx160_set_gyro_range(imu, range), expected_ret);
	return k_uptime_get() - start;
}

ZTEST(bmx160, test_gyro_range_change_waits_for_new_samples)
{
	int64_t ms = time_set_range(BMX160_GYRO_RANGE_2000DPS, 0);

	zassert_true(ms >= RANGE_SETTLE_MS, "range change returned after %lld ms", (long long)ms);
}

/* Same range again: nothing changes on the chip, so no wait (ADCS may re-send it) */
ZTEST(bmx160, test_gyro_range_unchanged_does_not_wait)
{
	int64_t ms = time_set_range(BMX160_GYRO_RANGE_500DPS, 0);

	zassert_true(ms < RANGE_SETTLE_MS, "unchanged range took %lld ms", (long long)ms);
}

/* Failed changes leave the chip in the old range, so there is nothing to wait for */
ZTEST(bmx160, test_gyro_range_failed_change_does_not_wait)
{
	bmx160_emul_ignore_writes(imu_emul, BMX160_EMUL_REG_GYR_RANGE);
	zassert_true(time_set_range(BMX160_GYRO_RANGE_2000DPS, -EIO) < RANGE_SETTLE_MS);

	bmx160_emul_ignore_writes(imu_emul, BMX160_EMUL_NO_REG);
	bmx160_emul_set_fail(imu_emul, true);
	zassert_true(time_set_range(BMX160_GYRO_RANGE_2000DPS, -EIO) < RANGE_SETTLE_MS);
}

ZTEST(bmx160, test_gyro_range_invalid)
{
	zassert_equal(bmx160_set_gyro_range(imu, 5), -EINVAL);
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x02);
	zassert_equal(bmx160_get_gyro_range(imu), BMX160_GYRO_RANGE_500DPS);
}

/* A write the chip didn't take must not change the scale: it still holds 500 deg/s */
ZTEST(bmx160, test_gyro_range_not_applied)
{
	bmx160_emul_ignore_writes(imu_emul, BMX160_EMUL_REG_GYR_RANGE);
	zassert_equal(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_2000DPS), -EIO);
	zassert_equal(bmx160_get_gyro_range(imu), BMX160_GYRO_RANGE_500DPS);
}

ZTEST(bmx160, test_gyro_range_i2c_error)
{
	bmx160_emul_set_fail(imu_emul, true);
	zassert_equal(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_2000DPS), -EIO);
	zassert_equal(bmx160_get_gyro_range(imu), BMX160_GYRO_RANGE_500DPS);
}

/* Python: accel(), with ACC_SCALAR = 8 g / 32768 per LSB */
ZTEST(bmx160, test_accel_8g)
{
	float a[3];

	set_xyz(BMX160_EMUL_REG_DATA_ACC, 4096, -8192, 0);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ));
	get3(imu, SENSOR_CHAN_ACCEL_XYZ, a);

	zassert_within(a[0], 9.80665f, 1e-4f);
	zassert_within(a[1], -2 * 9.80665f, 1e-4f);
	zassert_within(a[2], 0.0f, 1e-6f);
}

/*
 * Python: mag(). X ADC 1000, Y ADC -500, Z ADC 2000, RHALL 6500, each with status bits set
 * in the low bits, which the shifts must drop. Python gives
 * (304.22502, -179.75915, 794.31542) uT; Zephyr reports gauss.
 */
ZTEST(bmx160, test_mag_compensated)
{
	float m[3];

	set_xyz(BMX160_EMUL_REG_DATA_MAG, (1000 << 3) | 1, (-500 * 8) | 2, (2000 << 1) | 1);
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_DATA_MAG + 6, (6500 << 2) | 1);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_MAGN_XYZ));
	get3(imu, SENSOR_CHAN_MAGN_XYZ, m);

	zassert_within(m[0], 3.0422502f, 1e-4f);
	zassert_within(m[1], -1.7975915f, 1e-4f);
	zassert_within(m[2], 7.9431542f, 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_MAGN_Z), 7.9431542f, 1e-4f);
}

/* Overflow ADC values read as 0, as in Python. Y is valid: 367.01830 uT. */
ZTEST(bmx160, test_mag_overflow)
{
	float m[3];

	set_xyz(BMX160_EMUL_REG_DATA_MAG, -4096 * 8, (1000 << 3) | 1, -16384 * 2);
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_DATA_MAG + 6, (6500 << 2) | 1);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_MAGN_XYZ));
	get3(imu, SENSOR_CHAN_MAGN_XYZ, m);

	zassert_within(m[0], 0.0f, 1e-6f);
	zassert_within(m[1], 3.6701830f, 1e-4f);
	zassert_within(m[2], 0.0f, 1e-6f);
}

/* Python: temperature(). 0 is 23 degC, 1/512 K per LSB. */
ZTEST(bmx160, test_temperature)
{
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_TEMP, 1024);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_DIE_TEMP));
	zassert_within(get1(imu, SENSOR_CHAN_DIE_TEMP), 25.0f, 1e-4f);

	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_TEMP, -512);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_DIE_TEMP));
	zassert_within(get1(imu, SENSOR_CHAN_DIE_TEMP), 22.0f, 1e-4f);
}

/* Fetching everything updates every sensor; fetching one sensor leaves the others alone */
ZTEST(bmx160, test_fetch_all_and_one)
{
	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, 0, 0);
	set_xyz(BMX160_EMUL_REG_DATA_ACC, 4096, 0, 0);
	set_xyz(BMX160_EMUL_REG_DATA_MAG, (1000 << 3) | 1, (-500 * 8) | 2, (2000 << 1) | 1);
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_DATA_MAG + 6, (6500 << 2) | 1);
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_TEMP, 1024);
	zassert_ok(sensor_sample_fetch(imu));

	zassert_within(get1(imu, SENSOR_CHAN_GYRO_X), DEG_TO_RAD(250.0f), 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_ACCEL_X), 9.80665f, 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_MAGN_X), 3.0422502f, 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_DIE_TEMP), 25.0f, 1e-4f);

	set_xyz(BMX160_EMUL_REG_DATA_GYR, -16384, 0, 0);
	set_xyz(BMX160_EMUL_REG_DATA_ACC, 0, 0, 0);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ));

	zassert_within(get1(imu, SENSOR_CHAN_GYRO_X), DEG_TO_RAD(-250.0f), 1e-4f);
	zassert_within(get1(imu, SENSOR_CHAN_ACCEL_X), 9.80665f, 1e-4f);
}

/* A failed read reports the error and keeps the previous sample */
ZTEST(bmx160, test_fetch_i2c_error)
{
	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, 0, 0);
	zassert_ok(sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ));

	bmx160_emul_set_fail(imu_emul, true);
	zassert_equal(sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ), -EIO);
	zassert_equal(sensor_sample_fetch(imu), -EIO);
	zassert_within(get1(imu, SENSOR_CHAN_GYRO_X), DEG_TO_RAD(250.0f), 1e-4f);
}

/* Python: device_errors (fatal_err, drop_cmd_err) */
ZTEST(bmx160, test_read_errors)
{
	uint8_t err;

	bmx160_emul_force_err(imu_emul, BMX160_ERR_FATAL | BMX160_ERR_DROP_CMD);
	zassert_ok(bmx160_read_errors(imu, &err));
	zassert_true(err & BMX160_ERR_FATAL);
	zassert_true(err & BMX160_ERR_DROP_CMD);
	zassert_false(err & BMX160_ERR_CODE);
}

ZTEST(bmx160, test_wrong_channel)
{
	struct sensor_value val;

	zassert_equal(sensor_sample_fetch_chan(imu, SENSOR_CHAN_LIGHT), -ENOTSUP);
	zassert_equal(sensor_channel_get(imu, SENSOR_CHAN_LIGHT, &val), -ENOTSUP);
}

/* The deferred devices below are not initialized at boot, so each test sets up its fake
 * first and then calls device_init(). A device can only be initialized once.
 */

/* A BMI160 (ID 0xD1) is not a BMX160 */
ZTEST(bmx160, test_bad_chip_id)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(imu_bad_id));

	bmx160_emul_set_chip_id(EMUL_DT_GET(DT_NODELABEL(imu_bad_id)), 0xD1);
	zassert_equal(device_init(dev), -ENODEV);
	zassert_false(device_is_ready(dev));
}

/* Python raises if ERR_REG is non-zero after a power-mode change */
ZTEST(bmx160, test_init_err_reg)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(imu_err));

	bmx160_emul_force_err(EMUL_DT_GET(DT_NODELABEL(imu_err)), BMX160_ERR_DROP_CMD);
	zassert_equal(device_init(dev), -EIO);
	zassert_false(device_is_ready(dev));
}

/* With no trim (dig_xyz1 = 0), Python falls back to ADC / 16 uT */
ZTEST(bmx160, test_mag_without_trim)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(imu_no_trim));
	const struct emul *emul = EMUL_DT_GET(DT_NODELABEL(imu_no_trim));
	float m[3];

	bmx160_emul_set_bmm150_reg(emul, 0x6C, 0);
	bmx160_emul_set_bmm150_reg(emul, 0x6D, 0);
	zassert_ok(device_init(dev));

	bmx160_emul_set_reg16(emul, BMX160_EMUL_REG_DATA_MAG, (1000 << 3) | 1);
	bmx160_emul_set_reg16(emul, BMX160_EMUL_REG_DATA_MAG + 2, -500 * 8);
	bmx160_emul_set_reg16(emul, BMX160_EMUL_REG_DATA_MAG + 4, 2000 << 1);
	bmx160_emul_set_reg16(emul, BMX160_EMUL_REG_DATA_MAG + 6, 6500 << 2);
	zassert_ok(sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ));
	get3(dev, SENSOR_CHAN_MAGN_XYZ, m);

	zassert_within(m[0], 0.625f, 1e-5f);
	zassert_within(m[1], -0.3125f, 1e-5f);
	zassert_within(m[2], 1.25f, 1e-5f);
}
