/*
 * On-board check for the BMX160 IMU: prints gyro, accelerometer, magnetometer and die
 * temperature twice a second. Every 10 seconds it switches the gyro between the two ranges
 * the ADCS task uses (2000 deg/s detumbling, 125 deg/s nominal), so you can check the
 * readings keep the same scale across a range change.
 *
 * If init failed, it reads the chip ID again to show why.
 */

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/bmx160.h"

#define IMU_NODE           DT_NODELABEL(imu)
#define BMX160_REG_CHIP_ID 0x00
#define PERIOD_MS          500
#define TICKS_PER_RANGE    20 /* 10 s */
#define RAD_TO_DEG         (180.0 / 3.14159265358979)

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(IMU_NODE);

static const char *const range_names[] = {"2000", "1000", "500", "250", "125"};

/* Repeat the driver's ID read to tell an I2C failure from "wrong part" */
static void print_init_failure(void)
{
	uint8_t id;
	int ret = i2c_reg_read_byte_dt(&imu_i2c, BMX160_REG_CHIP_ID, &id);

	if (ret < 0) {
		printk("IMU init failed: I2C error at 0x%02x (%d)\n", imu_i2c.addr, ret);
	} else if (id == 0xD8) {
		printk("IMU init failed at boot, but answers now with ID 0xD8 (power-up timing?)\n");
	} else {
		printk("IMU init failed: chip ID 0x%02x at 0x%02x, expected 0xD8%s\n", id,
		       imu_i2c.addr, id == 0xD1 ? " (0xD1 is a BMI160)" : "");
	}
}

static void get3(enum sensor_channel chan, double out[3])
{
	struct sensor_value val[3];

	(void)sensor_channel_get(imu, chan, val);
	for (int i = 0; i < 3; i++) {
		out[i] = sensor_value_to_double(&val[i]);
	}
}

static double norm(const double v[3])
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void print_sample(void)
{
	struct sensor_value temp_val;
	double g[3], a[3], m[3];
	uint8_t err = 0;
	int ret = sensor_sample_fetch(imu);

	if (ret < 0) {
		printk("read failed (%d)\n", ret);
		return;
	}

	get3(SENSOR_CHAN_GYRO_XYZ, g);
	get3(SENSOR_CHAN_ACCEL_XYZ, a);
	get3(SENSOR_CHAN_MAGN_XYZ, m);
	/* -ENODATA: the chip has no valid temperature yet (register 0x8000) */
	bool temp_ok = sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &temp_val) == 0;

	for (int i = 0; i < 3; i++) {
		m[i] *= 100.0; /* gauss -> uT */
	}

	printk("gyro  %8.4f %8.4f %8.4f rad/s  (%7.2f %7.2f %7.2f deg/s, range %s)\n", g[0],
	       g[1], g[2], g[0] * RAD_TO_DEG, g[1] * RAD_TO_DEG, g[2] * RAD_TO_DEG,
	       range_names[bmx160_get_gyro_range(imu)]);
	printk("accel %8.3f %8.3f %8.3f m/s^2  |a| %6.3f\n", a[0], a[1], a[2], norm(a));
	printk("mag   %8.2f %8.2f %8.2f uT     |B| %6.2f\n", m[0], m[1], m[2], norm(m));

	if (temp_ok) {
		printk("temp  %6.2f degC  ", sensor_value_to_double(&temp_val));
	} else {
		printk("temp     n/a       ");
	}
	if (bmx160_read_errors(imu, &err) < 0) {
		printk("ERR_REG read failed\n");
	} else {
		printk("ERR_REG 0x%02x%s%s\n", err, (err & BMX160_ERR_FATAL) ? " FATAL" : "",
		       (err & BMX160_ERR_DROP_CMD) ? " DROP_CMD" : "");
	}
}

static void switch_range(void)
{
	enum bmx160_gyro_range next = bmx160_get_gyro_range(imu) == BMX160_GYRO_RANGE_2000DPS
					      ? BMX160_GYRO_RANGE_125DPS
					      : BMX160_GYRO_RANGE_2000DPS;
	int ret = bmx160_set_gyro_range(imu, next);

	printk("=== gyro range -> %s deg/s%s\n", range_names[next], ret < 0 ? " FAILED" : "");
}

int main(void)
{
	for (unsigned int tick = 0;; tick++) {
		if (!device_is_ready(imu)) {
			print_init_failure();
			k_msleep(1000);
			continue;
		}
		if (tick % TICKS_PER_RANGE == 0) {
			switch_range();
		}
		printk("--- BMX160 demo, tick %u\n", tick);
		print_sample();
		k_msleep(PERIOD_MS);
	}
}
