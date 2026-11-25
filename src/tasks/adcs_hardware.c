/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Driver-backed ADCS callbacks. So far only the IMU (BMX160) is connected; the light
 * sensors, coils and satellite state still come from adcs_console_only_io().
 */
#include "argus/tasks/adcs.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/bmx160.h"

LOG_MODULE_REGISTER(adcs_hardware);

static const struct device *const imu = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(imu));

static struct {
	int gyro_error;
	int mag_error;
	int range_error;
} hardware;

/* Log the first failure and each change of errno, like eps_hardware.c. A success clears
 * the last error, so the next outage is logged again.
 */
static int report_io_result(const char *operation, int ret, int *last_error)
{
	if (ret != 0 && ret != *last_error) {
		LOG_WRN("BMX160 %s failed (%d)", operation, ret);
	}
	*last_error = ret;
	return ret;
}

/* Python SATELLITE.IMU_AVAILABLE: no IMU means every IMU callback fails. */
static int fetch_xyz(enum sensor_channel chan, float out[3], double scale)
{
	struct sensor_value v[3];

	int ret = device_is_ready(imu) ? sensor_sample_fetch_chan(imu, chan) : -ENODEV;

	if (ret == 0) {
		ret = sensor_channel_get(imu, chan, v);
	}
	if (ret == 0) {
		for (int i = 0; i < 3; ++i) {
			out[i] = (float)(sensor_value_to_double(&v[i]) * scale);
		}
	}
	return ret;
}

/* Python IMU.gyro(): rad/s, scaled with the range the sample was taken in. */
static int read_gyro(void *context, float rad_s[3])
{
	ARG_UNUSED(context);
	return report_io_result("gyro read", fetch_xyz(SENSOR_CHAN_GYRO_XYZ, rad_s, 1.0),
				&hardware.gyro_error);
}

/* Python IMU.mag() is in uT; the Zephyr channel is in gauss (1 G = 100 uT). */
static int read_mag(void *context, float ut[3])
{
	ARG_UNUSED(context);
	return report_io_result("mag read", fetch_xyz(SENSOR_CHAN_MAGN_XYZ, ut, 100.0),
				&hardware.mag_error);
}

/* Python IMU.gyro_range setter. The driver reads the register back and waits until the
 * next sample is in the new range, so the following read_gyro is scaled correctly.
 */
static int set_gyro_range(void *context, enum adcs_gyro_range range)
{
	enum bmx160_gyro_range code;

	ARG_UNUSED(context);
	switch (range) {
	case ADCS_GYRO_RANGE_2000DPS:
		code = BMX160_GYRO_RANGE_2000DPS;
		break;
	case ADCS_GYRO_RANGE_125DPS:
		code = BMX160_GYRO_RANGE_125DPS;
		break;
	default:
		return -EINVAL;
	}
	int ret = device_is_ready(imu) ? bmx160_set_gyro_range(imu, code) : -ENODEV;

	return report_io_result("gyro range write", ret, &hardware.range_error);
}

const struct adcs_io *adcs_hardware_io(void)
{
	static struct adcs_io io;

	if (!device_is_ready(imu)) {
		LOG_WRN("BMX160 not ready; ADCS gyro and magnetometer reads will fail");
	}
	io = *adcs_console_only_io();
	io.read_gyro = read_gyro;
	io.read_mag = read_mag;
	io.set_gyro_range = set_gyro_range;
	return &io;
}
