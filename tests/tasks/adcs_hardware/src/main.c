/* SPDX-License-Identifier: Apache-2.0 */
/*
 * ADCS IMU callbacks (adcs_hardware_io) with the real BMX160 driver against the fake
 * IMU from tests/drivers/bmx160. Expected magnetometer values are the ones that test
 * takes from flight/hal/drivers/bmx160.py for the same raw registers and trim.
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/ztest.h>

#include "argus/drivers/bmx160.h"
#include "argus/tasks/adcs.h"
#include "bmx160_emul.h"

#define IMU_NODE DT_NODELABEL(imu)

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct emul *const imu_emul = EMUL_DT_GET(IMU_NODE);

#define PI_F            3.14159265f
#define DEG_TO_RAD(dps) ((dps) * PI_F / 180.0f)

/* Python mag() for these registers, in uT */
#define MAG_X_UT 304.22502f
#define MAG_Y_UT (-179.75915f)
#define MAG_Z_UT 794.31542f

static void set_xyz(uint8_t reg, int16_t x, int16_t y, int16_t z)
{
	bmx160_emul_set_reg16(imu_emul, reg, x);
	bmx160_emul_set_reg16(imu_emul, reg + 2, y);
	bmx160_emul_set_reg16(imu_emul, reg + 4, z);
}

static void set_mag_registers(void)
{
	set_xyz(BMX160_EMUL_REG_DATA_MAG, (1000 << 3) | 1, (-500 * 8) | 2, (2000 << 1) | 1);
	bmx160_emul_set_reg16(imu_emul, BMX160_EMUL_REG_DATA_MAG + 6, (6500 << 2) | 1);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	bmx160_emul_set_fail(imu_emul, false);
	bmx160_emul_force_err(imu_emul, 0);
	bmx160_emul_ignore_writes(imu_emul, BMX160_EMUL_NO_REG);
	bmx160_emul_fail_reads_of(imu_emul, BMX160_EMUL_NO_REG);
	zassert_ok(bmx160_set_gyro_range(imu, BMX160_GYRO_RANGE_500DPS));
}

ZTEST_SUITE(adcs_hardware, NULL, NULL, before, NULL, NULL);

/* Python IMU.mag() is in uT; the driver channel is in gauss. */
ZTEST(adcs_hardware, test_mag_in_microtesla)
{
	const struct adcs_io *io = adcs_hardware_io();
	float ut[3];

	set_mag_registers();
	zassert_ok(io->read_mag(io->context, ut));
	zassert_within(ut[0], MAG_X_UT, 1e-2f);
	zassert_within(ut[1], MAG_Y_UT, 1e-2f);
	zassert_within(ut[2], MAG_Z_UT, 1e-2f);
}

/* Python IMU.gyro() is in rad/s. */
ZTEST(adcs_hardware, test_gyro_in_rad_s)
{
	const struct adcs_io *io = adcs_hardware_io();
	float g[3];

	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, -16384, 0);
	zassert_ok(io->read_gyro(io->context, g));
	zassert_within(g[0], DEG_TO_RAD(250.0f), 1e-4f);
	zassert_within(g[1], DEG_TO_RAD(-250.0f), 1e-4f);
	zassert_within(g[2], 0.0f, 1e-6f);
}

/* The two ranges the task uses map to the Python gyro_range codes, and the next gyro
 * read is scaled with the new range.
 */
ZTEST(adcs_hardware, test_gyro_range)
{
	const struct adcs_io *io = adcs_hardware_io();
	float g[3];

	set_xyz(BMX160_EMUL_REG_DATA_GYR, 16384, 0, 0);

	zassert_ok(io->set_gyro_range(io->context, ADCS_GYRO_RANGE_2000DPS));
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x00);
	zassert_ok(io->read_gyro(io->context, g));
	zassert_within(g[0], DEG_TO_RAD(1000.0f), 1e-4f);

	zassert_ok(io->set_gyro_range(io->context, ADCS_GYRO_RANGE_125DPS));
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x04);
	zassert_ok(io->read_gyro(io->context, g));
	zassert_within(g[0], DEG_TO_RAD(62.5f), 1e-4f);
}

/* Not one of the task's ranges: rejected without touching the chip. */
ZTEST(adcs_hardware, test_gyro_range_invalid)
{
	const struct adcs_io *io = adcs_hardware_io();

	zassert_equal(io->set_gyro_range(io->context, (enum adcs_gyro_range)2), -EINVAL);
	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x02);
}

/* An I2C failure reaches the task as an error, so it marks the reading invalid. */
ZTEST(adcs_hardware, test_i2c_failure)
{
	const struct adcs_io *io = adcs_hardware_io();
	float v[3];

	bmx160_emul_set_fail(imu_emul, true);
	zassert_not_equal(io->read_gyro(io->context, v), 0);
	zassert_not_equal(io->read_mag(io->context, v), 0);
	zassert_not_equal(io->set_gyro_range(io->context, ADCS_GYRO_RANGE_2000DPS), 0);
}

static struct {
	struct adcs_record record;
	uint16_t valid;
	int count;
} published;

static void capture(void *context, const struct adcs_record *record, uint16_t valid)
{
	ARG_UNUSED(context);
	published.record = *record;
	published.valid = valid;
	published.count++;
}

/*
 * One task step on the IMU callbacks. The state is fixed at DETUMBLING, so the step
 * records IMU readings in SI units and then switches the gyro to 2000 deg/s.
 */
ZTEST(adcs_hardware, test_step_uses_imu)
{
	struct adcs_io io = *adcs_hardware_io();
	struct adcs_state state;

	io.publish = capture;
	memset(&published, 0, sizeof(published));
	set_mag_registers();
	set_xyz(BMX160_EMUL_REG_DATA_GYR, 1000, -2000, 3000); /* at 500 deg/s */

	adcs_init(&state, NULL);
	adcs_step(&state, &io, 1);

	zassert_equal(published.count, 1);
	zassert_equal(published.valid & (ADCS_VALID_GYRO | ADCS_VALID_MAG),
		      ADCS_VALID_GYRO | ADCS_VALID_MAG, "valid 0x%03x", published.valid);
	zassert_equal(published.valid & ~(ADCS_VALID_GYRO | ADCS_VALID_MAG), 0,
		      "only the IMU is connected: 0x%03x", published.valid);

	const float lsb = DEG_TO_RAD(500.0f / 32768.0f);

	zassert_within(published.record.gyro[0], 1000 * lsb, 1e-5f);
	zassert_within(published.record.gyro[1], -2000 * lsb, 1e-5f);
	zassert_within(published.record.gyro[2], 3000 * lsb, 1e-5f);
	/* The record holds tesla: no bias, unit scale */
	zassert_within(published.record.mag[0], MAG_X_UT * 1e-6f, 1e-8f);
	zassert_within(published.record.mag[1], MAG_Y_UT * 1e-6f, 1e-8f);
	zassert_within(published.record.mag[2], MAG_Z_UT * 1e-6f, 1e-8f);

	zassert_equal(bmx160_emul_get_reg(imu_emul, BMX160_EMUL_REG_GYR_RANGE), 0x00,
		      "DETUMBLING uses 2000 deg/s");
}
