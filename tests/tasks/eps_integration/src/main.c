/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"
#include "argus/drivers/adm1176.h"
#include "argus/drivers/drv8235.h"
#include "argus/drivers/max17205.h"
#include "adm1176_emul.h"
#include "drv8235_emul.h"
#include "max17205_emul.h"

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

DEFINE_FFF_GLOBALS;

#define DEV(label) DEVICE_DT_GET(DT_NODELABEL(label))
#define EMUL(label) EMUL_DT_GET(DT_NODELABEL(label))
static const struct device *const coils[] = {
	DEV(torque_xp), DEV(torque_xm), DEV(torque_yp),
	DEV(torque_ym), DEV(torque_zp), DEV(torque_zm),
};
static const struct emul *const coil_emuls[] = {
	EMUL(torque_xp), EMUL(torque_xm), EMUL(torque_yp),
	EMUL(torque_ym), EMUL(torque_zp), EMUL(torque_zm),
};
static const struct emul *const monitor_emuls[] = {
	EMUL(main_power_monitor), EMUL(gps_power_monitor),
	EMUL(radio_power_monitor), EMUL(jetson_power_monitor),
};
static int32_t cpu_uc;
static int cpu_error;
static int temp_fetch(const struct device *dev, enum sensor_channel channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	return cpu_error;
}
static int temp_get(const struct device *dev, enum sensor_channel channel, struct sensor_value *val)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	return sensor_value_from_micro(val, cpu_uc);
}
static DEVICE_API(sensor, temp_api) = { .sample_fetch = temp_fetch, .channel_get = temp_get };
DEVICE_DT_DEFINE(DT_NODELABEL(die_temp), NULL, NULL, NULL, NULL,
		 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &temp_api);

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	cpu_uc = 25000000;
	cpu_error = 0;
	for (size_t i = 0; i < ARRAY_SIZE(coils); ++i) {
		drv8235_emul_power_cycle(coil_emuls[i]);
	}
	/* Clear injected errors on BOTH buses before any driver transaction. */
	for (size_t i = 0; i < ARRAY_SIZE(coils); ++i) {
		zassert_ok(drv8235_reconfigure(coils[i]));
	}
	for (size_t i = 0; i < ARRAY_SIZE(monitor_emuls); ++i) {
		const uint8_t bytes[] = {0x80, 0x10, 0x00}; /* V=2048, I=256 */

		adm1176_emul_set_measurement(monitor_emuls[i], bytes);
	}
	const struct emul *g = EMUL(fuel_gauge);

	max17205_emul_reset(g);
	max17205_emul_set_reg(g, 0x36, 0x06, 0x3200);
	max17205_emul_set_reg(g, 0x36, 0x05, 3000);
	max17205_emul_set_reg(g, 0x36, 0x0A, 0xFF00);
	max17205_emul_set_reg(g, 0x36, 0xDA, 6400);
	max17205_emul_set_reg(g, 0x36, 0x09, 40960);
	max17205_emul_set_reg(g, 0x36, 0x11, 11); /* floor(11*3/32) min -> 60 s */
	max17205_emul_set_reg(g, 0x36, 0x20, 32);
	max17205_emul_set_reg(g, 0x36, 0x08, 0xFF00); /* preserve unsigned spec */
	max17205_emul_set_reg(g, 0x0B, 0x34, 2731);
	max17205_emul_set_reg(g, 0x0B, 0x3B, 2781);
	max17205_emul_set_reg(g, 0x0B, 0x35, 2982);
}
static void five(struct eps_state *state, const struct eps_io *io)
{
	for (int i = 1; i <= 5; ++i) { (void)eps_step(state, io, i); }
}

ZTEST(eps_integration, test_real_drivers_feed_spec_units_and_heater_gpio)
{
	const struct eps_io *io = eps_zephyr_init();
	struct eps_state state;

	eps_init(&state);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 0), 0);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 1), 0);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 2), 0);
	zassert_ok(drv8235_set_output_uv(coils[EPS_XP], -1000000));
	drv8235_emul_set_status(coil_emuls[EPS_XP], 30, 96, 0);
	five(&state, io);
	zassert_equal(state.fields[EPS_MAIN_MV], 13175);
	zassert_equal(state.fields[EPS_MAIN_MA], 661);
	zassert_equal(state.fields[EPS_XP_MV], -5019);
	zassert_equal(state.fields[EPS_XP_MA], -1850);
	zassert_equal(state.fields[EPS_XM_MV], 0); /* same address, different bus */
	zassert_equal(state.fields[EPS_PACK_TEMP_CC], 25500);
	zassert_equal(state.fields[EPS_AIN1_TEMP_CC], 0);
	zassert_equal(state.fields[EPS_AIN2_TEMP_CC], 500);
	zassert_equal(state.fields[EPS_DIE_TEMP_CC], 2510);
	zassert_equal(state.fields[EPS_PACK_MA], -40);
	zassert_equal(state.fields[EPS_PACK_MV], 8000);
	zassert_equal(state.fields[EPS_MIDPOINT_MV], 3200);
	zassert_equal(state.fields[EPS_CAPACITY_MAH], 1500);
	zassert_equal(state.fields[EPS_TTE_S], 60);
	zassert_equal(state.fields[EPS_TTF_S], 180);
	zassert_equal(state.fields[EPS_CPU_TEMP_CC], 2500);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 0), 1);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 1), 1);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 2), 0);
	zassert_equal(state.fields[EPS_HEATER1], 1);
}

ZTEST(eps_integration, test_gauge_missing_at_boot_uses_cpu_and_latches_unavailable)
{
	max17205_emul_set_nack(EMUL(fuel_gauge), true);
	cpu_uc = 2000000;
	const struct eps_io *io = eps_zephyr_init();
	struct eps_state state;

	eps_init(&state);
	max17205_emul_set_nack(EMUL(fuel_gauge), false); /* no fault recovery */
	five(&state, io);
	zassert_equal(state.fields[EPS_SOC_PCT], 0);
	zassert_equal(state.power_flag, EPS_NONE);
	zassert_equal(state.fields[EPS_CPU_TEMP_CC], 200);
	zassert_equal(state.fields[EPS_HEATER1], 1);
	zassert_equal(state.fields[EPS_HEATER2], 1);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 0), 1);
	zassert_equal(gpio_emul_output_get(DEV(test_gpio), 2), 1);
}

ZTEST(eps_integration, test_missing_one_coil_does_not_abort_other_reads)
{
	drv8235_emul_set_error(coil_emuls[EPS_XM], -ETIMEDOUT);
	drv8235_emul_set_status(coil_emuls[EPS_XP], 5, 12, 0);
	const struct eps_io *io = eps_zephyr_init();
	struct eps_state state;

	eps_init(&state);
	five(&state, io);
	zassert_equal(state.fields[EPS_XM_MV], 0);
	zassert_equal(state.fields[EPS_XM_MA], 0);
	zassert_equal(state.fields[EPS_XP_MV], 836);
	zassert_equal(state.fields[EPS_JETSON_MV], 13175);
	zassert_equal(state.fields[EPS_SOC_PCT], 50);
}

ZTEST(eps_integration, test_raw_temperature_api_rejects_invalid_or_failed_reads)
{
	struct max17205_raw_temperatures t = {1, 2, 3, 4};
	const struct max17205_raw_temperatures expected = t;

	zassert_equal(max17205_read_raw_temperatures(NULL, &t), -EINVAL);
	zassert_equal(max17205_read_raw_temperatures(DEV(fuel_gauge), NULL), -EINVAL);
	max17205_emul_set_nack(EMUL(fuel_gauge), true);
	zassert_equal(max17205_read_raw_temperatures(DEV(fuel_gauge), &t), -EIO);
	zassert_mem_equal(&t, &expected, sizeof(t));
}

ZTEST(eps_integration, test_adm_raw_sample_preserves_nibbles)
{
	const uint8_t bytes[] = {0x12, 0xAB, 0xC3};
	struct adm1176_sample sample;

	adm1176_emul_set_measurement(EMUL(main_power_monitor), bytes);
	zassert_ok(adm1176_read_sample(DEV(main_power_monitor), &sample));
	zassert_equal(sample.raw_voltage, 0x12C);
	zassert_equal(sample.raw_current, 0xAB3);
	zassert_equal(adm1176_read_sample(NULL, &sample), -EINVAL);
	zassert_equal(adm1176_read_sample(DEV(main_power_monitor), NULL), -EINVAL);
}

ZTEST_SUITE(eps_integration, NULL, NULL, before, NULL, NULL);
