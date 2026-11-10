/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <math.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/ztest.h>

#include "argus/drivers/drv8235.h"
#include "drv8235_emul.h"

DEFINE_FFF_GLOBALS;

static const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(drv8235_test));
static const struct emul *const emul = EMUL_DT_GET(DT_NODELABEL(drv8235_test));
static const struct device *const supply = DEVICE_DT_GET(DT_NODELABEL(test_supply));
static const struct device *const alt = DEVICE_DT_GET(DT_NODELABEL(drv8235_alt));
static const struct emul *const alt_emul = EMUL_DT_GET(DT_NODELABEL(drv8235_alt));
static uint8_t boot_config0, boot_config4, boot_target;
static size_t boot_writes;
static bool boot_power;

static int capture_boot(void)
{
	boot_config0 = drv8235_emul_get_reg(emul, 0x09);
	boot_config4 = drv8235_emul_get_reg(emul, 0x0D);
	boot_target = drv8235_emul_get_reg(emul, 0x0F);
	boot_writes = drv8235_emul_write_count(emul);
	boot_power = regulator_is_enabled(supply);
	return 0;
}
SYS_INIT(capture_boot, APPLICATION, 0);

static void *setup(void)
{
	zassert_true(device_is_ready(dev));
	zassert_equal(boot_config0 & 0x80, 0);
	zassert_equal(boot_config4, 0x38);
	zassert_equal(boot_target, 0xFF);
	zassert_equal(boot_writes, 0);
	zassert_false(boot_power);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	drv8235_emul_reset(emul);
	drv8235_emul_reset(alt_emul);
	if (!regulator_is_enabled(supply)) {
		zassert_ok(regulator_enable(supply));
	}
	zassert_ok(drv8235_init(dev));
	drv8235_emul_clear_writes(emul);
}

static void assert_command(uint8_t index, enum drv8235_bridge_mode mode)
{
	enum drv8235_bridge_mode actual;
	int16_t raw = 1234;
	float volts = 1234, throttle = 1234;

	zassert_equal(drv8235_emul_get_reg(emul, 0x0F), index);
	zassert_ok(drv8235_get_bridge_control(dev, &actual));
	zassert_equal(actual, mode);
	if (mode == DRV8235_BRIDGE_COAST) {
		zassert_equal(drv8235_get_throttle_raw(dev, &raw), -ENODATA);
		zassert_equal(drv8235_get_throttle_volts(dev, &volts), -ENODATA);
		zassert_equal(drv8235_get_throttle(dev, &throttle), -ENODATA);
		zassert_equal(raw, 1234);
		zassert_equal(volts, 1234);
		zassert_equal(throttle, 1234);
		return;
	}
	int sign = mode == DRV8235_BRIDGE_REVERSE ? -1 : 1;
	int16_t expected = mode == DRV8235_BRIDGE_BRAKE ? 0 : sign * index;

	zassert_ok(drv8235_get_throttle_raw(dev, &raw));
	zassert_equal(raw, expected);
	zassert_ok(drv8235_get_throttle_volts(dev, &volts));
	zassert_within(volts, expected * 0.16733f, 0.0001f);
	zassert_ok(drv8235_get_throttle(dev, &throttle));
}

ZTEST(drv8235, test_init_matches_original_settings)
{
	zassert_equal(drv8235_emul_get_reg(emul, 0x00), 0x02);
	zassert_equal(drv8235_emul_get_reg(emul, 0x09), 0xE0);
	zassert_equal(drv8235_emul_get_reg(emul, 0x0C), 0x73);
	zassert_equal(drv8235_emul_get_reg(emul, 0x0D), 0x3C);
	zassert_equal(drv8235_emul_get_reg(emul, 0x0E), 0x3F);
	zassert_equal(drv8235_emul_get_reg(emul, 0x13), 0xB3);
	zassert_equal(drv8235_emul_get_reg(emul, 0x14), 41);
	assert_command(0, DRV8235_BRIDGE_COAST);
}

ZTEST(drv8235, test_normalized_throttle_and_clamping)
{
	/* Expected codes and getters from the Python/Rust compatibility cases. */
	const struct {
		float command;
		uint8_t index;
		enum drv8235_bridge_mode mode;
		float reported;
	} cases[] = {
		{1.0f, 35, DRV8235_BRIDGE_FORWARD, 0.9742983f},
		{-0.5f, 17, DRV8235_BRIDGE_REVERSE, -0.4764817f},
		{2.0f, 35, DRV8235_BRIDGE_FORWARD, 0.9742983f},
		{-2.0f, 35, DRV8235_BRIDGE_REVERSE, -0.9742983f},
		{0.0f, 0, DRV8235_BRIDGE_BRAKE, 0.0f},
		{0.001f, 0, DRV8235_BRIDGE_BRAKE, 0.0f},
		{-0.001f, 0, DRV8235_BRIDGE_BRAKE, 0.0f},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); ++i) {
		float actual;

		zassert_ok(drv8235_set_throttle(dev, cases[i].command));
		assert_command(cases[i].index, cases[i].mode);
		zassert_ok(drv8235_get_throttle(dev, &actual));
		zassert_within(actual, cases[i].reported, 0.0001f);
	}
}

ZTEST(drv8235, test_voltage_throttle_rounding_with_safe_ceiling)
{
	const struct {
		float volts;
		uint8_t index;
		enum drv8235_bridge_mode mode;
	} cases[] = {
		{3.0f, 18, DRV8235_BRIDGE_FORWARD},
		{-6.0f, 35, DRV8235_BRIDGE_REVERSE},
		{6.0f, 35, DRV8235_BRIDGE_FORWARD},
		{0.1f, 1, DRV8235_BRIDGE_FORWARD},
		{0.001f, 0, DRV8235_BRIDGE_FORWARD},
		{0.0f, 0, DRV8235_BRIDGE_BRAKE},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); ++i) {
		zassert_ok(drv8235_set_throttle_volts(dev, cases[i].volts));
		assert_command(cases[i].index, cases[i].mode);
	}
}

ZTEST(drv8235, test_raw_throttle_handles_reverse_and_limits)
{
	const int16_t commands[] = {35, -35, -17, 17, 0};
	const uint8_t indices[] = {35, 35, 17, 17, 0};

	for (size_t i = 0; i < ARRAY_SIZE(commands); ++i) {
		zassert_ok(drv8235_set_throttle_raw(dev, commands[i]));
		assert_command(indices[i], commands[i] == 0 ? DRV8235_BRIDGE_BRAKE :
			(commands[i] < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD));
	}
}

ZTEST(drv8235, test_coil_limit_rejects_without_changing_output)
{
	const float volts[] = {6.001f, -6.001f, 42.0f, -42.0f};
	const int16_t raws[] = {36, -36, 255, -255, INT16_MIN, INT16_MAX};

	zassert_ok(drv8235_set_throttle_raw(dev, 17));
	drv8235_emul_clear_writes(emul);
	for (size_t i = 0; i < ARRAY_SIZE(volts); ++i) {
		zassert_equal(drv8235_set_throttle_volts(dev, volts[i]), -ERANGE);
	}
	for (size_t i = 0; i < ARRAY_SIZE(raws); ++i) {
		zassert_equal(drv8235_set_throttle_raw(dev, raws[i]), -ERANGE);
	}
	zassert_equal(drv8235_emul_write_count(emul), 0);
	assert_command(17, DRV8235_BRIDGE_FORWARD);
}

ZTEST(drv8235, test_board_properties_apply_per_device)
{
	float volts, amps, throttle;

	zassert_ok(drv8235_init(alt));
	/* 50 ohm selects 8192/50 -> 164, unlike the 25 ohm device's 1024/25 -> 41. */
	zassert_equal(drv8235_emul_get_reg(alt_emul, 0x13) >> 6, 3);
	zassert_equal(drv8235_emul_get_reg(alt_emul, 0x14), 164);
	zassert_ok(drv8235_set_throttle(alt, 1.0f));
	zassert_equal(drv8235_emul_get_reg(alt_emul, 0x0F), 17);
	zassert_ok(drv8235_get_throttle(alt, &throttle));
	zassert_within(throttle, 0.9529633f, 0.0001f);
	zassert_ok(drv8235_set_throttle_volts(alt, -3.0f));
	zassert_equal(drv8235_emul_get_reg(alt_emul, 0x0F), 17);
	zassert_equal(drv8235_set_throttle_volts(alt, 3.001f), -ERANGE);
	zassert_equal(drv8235_set_throttle_raw(alt, -18), -ERANGE);
	drv8235_emul_set_reg(alt_emul, 0x05, 96);
	zassert_ok(drv8235_read_voltage_current(alt, &volts, &amps));
	zassert_within(amps, -0.96f, 0.0001f); /* 96 * 1.92 / 192 */
}

ZTEST(drv8235, test_supply_off_and_initial_probe_failure_can_be_retried)
{
	zassert_ok(regulator_disable(supply));
	drv8235_emul_reset(emul);
	zassert_true(device_is_ready(dev));
	zassert_equal(drv8235_init(dev), -EAGAIN);
	zassert_equal(drv8235_set_throttle(dev, 1), -EAGAIN);
	zassert_equal(drv8235_emul_write_count(emul), 0);
	zassert_ok(regulator_enable(supply));
	drv8235_emul_set_error(emul, -ENXIO);
	zassert_equal(drv8235_init(dev), -ENXIO);
	zassert_true(device_is_ready(dev));
	drv8235_emul_set_error(emul, 0);
	zassert_ok(drv8235_init(dev));
	zassert_ok(drv8235_set_throttle(dev, 1));
}

ZTEST(drv8235, test_reversal_coasts_before_changing_target)
{
	const int16_t start[] = {17, -17};
	const int16_t end[] = {-35, 35};

	for (size_t i = 0; i < ARRAY_SIZE(start); ++i) {
		zassert_ok(drv8235_set_throttle_raw(dev, start[i]));
		drv8235_emul_clear_writes(emul);
		zassert_ok(drv8235_set_throttle_raw(dev, end[i]));
		zassert_equal(drv8235_emul_write_count(emul), 3);
		const struct drv8235_emul_write *coast = drv8235_emul_get_write(emul, 0);
		const struct drv8235_emul_write *target = drv8235_emul_get_write(emul, 1);
		const struct drv8235_emul_write *bridge = drv8235_emul_get_write(emul, 2);

		zassert_equal(coast->reg, 0x0D);
		zassert_equal(coast->value & 3, DRV8235_BRIDGE_COAST);
		zassert_equal(target->reg, 0x0F);
		zassert_equal(target->value, 35);
		zassert_equal(bridge->reg, 0x0D);
		zassert_equal(bridge->value & 3,
			      end[i] < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);
	}
}

ZTEST(drv8235, test_coast_is_distinct_from_zero_brake)
{
	zassert_ok(drv8235_set_throttle(dev, 1));
	zassert_ok(drv8235_set_throttle(dev, 0));
	assert_command(0, DRV8235_BRIDGE_BRAKE);
	zassert_ok(drv8235_coast(dev));
	assert_command(0, DRV8235_BRIDGE_COAST);
	zassert_equal(drv8235_emul_get_reg(emul, 0x09) & 0x80, 0x80);
}

ZTEST(drv8235, test_same_direction_updates_do_not_insert_coast)
{
	zassert_ok(drv8235_set_throttle(dev, 0.5f));
	drv8235_emul_clear_writes(emul);
	zassert_ok(drv8235_set_throttle(dev, 1));
	zassert_equal(drv8235_emul_write_count(emul), 2);
	const struct drv8235_emul_write *target = drv8235_emul_get_write(emul, 0);
	const struct drv8235_emul_write *bridge = drv8235_emul_get_write(emul, 1);

	zassert_not_null(target);
	zassert_not_null(bridge);
	zassert_equal(target->reg, 0x0F);
	zassert_equal(target->value, 35);
	zassert_equal(bridge->reg, 0x0D);
	zassert_equal(bridge->value & 3, DRV8235_BRIDGE_FORWARD);
}

ZTEST(drv8235, test_voltage_current_follow_eps_conversion_in_all_modes)
{
	const enum drv8235_bridge_mode modes[] = {
		DRV8235_BRIDGE_FORWARD, DRV8235_BRIDGE_REVERSE,
		DRV8235_BRIDGE_BRAKE, DRV8235_BRIDGE_COAST,
	};

	const struct {
		uint8_t raw;
		float amps;
	} currents[] = {
		{0, 0.0f},
		{50, 0.963541667f},
		{96, 1.85f},
		{192, 3.7f},
	};

	drv8235_emul_set_reg(emul, 0x04, 100);
	for (size_t i = 0; i < ARRAY_SIZE(modes); ++i) {
		float volts, amps;
		int sign = modes[i] == DRV8235_BRIDGE_REVERSE ? -1 : 1;

		drv8235_emul_set_reg(emul, 0x0D, 0x3C | modes[i]);
		for (size_t j = 0; j < ARRAY_SIZE(currents); ++j) {
			drv8235_emul_set_reg(emul, 0x05, currents[j].raw);
			zassert_ok(drv8235_read_voltage_current(dev, &volts, &amps));
			zassert_within(volts, sign * 16.733f, 0.0001f);
			zassert_within(amps, sign * currents[j].amps, 0.0001f);
		}
	}
	/* In particular, brake must retain nonzero IMTR, not fabricate zero. */
}

ZTEST(drv8235, test_fault_read_decodes_and_clears)
{
	struct drv8235_faults faults;

	zassert_ok(drv8235_set_throttle(dev, 1));
	drv8235_emul_set_reg(emul, 0x00, 0xBE);
	zassert_ok(drv8235_get_faults(dev, &faults));
	zassert_equal(faults.raw, 0xBE);
	zassert_true(faults.stall);
	zassert_true(faults.overcurrent);
	zassert_true(faults.overvoltage);
	zassert_true(faults.thermal_shutdown);
	zassert_false(faults.power_on_reset);
	zassert_equal(drv8235_emul_get_reg(emul, 0x00), 0x02);
	assert_command(0, DRV8235_BRIDGE_COAST);
}

ZTEST(drv8235, test_normal_npor_does_not_report_fault_or_write)
{
	struct drv8235_faults faults;

	zassert_ok(drv8235_get_faults(dev, &faults));
	zassert_false(faults.power_on_reset);
	zassert_false(faults.stall || faults.overcurrent || faults.overvoltage ||
		      faults.thermal_shutdown);
	zassert_equal(drv8235_emul_write_count(emul), 0);
}

ZTEST(drv8235, test_reset_requires_init_even_after_clear)
{
	struct drv8235_faults faults;

	drv8235_emul_reset(emul);
	zassert_ok(drv8235_get_faults(dev, &faults));
	zassert_true(faults.power_on_reset);
	zassert_equal(drv8235_set_throttle(dev, 1), -EAGAIN);
	zassert_ok(drv8235_clear_faults(dev));
	zassert_equal(drv8235_set_throttle(dev, 1), -EAGAIN);
	zassert_ok(drv8235_init(dev));
	zassert_ok(drv8235_set_throttle(dev, 1));
}

ZTEST(drv8235, test_latched_fault_does_not_gate_setters_or_zero_brake)
{
	drv8235_emul_set_reg(emul, 0x00, 0x92);
	zassert_ok(drv8235_set_throttle(dev, 1));
	assert_command(35, DRV8235_BRIDGE_FORWARD);
	zassert_ok(drv8235_set_throttle_volts(dev, -3));
	assert_command(18, DRV8235_BRIDGE_REVERSE);
	zassert_ok(drv8235_set_throttle_raw(dev, 0));
	assert_command(0, DRV8235_BRIDGE_BRAKE);
	zassert_equal(drv8235_emul_get_reg(emul, 0x00), 0x92);
	zassert_ok(drv8235_clear_faults(dev));
	assert_command(0, DRV8235_BRIDGE_COAST);
	zassert_ok(drv8235_set_throttle(dev, 1));
}

ZTEST(drv8235, test_reinit_disables_before_protected_writes)
{
	zassert_ok(drv8235_set_throttle(dev, 1));
	drv8235_emul_clear_writes(emul);
	zassert_ok(drv8235_init(dev));
	const struct drv8235_emul_write *first = drv8235_emul_get_write(emul, 0);

	zassert_not_null(first);
	zassert_equal(first->reg, 0x09);
	zassert_equal(first->value & 0x80, 0);
	assert_command(0, DRV8235_BRIDGE_COAST);
}

ZTEST(drv8235, test_i2c_errors_propagate_without_publishing_readings)
{
	float volts = 123, amps = 456;
	struct drv8235_faults faults;

	drv8235_emul_set_error(emul, -ETIMEDOUT);
	zassert_equal(drv8235_read_voltage_current(dev, &volts, &amps), -ETIMEDOUT);
	zassert_equal(volts, 123);
	zassert_equal(amps, 456);
	zassert_equal(drv8235_set_throttle(dev, 1), -ETIMEDOUT);
	zassert_equal(drv8235_get_faults(dev, &faults), -ETIMEDOUT);
	zassert_equal(drv8235_clear_faults(dev), -ETIMEDOUT);
	zassert_equal(drv8235_coast(dev), -ETIMEDOUT);
}

ZTEST(drv8235, test_failed_command_returns_error_and_attempts_coast)
{
	zassert_ok(drv8235_set_throttle(dev, 1));
	drv8235_emul_fail_next_write(emul, 0x0D, -EIO);
	zassert_equal(drv8235_set_throttle(dev, -1), -EIO);
	assert_command(0, DRV8235_BRIDGE_COAST);
}

ZTEST(drv8235, test_failed_init_can_be_retried)
{
	drv8235_emul_fail_next_write(emul, 0x14, -EIO);
	zassert_equal(drv8235_init(dev), -EIO);
	zassert_equal(drv8235_emul_get_reg(emul, 0x09) & 0x80, 0);
	zassert_equal(drv8235_set_throttle(dev, 1), -EAGAIN);
	zassert_ok(drv8235_init(dev));
	zassert_ok(drv8235_set_throttle(dev, 1));
}

ZTEST(drv8235, test_invalid_arguments_do_not_write)
{
	float value;
	enum drv8235_bridge_mode mode;

	zassert_equal(drv8235_set_throttle(dev, NAN), -EINVAL);
	zassert_equal(drv8235_set_throttle_volts(dev, INFINITY), -EINVAL);
	zassert_equal(drv8235_set_throttle(NULL, 1), -EINVAL);
	zassert_equal(drv8235_get_throttle(dev, NULL), -EINVAL);
	zassert_equal(drv8235_get_throttle_volts(NULL, &value), -EINVAL);
	zassert_equal(drv8235_get_throttle_raw(dev, NULL), -EINVAL);
	zassert_equal(drv8235_get_bridge_control(NULL, &mode), -EINVAL);
	zassert_equal(drv8235_read_voltage_current(dev, &value, NULL), -EINVAL);
	zassert_equal(drv8235_emul_write_count(emul), 0);
}

ZTEST_SUITE(drv8235, NULL, setup, before, NULL, NULL);
