/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/init.h>
#include <zephyr/fff.h>
#include <zephyr/ztest.h>

DEFINE_FFF_GLOBALS;

#include "argus/drivers/drv8235.h"
#include "drv8235_emul.h"

enum {
	DRV8235_REG_FAULT_STATUS = 0x00,
	DRV8235_REG_CONFIG0 = 0x09,
	DRV8235_REG_CONFIG3 = 0x0C,
	DRV8235_REG_CONFIG4 = 0x0D,
	DRV8235_REG_REG_CTRL0 = 0x0E,
	DRV8235_REG_TARGET = 0x0F,
	DRV8235_REG_RC_CTRL2 = 0x13,
	DRV8235_REG_RC_CTRL3 = 0x14,
};

struct drv8235_hardware_fixture {
	const struct device *dev;
	const struct emul *emul;
	const struct device *supply;
};

static uint8_t boot_fault_status;
static uint8_t boot_config0;
static uint8_t boot_config3;

static int capture_boot_configuration(void)
{
	const struct emul *emul = EMUL_DT_GET(DT_NODELABEL(drv8235_test));

	boot_fault_status = drv8235_emul_get_reg(emul, DRV8235_REG_FAULT_STATUS);
	boot_config0 = drv8235_emul_get_reg(emul, DRV8235_REG_CONFIG0);
	boot_config3 = drv8235_emul_get_reg(emul, DRV8235_REG_CONFIG3);
	return 0;
}

SYS_INIT(capture_boot_configuration, APPLICATION, 0);

static void *drv8235_hardware_setup(void)
{
	static struct drv8235_hardware_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(drv8235_test)),
		.emul = EMUL_DT_GET(DT_NODELABEL(drv8235_test)),
		.supply = DEVICE_DT_GET(DT_NODELABEL(test_coil_power)),
	};

	zassert_true(device_is_ready(fixture.dev));
	/* Check real boot initialization before the per-test resets run. */
	zassert_equal(boot_fault_status, 0x02U);
	zassert_equal(boot_config0, 0x40U);
	zassert_equal(boot_config3, 0x70U);
	return &fixture;
}

static void drv8235_hardware_before(void *fixture_ptr)
{
	struct drv8235_hardware_fixture *fixture = fixture_ptr;

	if (!regulator_is_enabled(fixture->supply)) {
		zassert_ok(regulator_enable(fixture->supply));
	}
	drv8235_emul_power_cycle(fixture->emul);
	zassert_ok(drv8235_reconfigure(fixture->dev));
	drv8235_emul_clear_writes(fixture->emul);
}

static void assert_write(const struct emul *emul, size_t index, uint8_t reg, uint8_t value)
{
	const struct drv8235_emul_write *write = drv8235_emul_get_write(emul, index);

	zassert_not_null(write);
	zassert_equal(write->reg, reg, "write %u has wrong register", (unsigned int)index);
	zassert_equal(write->value, value, "write %u has wrong value", (unsigned int)index);
}

ZTEST_F(drv8235_hardware, test_init_programs_safe_voltage_mode)
{
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0), 0x40U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4), 0x1CU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_REG_CTRL0), 0x1FU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG3), 0x70U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_RC_CTRL2), 0xB3U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_RC_CTRL3), 41U);
}

ZTEST_F(drv8235_hardware, test_reconfigure_after_power_cycle)
{
	drv8235_emul_power_cycle(fixture->emul);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0xFFU);

	zassert_ok(drv8235_reconfigure(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0), 0x40U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4), 0x1CU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_REG_CTRL0), 0x1FU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x02U);
}

ZTEST_F(drv8235_hardware, test_voltage_command_does_not_arm_output)
{
	bool enabled = true;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 6000000));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 35U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4) & 0x03U,
		      DRV8235_BRIDGE_FORWARD);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);
}

ZTEST_F(drv8235_hardware, test_reverse_command_and_coast)
{
	zassert_ok(drv8235_set_output_uv(fixture->dev, -5000000));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 30U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4) & 0x03U,
		      DRV8235_BRIDGE_REVERSE);

	zassert_ok(drv8235_coast(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4) & 0x03U,
		      DRV8235_BRIDGE_COAST);
}

ZTEST_F(drv8235_hardware, test_enable_rejects_fault_then_succeeds)
{
	bool enabled;

	drv8235_emul_set_faults(fixture->emul, 0x92U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EIO);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);

	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_true(enabled);

	zassert_ok(drv8235_set_enabled(fixture->dev, false));
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
}

ZTEST_F(drv8235_hardware, test_brake_is_explicit)
{
	zassert_ok(drv8235_brake(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4) & 0x03U,
		      DRV8235_BRIDGE_BRAKE);
}

ZTEST_F(drv8235_hardware, test_telemetry_uses_actual_hardware_state)
{
	struct drv8235_telemetry telemetry;

	drv8235_emul_set_status(fixture->emul, 30U, 0x60U, 0x20U);
	zassert_ok(drv8235_set_output_uv(fixture->dev, -5000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_equal(telemetry.raw_voltage, 30U);
	zassert_equal(telemetry.raw_current, 0x60U);
	zassert_equal(telemetry.raw_duty, 0x20U);
	zassert_equal(telemetry.voltage_uv, -5000000);
	zassert_equal(telemetry.current_ua, 1850000U);
	zassert_true(telemetry.current_valid);
	zassert_equal(telemetry.duty_permille, 508U);
	zassert_equal(telemetry.bridge_mode, DRV8235_BRIDGE_REVERSE);
	zassert_true(telemetry.output_enabled);
}

ZTEST_F(drv8235_hardware, test_fault_read_does_not_clear_until_requested)
{
	struct drv8235_faults faults;

	drv8235_emul_set_faults(fixture->emul, 0x96U);
	zassert_ok(drv8235_get_faults(fixture->dev, &faults));
	zassert_true(faults.any);
	zassert_true(faults.overcurrent);
	zassert_true(faults.thermal_shutdown);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x96U);

	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x02U);
}

ZTEST_F(drv8235_hardware, test_limits_are_exposed)
{
	struct drv8235_limits limits;

	zassert_ok(drv8235_get_limits(fixture->dev, &limits));
	zassert_equal(limits.max_output_uv, 6000000U);
	zassert_equal(limits.coil_resistance_milliohm, 25000U);
	zassert_equal(limits.imtr_full_scale_ua, 3700000U);
}

ZTEST_F(drv8235_hardware, test_i2c_errors_propagate)
{
	struct drv8235_faults faults;

	drv8235_emul_set_error(fixture->emul, -ETIMEDOUT);
	zassert_equal(drv8235_get_faults(fixture->dev, &faults), -ETIMEDOUT);
	zassert_equal(drv8235_set_output_uv(fixture->dev, 1000000), -ETIMEDOUT);
}

ZTEST_F(drv8235_hardware, test_unexpected_reset_blocks_enable_until_reconfigured)
{
	bool enabled;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_power_cycle(fixture->emul);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);
	zassert_ok(drv8235_reconfigure(fixture->dev));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
}

ZTEST_F(drv8235_hardware, test_fault_clear_cannot_hide_unconfigured_reset)
{
	drv8235_emul_power_cycle(fixture->emul);
	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x02U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_equal(drv8235_set_output_uv(fixture->dev, 1000000), -EAGAIN);
	zassert_ok(drv8235_reconfigure(fixture->dev));
	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
}

ZTEST_F(drv8235_hardware, test_hardware_settings_are_checked_even_with_npor_high)
{
	static const struct {
		uint8_t reg;
		uint8_t toggle;
	} changes[] = {
		{DRV8235_REG_CONFIG0, 0x20U}, /* Stall detection accidentally enabled. */
		{DRV8235_REG_CONFIG3, 0x02U}, /* Auto-retry accidentally enabled. */
		{DRV8235_REG_CONFIG4, 0x04U}, /* Physical input pins take control. */
		{DRV8235_REG_REG_CTRL0, 0x20U}, /* Soft start accidentally enabled. */
		{DRV8235_REG_RC_CTRL2, 0x40U},
		{DRV8235_REG_RC_CTRL3, 0x01U},
	};

	for (size_t i = 0; i < ARRAY_SIZE(changes); ++i) {
		zassert_ok(drv8235_reconfigure(fixture->dev));
		uint8_t value = drv8235_emul_get_reg(fixture->emul, changes[i].reg);

		drv8235_emul_set_reg(fixture->emul, changes[i].reg, value ^ changes[i].toggle);
		zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
		zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0) & 0x80U,
			      0U);
	}
}

ZTEST_F(drv8235_hardware, test_separately_cleared_reset_still_fails_readback)
{
	/* Simulate a reset followed by a CLR_FLT from outside the driver. */
	drv8235_emul_power_cycle(fixture->emul);
	drv8235_emul_set_faults(fixture->emul, 0x02U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
}

ZTEST_F(drv8235_hardware, test_out_of_limit_hardware_target_cannot_be_enabled)
{
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_TARGET, 36U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
}

ZTEST_F(drv8235_hardware, test_failed_reconfigure_requires_successful_retry)
{
	/* Even with otherwise valid settings and NPOR=1, a failed write invalidates init. */
	drv8235_emul_fail_next_write(fixture->emul, DRV8235_REG_RC_CTRL3, -ETIMEDOUT);
	zassert_equal(drv8235_reconfigure(fixture->dev), -ETIMEDOUT);
	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_ok(drv8235_reconfigure(fixture->dev));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
}

ZTEST_F(drv8235_hardware, test_reconfigure_detects_acknowledged_but_lost_write)
{
	drv8235_emul_power_cycle(fixture->emul);
	drv8235_emul_ignore_next_write(fixture->emul, DRV8235_REG_CONFIG3);
	zassert_equal(drv8235_reconfigure(fixture->dev), -EAGAIN);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0) & 0x80U, 0U);
}

ZTEST_F(drv8235_hardware, test_same_direction_update_never_coasts)
{
	for (int sign = -1; sign <= 1; sign += 2) {
		zassert_ok(drv8235_set_output_uv(fixture->dev, sign * 1000000));
		zassert_ok(drv8235_set_enabled(fixture->dev, true));
		drv8235_emul_clear_writes(fixture->emul);
		zassert_ok(drv8235_set_output_uv(fixture->dev, sign * 5000000));
		zassert_equal(drv8235_emul_write_count(fixture->emul), 1U);
		assert_write(fixture->emul, 0U, DRV8235_REG_TARGET, 30U);
		zassert_true(drv8235_emul_get_write(fixture->emul, 0U)->output_was_enabled);
	}
}

ZTEST_F(drv8235_hardware, test_direction_reversal_orders_coast_target_direction)
{
	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_clear_writes(fixture->emul);
	zassert_ok(drv8235_set_output_uv(fixture->dev, -5000000));
	zassert_equal(drv8235_emul_write_count(fixture->emul), 3U);
	assert_write(fixture->emul, 0U, DRV8235_REG_CONFIG4, 0x1CU);
	assert_write(fixture->emul, 1U, DRV8235_REG_TARGET, 30U);
	assert_write(fixture->emul, 2U, DRV8235_REG_CONFIG4, 0x1DU);
}

ZTEST_F(drv8235_hardware, test_disable_orders_output_off_target_zero_coast)
{
	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_clear_writes(fixture->emul);
	zassert_ok(drv8235_set_enabled(fixture->dev, false));
	zassert_equal(drv8235_emul_write_count(fixture->emul), 3U);
	assert_write(fixture->emul, 0U, DRV8235_REG_CONFIG0, 0x40U);
	assert_write(fixture->emul, 1U, DRV8235_REG_TARGET, 0U);
	assert_write(fixture->emul, 2U, DRV8235_REG_CONFIG4, 0x1CU);
}

ZTEST_F(drv8235_hardware, test_reconfigure_disables_before_protected_fields)
{
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	/* Model stale settings while enabled; only the first EN_OUT write is legal. */
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG3, 0x63U);
	drv8235_emul_clear_writes(fixture->emul);
	zassert_ok(drv8235_reconfigure(fixture->dev));
	assert_write(fixture->emul, 0U, DRV8235_REG_CONFIG0, 0x40U);
	for (size_t i = 1; i < drv8235_emul_write_count(fixture->emul); ++i) {
		zassert_false(drv8235_emul_get_write(fixture->emul, i)->output_was_enabled);
	}
	zassert_equal(drv8235_emul_protected_violations(fixture->emul), 0U);
}

ZTEST_F(drv8235_hardware, test_emulator_rejects_protected_write_while_enabled)
{
	const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_NODELABEL(drv8235_test));
	const uint8_t message[] = {DRV8235_REG_CONFIG3, 0x73U};

	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_equal(i2c_write_dt(&bus, message, sizeof(message)), -EACCES);
	zassert_equal(drv8235_emul_protected_violations(fixture->emul), 1U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG3), 0x70U);
}

ZTEST_F(drv8235_hardware, test_target_write_failure_attempts_all_safe_steps)
{
	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_clear_writes(fixture->emul);
	drv8235_emul_fail_next_write(fixture->emul, DRV8235_REG_TARGET, -ETIMEDOUT);
	zassert_equal(drv8235_set_output_uv(fixture->dev, 5000000), -ETIMEDOUT);
	assert_write(fixture->emul, 0U, DRV8235_REG_CONFIG0, 0x40U);
	assert_write(fixture->emul, 1U, DRV8235_REG_TARGET, 0U);
	assert_write(fixture->emul, 2U, DRV8235_REG_CONFIG4, 0x1CU);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
}

ZTEST_F(drv8235_hardware, test_clear_faults_never_restarts_previous_output)
{
	bool enabled;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_faults(fixture->emul, 0x92U);
	drv8235_emul_clear_writes(fixture->emul);
	zassert_ok(drv8235_clear_faults(fixture->dev));
	assert_write(fixture->emul, 0U, DRV8235_REG_CONFIG0, 0x40U);
	assert_write(fixture->emul, 1U, DRV8235_REG_TARGET, 0U);
	assert_write(fixture->emul, 2U, DRV8235_REG_CONFIG4, 0x1CU);
	assert_write(fixture->emul, 3U, DRV8235_REG_CONFIG0, 0x42U);
	zassert_false(drv8235_emul_get_write(fixture->emul, 3U)->output_was_enabled);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
}

ZTEST_F(drv8235_hardware, test_clear_fault_is_not_issued_if_disable_fails)
{
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_faults(fixture->emul, 0x92U);
	drv8235_emul_fail_next_write(fixture->emul, DRV8235_REG_CONFIG0, -ETIMEDOUT);
	drv8235_emul_clear_writes(fixture->emul);
	zassert_equal(drv8235_clear_faults(fixture->dev), -ETIMEDOUT);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x92U);
	for (size_t i = 0; i < drv8235_emul_write_count(fixture->emul); ++i) {
		const struct drv8235_emul_write *write = drv8235_emul_get_write(fixture->emul, i);

		zassert_false(write->reg == DRV8235_REG_CONFIG0 && (write->value & 0x02U) != 0U);
	}
}

ZTEST_F(drv8235_hardware, test_active_fault_cannot_be_cleared_into_enabled_output)
{
	drv8235_emul_set_active_faults(fixture->emul, 0x14U);
	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EIO);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG3) & 0x03U, 0U);
	drv8235_emul_set_active_faults(fixture->emul, 0U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EIO, "fault stays latched");
	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
}

ZTEST_F(drv8235_hardware, test_acknowledged_disable_must_read_back_before_fault_clear)
{
	bool enabled;

	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_faults(fixture->emul, 0x92U);
	drv8235_emul_ignore_next_write(fixture->emul, DRV8235_REG_CONFIG0);
	zassert_equal(drv8235_clear_faults(fixture->dev), -EIO);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_true(enabled, "a lost write must not be reported as a successful shutdown");
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x92U);
}

ZTEST_F(drv8235_hardware, test_lost_disable_cannot_change_protected_settings)
{
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG3, 0x63U);
	drv8235_emul_ignore_next_write(fixture->emul, DRV8235_REG_CONFIG0);
	zassert_equal(drv8235_reconfigure(fixture->dev), -EIO);
	zassert_equal(drv8235_emul_protected_violations(fixture->emul), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG3), 0x63U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
}

ZTEST_F(drv8235_hardware, test_bad_imtr_preserves_other_telemetry)
{
	struct drv8235_telemetry telemetry;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_status(fixture->emul, 30U, 0xFFU, 0x20U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_false(telemetry.current_valid);
	zassert_equal(telemetry.current_ua, 0);
	zassert_equal(telemetry.raw_current, 0xFFU);
	zassert_equal(telemetry.voltage_uv, 5000000);
	zassert_equal(telemetry.duty_permille, 508U);
	drv8235_emul_set_status(fixture->emul, 30U, 0xC0U, 0x20U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_true(telemetry.current_valid);
	zassert_equal(telemetry.current_ua, 3700000);
}

ZTEST_F(drv8235_hardware, test_brake_reports_nonzero_coil_current_magnitude)
{
	struct drv8235_telemetry telemetry;

	/* A previous reverse command does not turn IMTR into a signed sensor. */
	zassert_ok(drv8235_set_output_uv(fixture->dev, -1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_ok(drv8235_brake(fixture->dev));
	drv8235_emul_set_status(fixture->emul, 0U, 0x60U, 0U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_equal(telemetry.bridge_mode, DRV8235_BRIDGE_BRAKE);
	zassert_true(telemetry.output_enabled);
	zassert_true(telemetry.current_valid);
	zassert_equal(telemetry.current_ua, 1850000U);
	zassert_equal(telemetry.raw_current, 0x60U);
	zassert_equal(telemetry.voltage_uv, 0);
}

ZTEST_F(drv8235_hardware, test_coast_preserves_raw_but_does_not_claim_zero_current)
{
	struct drv8235_telemetry telemetry;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	zassert_ok(drv8235_coast(fixture->dev));
	drv8235_emul_set_status(fixture->emul, 0U, 0x60U, 0U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_true(telemetry.output_enabled);
	zassert_equal(telemetry.bridge_mode, DRV8235_BRIDGE_COAST);
	zassert_false(telemetry.current_valid);
	zassert_equal(telemetry.raw_current, 0x60U);
	zassert_equal(telemetry.current_ua, 0U, "placeholder must be qualified by current_valid");
}

ZTEST_F(drv8235_hardware, test_disabled_output_invalidates_current_in_every_mode)
{
	struct drv8235_telemetry telemetry;

	for (uint8_t mode = DRV8235_BRIDGE_COAST; mode <= DRV8235_BRIDGE_BRAKE; ++mode) {
		/* Mode bits alone do not establish that the low-side sense path is active. */
		drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG4, 0x1CU | mode);
		drv8235_emul_set_status(fixture->emul, 30U, 0x60U, 0x20U);
		zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
		zassert_false(telemetry.output_enabled);
		zassert_false(telemetry.current_valid);
		zassert_equal(telemetry.raw_current, 0x60U);
		zassert_equal(telemetry.current_ua, 0U);
		zassert_equal(telemetry.duty_permille, 508U);
	}
}

ZTEST_F(drv8235_hardware, test_drive_current_magnitude_is_independent_of_command_sign)
{
	struct drv8235_telemetry telemetry;

	for (int sign = -1; sign <= 1; sign += 2) {
		zassert_ok(drv8235_set_output_uv(fixture->dev, sign * 1000000));
		zassert_ok(drv8235_set_enabled(fixture->dev, true));
		drv8235_emul_set_status(fixture->emul, 30U, 0x60U, 0x20U);
		zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
		zassert_true(telemetry.current_valid);
		zassert_equal(telemetry.current_ua, 1850000U);
		zassert_equal(telemetry.voltage_uv, sign * 5000000);
	}
}

ZTEST_F(drv8235_hardware, test_zero_imtr_in_enabled_brake_is_valid_zero)
{
	struct drv8235_telemetry telemetry;

	zassert_ok(drv8235_brake(fixture->dev));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_status(fixture->emul, 0U, 0U, 0U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_true(telemetry.current_valid);
	zassert_equal(telemetry.current_ua, 0U);
}

ZTEST_F(drv8235_hardware, test_fault_or_reset_invalidates_current_even_with_en_out_set)
{
	static const uint8_t statuses[] = {0x92U, 0x06U, 0x00U};
	struct drv8235_telemetry telemetry;

	for (size_t i = 0; i < ARRAY_SIZE(statuses); ++i) {
		zassert_ok(drv8235_reconfigure(fixture->dev));
		zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
		zassert_ok(drv8235_set_enabled(fixture->dev, true));
		drv8235_emul_set_status(fixture->emul, 30U, 0x60U, 0x20U);
		drv8235_emul_set_faults(fixture->emul, statuses[i]);
		zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
		zassert_true(telemetry.output_enabled);
		zassert_false(telemetry.current_valid);
		zassert_equal(telemetry.raw_current, 0x60U);
		zassert_equal(telemetry.voltage_uv, 5000000);
		zassert_equal(telemetry.duty_permille, 508U);
	}
}

ZTEST_F(drv8235_hardware, test_pin_control_mode_does_not_use_i2c_bits_as_sense_state)
{
	struct drv8235_telemetry telemetry;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 1000000));
	zassert_ok(drv8235_set_enabled(fixture->dev, true));
	drv8235_emul_set_status(fixture->emul, 30U, 0x60U, 0x20U);
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG4, 0x1AU);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_false(telemetry.current_valid);
	zassert_equal(telemetry.raw_current, 0x60U);
}

ZTEST_F(drv8235_hardware, test_unconfigured_state_does_not_publish_valid_current)
{
	struct drv8235_telemetry telemetry;

	drv8235_emul_fail_next_write(fixture->emul, DRV8235_REG_RC_CTRL3, -ETIMEDOUT);
	zassert_equal(drv8235_reconfigure(fixture->dev), -ETIMEDOUT);
	/* Mimic an external write, without a successful driver reconfiguration. */
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG0, 0xC0U);
	drv8235_emul_set_reg(fixture->emul, DRV8235_REG_CONFIG4, 0x1FU);
	drv8235_emul_set_status(fixture->emul, 0U, 0x60U, 0U);
	zassert_ok(drv8235_get_telemetry(fixture->dev, &telemetry));
	zassert_true(telemetry.output_enabled);
	zassert_false(telemetry.current_valid);
}

ZTEST_F(drv8235_hardware, test_supply_off_blocks_commands_without_enabling_rail)
{
	zassert_ok(regulator_disable(fixture->supply));
	zassert_equal(drv8235_reconfigure(fixture->dev), -EACCES);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EACCES);
	zassert_equal(drv8235_set_output_uv(fixture->dev, 1000000), -EACCES);
	zassert_equal(drv8235_emul_write_count(fixture->emul), 0U);
	zassert_false(regulator_is_enabled(fixture->supply));
	zassert_ok(regulator_enable(fixture->supply));
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EAGAIN);
	zassert_ok(drv8235_reconfigure(fixture->dev));
}

ZTEST_SUITE(drv8235_hardware, NULL, drv8235_hardware_setup, drv8235_hardware_before, NULL, NULL);
