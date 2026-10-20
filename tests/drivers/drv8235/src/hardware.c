/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/ztest.h>

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
};

static void *drv8235_hardware_setup(void)
{
	static struct drv8235_hardware_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(drv8235_test)),
		.emul = EMUL_DT_GET(DT_NODELABEL(drv8235_test)),
	};

	zassert_true(device_is_ready(fixture.dev));
	return &fixture;
}

static void drv8235_hardware_before(void *fixture_ptr)
{
	struct drv8235_hardware_fixture *fixture = fixture_ptr;

	drv8235_emul_reset(fixture->emul);
}

ZTEST_F(drv8235_hardware, test_init_programs_safe_voltage_mode)
{
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0), 0x60U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4), 0x3CU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_REG_CTRL0), 0x3FU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG3), 0x73U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_RC_CTRL2), 0xB3U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_RC_CTRL3), 41U);
}

ZTEST_F(drv8235_hardware, test_reconfigure_after_power_cycle)
{
	drv8235_emul_power_cycle(fixture->emul);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0xFFU);

	zassert_ok(drv8235_reconfigure(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG0), 0x60U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 0U);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_CONFIG4), 0x3CU);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_REG_CTRL0), 0x3FU);
}

ZTEST_F(drv8235_hardware, test_voltage_command_does_not_arm_output)
{
	bool enabled = true;

	zassert_ok(drv8235_set_output_uv(fixture->dev, 6000000));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_TARGET), 36U);
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

	drv8235_emul_set_faults(fixture->emul, 0x90U);
	zassert_equal(drv8235_set_enabled(fixture->dev, true), -EIO);
	zassert_ok(drv8235_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);

	drv8235_emul_set_faults(fixture->emul, 0U);
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
	zassert_equal(telemetry.voltage_uv, -5020000);
	zassert_equal(telemetry.current_ua, -1850000);
	zassert_equal(telemetry.duty_permille, 508U);
	zassert_equal(telemetry.bridge_mode, DRV8235_BRIDGE_REVERSE);
	zassert_true(telemetry.output_enabled);
}

ZTEST_F(drv8235_hardware, test_fault_read_does_not_clear_until_requested)
{
	struct drv8235_faults faults;

	drv8235_emul_set_faults(fixture->emul, 0x94U);
	zassert_ok(drv8235_get_faults(fixture->dev, &faults));
	zassert_true(faults.any);
	zassert_true(faults.overcurrent);
	zassert_true(faults.thermal_shutdown);
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0x94U);

	zassert_ok(drv8235_clear_faults(fixture->dev));
	zassert_equal(drv8235_emul_get_reg(fixture->emul, DRV8235_REG_FAULT_STATUS), 0U);
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

ZTEST_SUITE(drv8235_hardware, NULL, drv8235_hardware_setup, drv8235_hardware_before, NULL, NULL);
