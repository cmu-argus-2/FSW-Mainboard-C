/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "argus/drivers/drv8235.h"

ZTEST(drv8235_common, test_voltage_command_zero_is_coast)
{
	uint8_t code = UINT8_MAX;
	enum drv8235_bridge_mode mode = DRV8235_BRIDGE_BRAKE;

	zassert_ok(drv8235_voltage_to_code(0, 6000000U, &code, &mode));
	zassert_equal(code, 0U);
	zassert_equal(mode, DRV8235_BRIDGE_COAST);
}

ZTEST(drv8235_common, test_voltage_command_converts_sign_and_magnitude)
{
	uint8_t code;
	enum drv8235_bridge_mode mode;

	zassert_ok(drv8235_voltage_to_code(6000000, 6000000U, &code, &mode));
	zassert_equal(code, 36U);
	zassert_equal(mode, DRV8235_BRIDGE_FORWARD);
	zassert_ok(drv8235_voltage_to_code(-6000000, 6000000U, &code, &mode));
	zassert_equal(code, 36U);
	zassert_equal(mode, DRV8235_BRIDGE_REVERSE);
}

ZTEST(drv8235_common, test_voltage_command_enforces_safety_limit)
{
	uint8_t code;
	enum drv8235_bridge_mode mode;

	zassert_equal(drv8235_voltage_to_code(6000001, 6000000U, &code, &mode), -ERANGE);
	zassert_equal(drv8235_voltage_to_code(INT32_MIN, 6000000U, &code, &mode), -ERANGE);
}

ZTEST(drv8235_common, test_voltage_status_conversion)
{
	int32_t voltage_uv;

	zassert_ok(drv8235_voltage_from_code(30U, DRV8235_BRIDGE_FORWARD, &voltage_uv));
	zassert_equal(voltage_uv, 5020000);
	zassert_ok(drv8235_voltage_from_code(30U, DRV8235_BRIDGE_REVERSE, &voltage_uv));
	zassert_equal(voltage_uv, -5020000);
	zassert_ok(drv8235_voltage_from_code(30U, DRV8235_BRIDGE_COAST, &voltage_uv));
	zassert_equal(voltage_uv, 0);
}

ZTEST(drv8235_common, test_current_and_duty_conversion)
{
	int32_t current_ua;
	uint16_t duty_permille;

	zassert_ok(drv8235_current_from_code(0x60U, 3700000U, DRV8235_BRIDGE_FORWARD, &current_ua));
	zassert_equal(current_ua, 1850000);
	zassert_ok(drv8235_current_from_code(0x60U, 3700000U, DRV8235_BRIDGE_REVERSE, &current_ua));
	zassert_equal(current_ua, -1850000);
	zassert_equal(
		drv8235_current_from_code(0xC1U, 3700000U, DRV8235_BRIDGE_FORWARD, &current_ua),
		-ERANGE);

	zassert_ok(drv8235_duty_from_code(0x20U, &duty_permille));
	zassert_equal(duty_permille, 508U);
	zassert_ok(drv8235_duty_from_code(0x3FU, &duty_permille));
	zassert_equal(duty_permille, 1000U);
}

ZTEST(drv8235_common, test_fault_decode)
{
	struct drv8235_faults faults;

	zassert_ok(drv8235_decode_faults(0xBEU, &faults));
	zassert_true(faults.any);
	zassert_true(faults.stall);
	zassert_true(faults.overcurrent);
	zassert_true(faults.overvoltage);
	zassert_true(faults.thermal_shutdown);
	zassert_true(faults.undervoltage_lockout);
	zassert_equal(faults.raw, 0xBEU);
}

ZTEST(drv8235_common, test_inv_r_for_argus_coil)
{
	uint8_t scale;
	uint8_t inv_r;

	zassert_ok(drv8235_calculate_inv_r(25000U, &scale, &inv_r));
	zassert_equal(scale, 2U);
	zassert_equal(inv_r, 41U);
}

ZTEST(drv8235_common, test_common_helpers_reject_null)
{
	uint8_t code;
	uint8_t inv_r;
	int32_t value;

	zassert_equal(drv8235_voltage_to_code(0, 6000000U, NULL, NULL), -EINVAL);
	zassert_equal(drv8235_voltage_from_code(0U, DRV8235_BRIDGE_COAST, NULL), -EINVAL);
	zassert_equal(drv8235_current_from_code(0U, 3700000U, DRV8235_BRIDGE_COAST, NULL), -EINVAL);
	zassert_equal(drv8235_decode_faults(0U, NULL), -EINVAL);
	zassert_equal(drv8235_calculate_inv_r(25000U, NULL, &inv_r), -EINVAL);
	zassert_equal(drv8235_calculate_inv_r(0U, &code, &inv_r), -EINVAL);
	ARG_UNUSED(value);
}

ZTEST_SUITE(drv8235_common, NULL, NULL, NULL, NULL, NULL);
