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
	zassert_equal(code, 35U);
	zassert_equal(mode, DRV8235_BRIDGE_FORWARD);
	zassert_ok(drv8235_voltage_to_code(-6000000, 6000000U, &code, &mode));
	zassert_equal(code, 35U);
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
	zassert_equal(voltage_uv, 5000000);
	zassert_ok(drv8235_voltage_from_code(30U, DRV8235_BRIDGE_REVERSE, &voltage_uv));
	zassert_equal(voltage_uv, -5000000);
	zassert_ok(drv8235_voltage_from_code(30U, DRV8235_BRIDGE_COAST, &voltage_uv));
	zassert_equal(voltage_uv, 0);
}

ZTEST(drv8235_common, test_current_and_duty_conversion)
{
	uint32_t current_ua;
	uint16_t duty_permille;

	zassert_ok(drv8235_current_from_code(0x60U, 3700000U, &current_ua));
	zassert_equal(current_ua, 1850000U);
	zassert_ok(drv8235_current_from_code(0U, 3700000U, &current_ua));
	zassert_equal(current_ua, 0U);
	zassert_ok(drv8235_current_from_code(0xC0U, 3700000U, &current_ua));
	zassert_equal(current_ua, 3700000U);
	zassert_equal(drv8235_current_from_code(0xC1U, 3700000U, &current_ua), -ERANGE);
	zassert_equal(drv8235_current_from_code(0x60U, 0U, &current_ua), -EINVAL);
	zassert_equal(drv8235_current_from_code(0x60U, UINT32_MAX, &current_ua), -EINVAL);

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
	zassert_false(faults.power_on_reset);
	zassert_equal(faults.raw, 0xBEU);
}

ZTEST(drv8235_common, test_npor_is_active_low_and_normal_status_is_not_a_fault)
{
	struct drv8235_faults faults;

	zassert_ok(drv8235_decode_faults(0x00U, &faults));
	zassert_true(faults.power_on_reset);
	zassert_true(faults.any);
	zassert_ok(drv8235_decode_faults(0x02U, &faults));
	zassert_false(faults.power_on_reset);
	zassert_false(faults.any);
	zassert_ok(drv8235_decode_faults(0x06U, &faults));
	zassert_true(faults.thermal_shutdown);
	zassert_true(faults.any, "latched TSD must not be hidden by FAULT=0");
}

ZTEST(drv8235_common, test_rounded_commands_never_exceed_encoded_limit)
{
	static const uint32_t limits[] = {1U, 100000U, 1000000U, 6000000U, 42670000U};
	uint8_t code;
	enum drv8235_bridge_mode mode;

	for (size_t i = 0; i < ARRAY_SIZE(limits); ++i) {
		for (uint32_t step = 0; step <= 255U; ++step) {
			const int32_t command = (int32_t)((uint64_t)limits[i] * step / 255U);

			for (int sign = -1; sign <= 1; sign += 2) {
				zassert_ok(drv8235_voltage_to_code(sign * command, limits[i],
								 &code, &mode));
				zassert_true((uint64_t)code * DRV8235_TARGET_FULL_SCALE_UV <=
					     (uint64_t)limits[i] * DRV8235_TARGET_FULL_SCALE_CODE);
				if (code == 0U) {
					zassert_equal(mode, DRV8235_BRIDGE_COAST);
				}
			}
		}
	}
	/* Ordinary nearest-code rounding is retained away from the hard limit. */
	zassert_ok(drv8235_voltage_to_code(5000000, 6000000U, &code, &mode));
	zassert_equal(code, 30U);
}

ZTEST(drv8235_common, test_vmtr_uses_its_own_reference_scale)
{
	int32_t voltage_uv;

	zassert_ok(drv8235_voltage_from_code(0xE4U, DRV8235_BRIDGE_FORWARD, &voltage_uv));
	zassert_equal(voltage_uv, 38000000);
	zassert_ok(drv8235_voltage_from_code(0xE4U, DRV8235_BRIDGE_REVERSE, &voltage_uv));
	zassert_equal(voltage_uv, -38000000);
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
	zassert_equal(drv8235_current_from_code(0U, 3700000U, NULL), -EINVAL);
	zassert_equal(drv8235_decode_faults(0U, NULL), -EINVAL);
	zassert_equal(drv8235_calculate_inv_r(25000U, NULL, &inv_r), -EINVAL);
	zassert_equal(drv8235_calculate_inv_r(0U, &code, &inv_r), -EINVAL);
	ARG_UNUSED(value);
}

ZTEST_SUITE(drv8235_common, NULL, NULL, NULL, NULL, NULL);
