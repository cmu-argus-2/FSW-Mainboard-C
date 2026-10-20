/* SPDX-License-Identifier: Apache-2.0 */

#include "argus/drivers/drv8235.h"

#include <errno.h>
#include <limits.h>
#include <stddef.h>

#include <zephyr/sys/util.h>

static uint64_t rounded_divide(uint64_t numerator, uint64_t denominator)
{
	return (numerator + denominator / 2U) / denominator;
}

static bool mode_is_valid(enum drv8235_bridge_mode mode)
{
	return mode >= DRV8235_BRIDGE_COAST && mode <= DRV8235_BRIDGE_BRAKE;
}

static int direction_sign(enum drv8235_bridge_mode mode)
{
	return mode == DRV8235_BRIDGE_REVERSE ? -1 : 1;
}

int drv8235_voltage_to_code(int32_t voltage_uv, uint32_t max_output_uv, uint8_t *code,
			    enum drv8235_bridge_mode *mode)
{
	if (code == NULL || mode == NULL || max_output_uv == 0U ||
	    max_output_uv > DRV8235_TARGET_FULL_SCALE_UV) {
		return -EINVAL;
	}

	const int64_t signed_command = voltage_uv;
	const uint64_t magnitude =
		signed_command < 0 ? (uint64_t)-signed_command : (uint64_t)signed_command;

	if (magnitude > max_output_uv) {
		return -ERANGE;
	}

	if (magnitude == 0U) {
		*code = 0U;
		*mode = DRV8235_BRIDGE_COAST;
		return 0;
	}

	const uint64_t raw = rounded_divide(magnitude * DRV8235_TARGET_FULL_SCALE_CODE,
					    DRV8235_TARGET_FULL_SCALE_UV);

	*code = (uint8_t)raw;
	*mode = signed_command < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD;
	return 0;
}

int drv8235_voltage_from_code(uint8_t code, enum drv8235_bridge_mode mode, int32_t *voltage_uv)
{
	if (voltage_uv == NULL || !mode_is_valid(mode)) {
		return -EINVAL;
	}

	int64_t value = (int64_t)rounded_divide((uint64_t)code * DRV8235_TARGET_FULL_SCALE_UV,
						DRV8235_TARGET_FULL_SCALE_CODE);

	if (mode == DRV8235_BRIDGE_COAST || mode == DRV8235_BRIDGE_BRAKE) {
		value = 0;
	} else {
		value *= direction_sign(mode);
	}

	*voltage_uv = (int32_t)value;
	return 0;
}

int drv8235_current_from_code(uint8_t code, uint32_t full_scale_ua, enum drv8235_bridge_mode mode,
			      int32_t *current_ua)
{
	if (current_ua == NULL || full_scale_ua == 0U || full_scale_ua > INT32_MAX ||
	    !mode_is_valid(mode)) {
		return -EINVAL;
	}
	if (code > DRV8235_IMTR_FULL_SCALE_CODE) {
		return -ERANGE;
	}

	int64_t value = (int64_t)rounded_divide((uint64_t)code * full_scale_ua,
						DRV8235_IMTR_FULL_SCALE_CODE);

	if (mode == DRV8235_BRIDGE_COAST || mode == DRV8235_BRIDGE_BRAKE) {
		value = 0;
	} else {
		value *= direction_sign(mode);
	}

	*current_ua = (int32_t)value;
	return 0;
}

int drv8235_duty_from_code(uint8_t code, uint16_t *duty_permille)
{
	if (duty_permille == NULL) {
		return -EINVAL;
	}
	if (code > DRV8235_DUTY_FULL_SCALE_CODE) {
		return -ERANGE;
	}

	*duty_permille =
		(uint16_t)rounded_divide((uint64_t)code * 1000U, DRV8235_DUTY_FULL_SCALE_CODE);
	return 0;
}

int drv8235_decode_faults(uint8_t raw, struct drv8235_faults *faults)
{
	if (faults == NULL) {
		return -EINVAL;
	}

	faults->raw = raw;
	faults->any = (raw & BIT(7)) != 0U;
	faults->stall = (raw & BIT(5)) != 0U;
	faults->overcurrent = (raw & BIT(4)) != 0U;
	faults->overvoltage = (raw & BIT(3)) != 0U;
	faults->thermal_shutdown = (raw & BIT(2)) != 0U;
	faults->undervoltage_lockout = (raw & BIT(1)) != 0U;
	return 0;
}

int drv8235_calculate_inv_r(uint32_t coil_resistance_milliohm, uint8_t *scale_code, uint8_t *inv_r)
{
	static const uint32_t scales[] = {2U, 64U, 1024U, 8192U};

	if (coil_resistance_milliohm == 0U || scale_code == NULL || inv_r == NULL) {
		return -EINVAL;
	}

	for (int code = (int)ARRAY_SIZE(scales) - 1; code >= 0; --code) {
		const uint64_t value =
			rounded_divide((uint64_t)scales[code] * 1000U, coil_resistance_milliohm);

		if (value >= 1U && value <= UINT8_MAX) {
			*scale_code = (uint8_t)code;
			*inv_r = (uint8_t)value;
			return 0;
		}
	}

	return -ERANGE;
}
