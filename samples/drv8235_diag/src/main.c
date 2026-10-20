/*
 * Safe interactive diagnostic for the six Argus DRV8235 magnetorquer drivers.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/drv8235.h"

struct torque_axis {
	const char *name;
	const struct device *dev;
};

static const struct torque_axis axes[] = {
	{"xm", DEVICE_DT_GET(DT_NODELABEL(torque_xm))},
	{"ym", DEVICE_DT_GET(DT_NODELABEL(torque_ym))},
	{"zm", DEVICE_DT_GET(DT_NODELABEL(torque_zm))},
	{"xp", DEVICE_DT_GET(DT_NODELABEL(torque_xp))},
	{"yp", DEVICE_DT_GET(DT_NODELABEL(torque_yp))},
	{"zp", DEVICE_DT_GET(DT_NODELABEL(torque_zp))},
};

static const struct device *const coil_power = DEVICE_DT_GET(DT_NODELABEL(coil_power));

static bool rail_armed;

static const char *bridge_mode_name(enum drv8235_bridge_mode mode)
{
	switch (mode) {
	case DRV8235_BRIDGE_COAST:
		return "coast";
	case DRV8235_BRIDGE_REVERSE:
		return "reverse";
	case DRV8235_BRIDGE_FORWARD:
		return "forward";
	case DRV8235_BRIDGE_BRAKE:
		return "brake";
	default:
		return "invalid";
	}
}

static const struct torque_axis *find_axis(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		if (strcmp(name, axes[i].name) == 0) {
			return &axes[i];
		}
	}

	return NULL;
}

static int make_axis_safe(const struct torque_axis *axis)
{
	if (!device_is_ready(axis->dev)) {
		return -ENODEV;
	}

	/* Disabling also writes target=0 and selects coast in the driver. */
	return drv8235_set_enabled(axis->dev, false);
}

static int disarm_all(void)
{
	int first_error = 0;

	rail_armed = false;
	if (regulator_is_enabled(coil_power)) {
		for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
			const int result = make_axis_safe(&axes[i]);

			if (first_error == 0 && result != 0) {
				first_error = result;
			}
		}
	}

	const int power_result = regulator_disable(coil_power);

	if (first_error == 0) {
		first_error = power_result;
	}

	return first_error;
}

static void print_axis_status(const struct shell *sh, const struct torque_axis *axis)
{
	struct drv8235_telemetry telemetry;
	struct drv8235_faults faults;
	struct drv8235_limits limits;
	int result;

	if (!device_is_ready(axis->dev)) {
		shell_error(sh, "%s: NOT READY (check face board and I2C)", axis->name);
		return;
	}

	result = drv8235_get_limits(axis->dev, &limits);
	if (result != 0) {
		shell_error(sh, "%s: limits read failed (%d)", axis->name, result);
		return;
	}

	result = drv8235_get_faults(axis->dev, &faults);
	if (result != 0) {
		shell_error(sh, "%s: fault read failed (%d)", axis->name, result);
		return;
	}

	result = drv8235_get_telemetry(axis->dev, &telemetry);
	if (result != 0) {
		shell_error(sh, "%s: telemetry read failed (%d)", axis->name, result);
		return;
	}

	shell_print(sh,
		    "%s: ready enabled=%s mode=%s voltage=%" PRId32 "mV current=%" PRId32
		    "mA duty=%u.%u%% fault=0x%02x "
		    "limit=%" PRIu32 "mV",
		    axis->name, telemetry.output_enabled ? "yes" : "no",
		    bridge_mode_name(telemetry.bridge_mode), telemetry.voltage_uv / 1000,
		    telemetry.current_ua / 1000, telemetry.duty_permille / 10,
		    telemetry.duty_permille % 10, faults.raw, limits.max_output_uv / 1000);
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "COIL_EN: %s", rail_armed ? "ARMED" : "OFF");
	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		shell_print(sh, "%s: %s", axes[i].name,
			    device_is_ready(axes[i].dev) ? "ready" : "NOT READY");
	}

	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	if (!regulator_is_enabled(coil_power)) {
		shell_warn(sh, "COIL_EN is OFF; arm the rail before reading DRV8235 status");
		return -EACCES;
	}

	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		shell_print(sh, "COIL_EN: %s", rail_armed ? "ARMED" : "OFF");
		for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
			print_axis_status(sh, &axes[i]);
		}
		return 0;
	}

	const struct torque_axis *axis = find_axis(argv[1]);

	if (axis == NULL) {
		shell_error(sh, "unknown axis '%s' (use xm ym zm xp yp zp or all)", argv[1]);
		return -EINVAL;
	}

	print_axis_status(sh, axis);
	return 0;
}

static int cmd_arm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2 || strcmp(argv[1], "I_UNDERSTAND") != 0) {
		shell_error(sh, "refused; disconnect sensitive hardware, then use: "
				"drv8235 arm I_UNDERSTAND");
		return -EPERM;
	}

	if (rail_armed) {
		shell_print(sh, "COIL_EN is already armed");
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		if (!device_is_ready(axes[i].dev)) {
			shell_error(sh, "%s is not ready; rail remains off", axes[i].name);
			return -ENODEV;
		}
	}

	int result = regulator_enable(coil_power);

	if (result != 0) {
		shell_error(sh, "could not enable COIL_EN (%d)", result);
		return result;
	}

	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		struct drv8235_faults faults;

		result = drv8235_reconfigure(axes[i].dev);
		if (result == 0) {
			result = drv8235_get_faults(axes[i].dev, &faults);
		}
		if (result != 0 || faults.any) {
			shell_error(sh, "%s is unsafe after power-up (result=%d fault=0x%02x)",
				    axes[i].name, result, result == 0 ? faults.raw : 0U);
			(void)disarm_all();
			return result != 0 ? result : -EIO;
		}
	}

	rail_armed = true;
	shell_warn(sh, "COIL_EN ARMED; every bridge remains disabled and coasting");
	return 0;
}

static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
	if (!rail_armed) {
		shell_error(sh, "COIL_EN is off; arm first");
		return -EPERM;
	}

	const struct torque_axis *axis = find_axis(argv[1]);

	if (axis == NULL) {
		shell_error(sh, "unknown axis '%s'", argv[1]);
		return -EINVAL;
	}

	char *voltage_end;
	char *duration_end;
	const long millivolts = strtol(argv[2], &voltage_end, 10);
	const long duration_ms = strtol(argv[3], &duration_end, 10);

	if (*argv[2] == '\0' || *voltage_end != '\0' || millivolts < INT32_MIN / 1000L ||
	    millivolts > INT32_MAX / 1000L) {
		shell_error(sh, "invalid millivolt value '%s'", argv[2]);
		return -EINVAL;
	}
	if (*argv[3] == '\0' || *duration_end != '\0' || duration_ms < 1 || duration_ms > 5000) {
		shell_error(sh, "duration must be 1..5000 ms");
		return -EINVAL;
	}
	if (millivolts == 0) {
		shell_error(sh, "pulse voltage must be nonzero");
		return -EINVAL;
	}

	int result = drv8235_set_output_uv(axis->dev, (int32_t)millivolts * 1000);

	if (result == 0) {
		result = drv8235_set_enabled(axis->dev, true);
	}
	if (result != 0) {
		(void)make_axis_safe(axis);
		shell_error(sh, "%s command failed (%d); axis made safe", axis->name, result);
		return result;
	}

	shell_warn(sh, "%s ACTIVE at requested %ld mV for at most %ld ms", axis->name, millivolts,
		   duration_ms);
	long elapsed_ms = 0;
	bool printed_status = false;

	while (elapsed_ms < duration_ms) {
		struct drv8235_faults faults;
		const long slice_ms = MIN(50L, duration_ms - elapsed_ms);

		k_msleep(slice_ms);
		elapsed_ms += slice_ms;
		result = drv8235_get_faults(axis->dev, &faults);
		if (result != 0 || faults.any) {
			(void)make_axis_safe(axis);
			shell_error(sh, "%s pulse aborted at %ld ms (result=%d fault=0x%02x)",
				    axis->name, elapsed_ms, result, result == 0 ? faults.raw : 0U);
			return result != 0 ? result : -EIO;
		}
		if (!printed_status) {
			print_axis_status(sh, axis);
			printed_status = true;
		}
	}

	result = make_axis_safe(axis);
	if (result != 0) {
		shell_error(sh, "%s pulse ended but safe-state write failed (%d)", axis->name,
			    result);
		(void)disarm_all();
		return result;
	}

	shell_print(sh, "%s pulse complete; bridge disabled and coasting", axis->name);
	return 0;
}

static int cmd_coast(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct torque_axis *axis = find_axis(argv[1]);

	if (axis == NULL) {
		shell_error(sh, "unknown axis '%s'", argv[1]);
		return -EINVAL;
	}

	const int result = make_axis_safe(axis);

	if (result == 0) {
		shell_print(sh, "%s disabled and coasting", axis->name);
	}
	return result;
}

static int cmd_disarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const int result = disarm_all();

	if (result == 0) {
		shell_print(sh, "COIL_EN off; all bridges disabled and coasting");
	} else {
		shell_error(sh, "COIL_EN is off, but an I2C safe-state write failed (%d)", result);
	}
	return result;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_drv8235,
	SHELL_CMD_ARG(list, NULL, "List the six axes and COIL_EN state", cmd_list, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Read status [all|xm|ym|zm|xp|yp|zp]", cmd_status, 1, 1),
	SHELL_CMD_ARG(arm, NULL, "Arm rail: arm I_UNDERSTAND", cmd_arm, 2, 0),
	SHELL_CMD_ARG(pulse, NULL, "Timed drive: pulse <axis> <millivolts> <1..5000 ms>", cmd_pulse,
		      4, 0),
	SHELL_CMD_ARG(coast, NULL, "Disable one axis: coast <axis>", cmd_coast, 2, 0),
	SHELL_CMD_ARG(disarm, NULL, "Turn rail off and make every axis safe", cmd_disarm, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(drv8235, &sub_drv8235, "Argus DRV8235 diagnostics", NULL);

int main(void)
{
	if (!device_is_ready(coil_power)) {
		printk("FATAL: COIL_EN regulator is not ready\n");
		return -ENODEV;
	}

	const int result = disarm_all();
	printk("\nArgus DRV8235 diagnostic\n");
	printk("SAFE START: COIL_EN is OFF; all responding bridges are disabled\n");
	printk("Run 'drv8235 list', then arm before reading live status\n");
	printk("Run 'drv8235 help' for commands\n");
	if (result != 0) {
		printk("WARNING: one or more DRV8235 devices did not respond (%d)\n", result);
	}

	return 0;
}
