/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/drv8235.h"

static const struct {
	const char *name;
	const struct device *dev;
} axes[] = {
	{"xm", DEVICE_DT_GET(DT_NODELABEL(torque_xm))},
	{"ym", DEVICE_DT_GET(DT_NODELABEL(torque_ym))},
	{"zm", DEVICE_DT_GET(DT_NODELABEL(torque_zm))},
	{"xp", DEVICE_DT_GET(DT_NODELABEL(torque_xp))},
	{"yp", DEVICE_DT_GET(DT_NODELABEL(torque_yp))},
	{"zp", DEVICE_DT_GET(DT_NODELABEL(torque_zp))},
};

static const struct device *const supply = DEVICE_DT_GET(DT_NODELABEL(coil_power));
static bool armed;

static int find_axis(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		if (strcmp(name, axes[i].name) == 0) {
			return (int)i;
		}
	}
	return -EINVAL;
}

static int disarm(void)
{
	int ret = 0;

	armed = false;
	if (regulator_is_enabled(supply)) {
		for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
			int next = device_is_ready(axes[i].dev) ?
				drv8235_coast(axes[i].dev) : -ENODEV;

			if (ret == 0) {
				ret = next;
			}
		}
		int next = regulator_disable(supply);

		if (ret == 0) {
			ret = next;
		}
	}
	return ret;
}

static int stop_axis(const struct shell *sh, int axis)
{
	int ret = drv8235_coast(axes[axis].dev);

	if (ret != 0) {
		(void)disarm();
		shell_error(sh, "coast failed (%d); COIL_EN %s", ret,
			    regulator_is_enabled(supply) ?
			    "STILL ON: REMOVE EXTERNAL POWER" : "OFF");
	}
	return ret;
}

struct status_snapshot {
	struct drv8235_faults faults;
	enum drv8235_bridge_mode mode;
	float volts, amps;
};

/* No console I/O here: pulse uses this while the coil is driven. */
static int read_status(int axis, struct status_snapshot *snapshot)
{
	const struct device *dev = axes[axis].dev;
	int ret = drv8235_read_voltage_current(dev, &snapshot->volts, &snapshot->amps);

	if (ret == 0) {
		ret = drv8235_get_bridge_control(dev, &snapshot->mode);
	}
	if (ret == 0) {
		ret = drv8235_get_faults(dev, &snapshot->faults);
	}
	return ret;
}

static void print_snapshot(const struct shell *sh, int axis,
			   const struct status_snapshot *snapshot)
{
	const char *const names[] = {"coast", "reverse", "forward", "brake"};

	shell_print(sh, "%s: mode=%s voltage=%dmV current=%dmA fault=0x%02x",
		    axes[axis].name, names[snapshot->mode], (int)(snapshot->volts * 1000),
		    (int)(snapshot->amps * 1000), snapshot->faults.raw);
	if (snapshot->mode == DRV8235_BRIDGE_COAST) {
		shell_print(sh, "coast current may be stale/unsensed; current calibration pending");
	}
}

static int print_status(const struct shell *sh, int axis)
{
	struct status_snapshot snapshot;
	int ret = read_status(axis, &snapshot);

	if (ret != 0) {
		shell_error(sh, "%s: status failed (%d)", axes[axis].name, ret);
	} else {
		print_snapshot(sh, axis, &snapshot);
	}
	return ret;
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "COIL_EN: %s", regulator_is_enabled(supply) ? "ON" : "OFF");
	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		shell_print(sh, "%s: %s", axes[i].name,
			    device_is_ready(axes[i].dev) ? "driver ready (chip checked on arm)" :
			    "NOT READY");
	}
	return 0;
}

static int cmd_arm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "I_UNDERSTAND") != 0) {
		shell_error(sh, "use: drv8235 arm I_UNDERSTAND");
		return -EPERM;
	}
	if (armed) {
		shell_print(sh, "already armed");
		return 0;
	}
	if (regulator_is_enabled(supply)) {
		shell_error(sh, "rail is already ON; disarm first");
		return -EBUSY;
	}
	for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
		if (!device_is_ready(axes[i].dev)) {
			shell_error(sh, "%s: NOT READY; rail remains off", axes[i].name);
			return -ENODEV;
		}
	}
	int ret = regulator_enable(supply);

	for (size_t i = 0; ret == 0 && i < ARRAY_SIZE(axes); ++i) {
		ret = drv8235_init(axes[i].dev);
	}
	if (ret != 0) {
		(void)disarm();
		shell_error(sh, "arm failed (%d); COIL_EN %s", ret,
			    regulator_is_enabled(supply) ?
			    "STILL ON: REMOVE EXTERNAL POWER" : "OFF");
		return ret;
	}
	armed = true;
	shell_print(sh, "COIL_EN ARMED; bridges enabled at target zero in coast");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	if (!armed) {
		shell_error(sh, "arm first");
		return -EACCES;
	}
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		int ret = 0;

		for (size_t i = 0; i < ARRAY_SIZE(axes); ++i) {
			int next = print_status(sh, (int)i);

			if (ret == 0) {
				ret = next;
			}
		}
		return ret;
	}
	int axis = find_axis(argv[1]);

	return axis < 0 ? axis : print_status(sh, axis);
}

static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (!armed) {
		shell_error(sh, "arm first");
		return -EPERM;
	}
	int axis = find_axis(argv[1]);
	char *v_end, *t_end;
	long mv = strtol(argv[2], &v_end, 10);
	long ms = strtol(argv[3], &t_end, 10);

	if (axis < 0 || *argv[2] == '\0' || *v_end != '\0' ||
	    *argv[3] == '\0' || *t_end != '\0' ||
	    mv < -6000 || mv > 6000 || ms < 1 || ms > 5000) {
		shell_error(sh, "pulse <axis> <-6000..6000 mV> <1..5000 ms>; zero brakes");
		return -EINVAL;
	}
	int ret = drv8235_set_throttle_volts(axes[axis].dev, mv / 1000.0f);
	int64_t deadline = k_uptime_get() + ms;
	struct status_snapshot snapshot;
	bool captured = false;
	bool fault_seen = false;
	uint8_t pulse_fault = 0;

	while (ret == 0 && k_uptime_get() < deadline) {
		k_msleep((int32_t)MIN(50LL, MAX(0LL, deadline - k_uptime_get())));
		struct drv8235_faults faults;

		ret = drv8235_get_faults(axes[axis].dev, &faults);
		if (ret == 0 && ((faults.raw & 0xBC) || faults.power_on_reset)) {
			pulse_fault = faults.raw;
			fault_seen = true;
			ret = -EIO;
		}
		if (ret == 0 && !captured) {
			/* Save the active reading, but never print until the coil is stopped. */
			ret = drv8235_read_voltage_current(axes[axis].dev,
							 &snapshot.volts, &snapshot.amps);
			if (ret == 0) {
				ret = drv8235_get_bridge_control(axes[axis].dev, &snapshot.mode);
			}
			snapshot.faults = faults;
			captured = ret == 0;
		}
	}
	int stop = stop_axis(sh, axis);

	/* USB/shell writes can wait indefinitely if the host disconnects. */
	if (captured) {
		shell_print(sh, "Captured during pulse:");
		print_snapshot(sh, axis, &snapshot);
	}
	if (fault_seen) {
		shell_error(sh, "pulse aborted; last fault=0x%02x", pulse_fault);
	}
	if (ret == 0) {
		ret = stop;
	}
	if (ret == 0) {
		shell_print(sh, "%s pulse complete; coasting", axes[axis].name);
	} else {
		shell_error(sh, "%s pulse failed (%d)", axes[axis].name, ret);
	}
	return ret;
}

static int cmd_coast(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	int axis = find_axis(argv[1]);

	return axis < 0 ? axis : stop_axis(sh, axis);
}

static int cmd_disarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	int ret = disarm();

	shell_print(sh, "disarm result=%d; COIL_EN %s", ret,
		    regulator_is_enabled(supply) ? "STILL ON: REMOVE EXTERNAL POWER" : "OFF");
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	commands,
	SHELL_CMD_ARG(list, NULL, "List axes and rail state", cmd_list, 1, 0),
	SHELL_CMD_ARG(arm, NULL, "arm I_UNDERSTAND", cmd_arm, 2, 0),
	SHELL_CMD_ARG(status, NULL, "status [all|axis]; reported faults are cleared", cmd_status, 1, 1),
	SHELL_CMD_ARG(pulse, NULL, "pulse <axis> <mV> <ms>; zero brakes", cmd_pulse, 4, 0),
	SHELL_CMD_ARG(coast, NULL, "coast <axis>", cmd_coast, 2, 0),
	SHELL_CMD_ARG(disarm, NULL, "Coast all axes and remove shared power", cmd_disarm, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(drv8235, &commands, "Argus DRV8235 diagnostic", NULL);

int main(void)
{
	if (!device_is_ready(supply)) {
		printk("FATAL: COIL_EN regulator not ready\n");
		return -ENODEV;
	}
	int ret = disarm();

	printk("\nArgus DRV8235 diagnostic\n");
	if (regulator_is_enabled(supply)) {
		printk("UNSAFE START: REMOVE EXTERNAL POWER\n");
		return -EIO;
	}
	printk("SAFE START: COIL_EN is OFF\n");
	printk("Run 'drv8235 list', then 'drv8235 arm I_UNDERSTAND'\n");
	if (ret != 0) {
		printk("WARNING: device initialization/coast failed (%d)\n", ret);
	}
	return 0;
}
