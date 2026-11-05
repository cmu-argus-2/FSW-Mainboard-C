/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static int monitor_unavailable(void *context, enum eps_monitor monitor, struct eps_vi *reading)
{
	ARG_UNUSED(context);
	ARG_UNUSED(monitor);
	ARG_UNUSED(reading);
	return -ENODEV;
}

static int coil_unavailable(void *context, enum eps_axis axis, struct eps_vi *reading)
{
	ARG_UNUSED(context);
	ARG_UNUSED(axis);
	ARG_UNUSED(reading);
	return -ENODEV;
}

static int fuel_unavailable(void *context, struct eps_fuel *reading)
{
	ARG_UNUSED(context);
	ARG_UNUSED(reading);
	return -ENODEV;
}

static int cpu_unavailable(void *context, int32_t *temperature)
{
	ARG_UNUSED(context);
	ARG_UNUSED(temperature);
	return -ENODEV;
}

static int heaters_unavailable(void *context, bool first, bool second)
{
	ARG_UNUSED(context);
	ARG_UNUSED(first);
	ARG_UNUSED(second);
	return -ENODEV;
}

static void console_publish(void *context, bool full, const int32_t *fields, size_t count)
{
	char line[EPS_RECORD_BUFFER_SIZE];

	ARG_UNUSED(context);
	if (eps_format_record(line, sizeof(line), full, fields, count) >= 0) {
		printk("%s\n", line);
	}
}

const struct eps_io *eps_console_only_io(void)
{
	static const struct eps_io io = {
		.read_monitor = monitor_unavailable,
		.read_coil = coil_unavailable,
		.read_fuel = fuel_unavailable,
		.read_cpu_temp = cpu_unavailable,
		.set_heaters = heaters_unavailable,
		.publish = console_publish,
	};

	return &io;
}

void eps_run(const struct eps_io *io)
{
	struct eps_state state;
	int64_t next_run = k_uptime_get();

	eps_init(&state);
	for (;;) {
		(void)eps_step(&state, io, (int32_t)(k_uptime_get() / 1000));
		next_run += 1000;
		/* Absolute deadlines avoid drift. Missed ticks do not cause a burst. */
		int64_t now = k_uptime_get();

		if (next_run <= now) {
			next_run += ((now - next_run) / 1000 + 1) * 1000;
		}
		k_sleep(K_TIMEOUT_ABS_MS(next_run));
	}
}
