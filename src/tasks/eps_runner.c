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

void eps_console_publish(void *context, bool full, const int32_t *fields, size_t count,
			 uint16_t valid_reads)
{
	char line[EPS_RECORD_BUFFER_SIZE];

	ARG_UNUSED(context);
	if (eps_format_record(line, sizeof(line), full, fields, count) >= 0) {
		printk("%s\n%s,%ld,0x%03x\n", line,
		       full ? "EPS_VALIDITY" : "EPS_WARNING_VALIDITY",
		       (long)fields[0], (unsigned int)valid_reads);
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
		.publish = eps_console_publish,
	};

	return &io;
}

#define EPS_PERIOD_MS 1000
#define EPS_THREAD_PRIORITY K_PRIO_PREEMPT(2)
#define EPS_STACK_SIZE 4096

static struct eps_io thread_io;

static void eps_thread_entry(void *p1, void *p2, void *p3)
{
	struct eps_state state;
	int64_t next_run = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	eps_init(&state);
	for (;;) {
		(void)eps_step(&state, &thread_io, (int32_t)(k_uptime_get() / MSEC_PER_SEC));
		next_run += EPS_PERIOD_MS;
		/* Absolute deadlines avoid drift. Missed ticks do not cause a burst. */
		int64_t now = k_uptime_get();

		if (next_run <= now) {
			next_run += ((now - next_run) / EPS_PERIOD_MS + 1) * EPS_PERIOD_MS;
		}
		k_sleep(K_TIMEOUT_ABS_MS(next_run));
	}
}

K_THREAD_DEFINE(eps_thread, EPS_STACK_SIZE, eps_thread_entry, NULL, NULL, NULL,
		EPS_THREAD_PRIORITY, 0, SYS_FOREVER_MS);

void eps_start(struct eps_io io)
{
	thread_io = io;
	k_thread_start(eps_thread);
}
