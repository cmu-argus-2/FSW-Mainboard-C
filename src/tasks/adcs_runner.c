/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/adcs.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static enum argus_sat_state state_detumbling(void *context)
{
	ARG_UNUSED(context);
	return ARGUS_STATE_DETUMBLING;
}

static int vec_unavailable(void *context, float out[3])
{
	ARG_UNUSED(context);
	ARG_UNUSED(out);
	return -ENODEV;
}

static int range_unavailable(void *context, enum adcs_gyro_range range)
{
	ARG_UNUSED(context);
	ARG_UNUSED(range);
	return -ENODEV;
}

static int lux_unavailable(void *context, enum adcs_face face, float *lux)
{
	ARG_UNUSED(context);
	ARG_UNUSED(face);
	ARG_UNUSED(lux);
	return -ENODEV;
}

static bool coil_not_available(void *context, enum adcs_coil coil)
{
	ARG_UNUSED(context);
	ARG_UNUSED(coil);
	return false;
}

static int coil_unavailable(void *context, enum adcs_coil coil, float throttle)
{
	ARG_UNUSED(context);
	ARG_UNUSED(coil);
	ARG_UNUSED(throttle);
	return -ENODEV;
}

static int coil_power_unavailable(void *context, bool on)
{
	ARG_UNUSED(context);
	ARG_UNUSED(on);
	return -ENODEV;
}

static int64_t kernel_uptime_ms(void *context)
{
	ARG_UNUSED(context);
	return k_uptime_get();
}

static void kernel_sleep_ms(void *context, int32_t ms)
{
	ARG_UNUSED(context);
	k_msleep(ms);
}

static void console_publish(void *context, const struct adcs_record *record,
			    uint16_t valid_reads)
{
	char line[ADCS_RECORD_BUFFER_SIZE];

	ARG_UNUSED(context);
	if (adcs_format_record(line, sizeof(line), record) >= 0) {
		printk("%s\nADCS_VALIDITY,%lu,0x%03x\n", line, (unsigned long)record->time,
		       (unsigned int)valid_reads);
	}
}

const struct adcs_io *adcs_console_only_io(void)
{
	static const struct adcs_io io = {
		.get_sat_state = state_detumbling,
		.read_gyro = vec_unavailable,
		.read_mag = vec_unavailable,
		.set_gyro_range = range_unavailable,
		.read_lux = lux_unavailable,
		.coil_available = coil_not_available,
		.set_coil = coil_unavailable,
		.set_coil_power = coil_power_unavailable,
		.uptime_ms = kernel_uptime_ms,
		.sleep_ms = kernel_sleep_ms,
		.publish = console_publish,
	};

	return &io;
}

void adcs_run(const struct adcs_io *io)
{
	struct adcs_state state;
	int64_t next_run = io->uptime_ms(io->context);

	adcs_init(&state, NULL);
	for (;;) {
		adcs_step(&state, io, (uint32_t)(io->uptime_ms(io->context) / 1000));
		next_run += ADCS_PERIOD_MS;
		/* A B-cross/sun cycle lasts the whole period. Start the next run
		 * immediately rather than skipping a period, like the Python
		 * scheduler running a late task.
		 */
		int64_t now = io->uptime_ms(io->context);

		if (next_run < now) {
			next_run = now;
		}
		k_sleep(K_TIMEOUT_ABS_MS(next_run));
	}
}
