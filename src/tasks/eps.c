/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static const enum eps_field monitor_fields[EPS_MONITOR_COUNT] = {
	EPS_MAIN_MV, EPS_PERIPHERAL_MV, EPS_RADIO_MV, EPS_JETSON_MV,
};
static const int32_t monitor_limits[EPS_MONITOR_COUNT] = {
	EPS_MAIN_LIMIT_MW, EPS_PERIPHERAL_LIMIT_MW, EPS_RADIO_LIMIT_MW, EPS_JETSON_LIMIT_MW,
};

_Static_assert(EPS_FIELD_COUNT == 44, "EPS telemetry field order changed");
_Static_assert(EPS_HEATER1 == 42 && EPS_SOLAR_XP_MV == 34, "EPS indices changed");
_Static_assert(EPS_VALID_FUEL <= UINT16_MAX, "EPS validity bits exceed the record mask");

void eps_init(struct eps_state *state)
{
	memset(state, 0, sizeof(*state));
}

static void store_vi(struct eps_state *state, enum eps_field field, struct eps_vi reading)
{
	state->fields[field] = reading.mv;
	state->fields[field + 1] = reading.ma;
}

static uint16_t slow_step(struct eps_state *state, const struct eps_io *io)
{
	struct eps_fuel fuel = {0};
	int32_t cpu_cc = 0;
	bool cpu_valid = io->read_cpu_temp(io->context, &cpu_cc) == 0;
	bool fuel_valid = io->read_fuel(io->context, &fuel) == 0;

	state->fields[EPS_CPU_TEMP_CC] = cpu_valid ? cpu_cc : 0;
	/* Clear the whole fuel group on a failed read, avoiding partially new data. */
	memset(&state->fields[EPS_PACK_TEMP_CC], 0,
	       (EPS_TTF_S - EPS_PACK_TEMP_CC + 1) * sizeof(int32_t));
	if (fuel_valid) {
		state->fields[EPS_PACK_TEMP_CC] = fuel.pack_temp_cc;
		state->fields[EPS_AIN1_TEMP_CC] = fuel.ain1_temp_cc;
		state->fields[EPS_AIN2_TEMP_CC] = fuel.ain2_temp_cc;
		state->fields[EPS_DIE_TEMP_CC] = fuel.die_temp_cc;
		state->fields[EPS_SOC_PCT] = fuel.soc_pct;
		state->fields[EPS_CAPACITY_MAH] = fuel.capacity_mah;
		state->fields[EPS_PACK_MA] = fuel.current_ma;
		state->fields[EPS_PACK_MV] = fuel.voltage_mv;
		state->fields[EPS_MIDPOINT_MV] = fuel.midpoint_mv;
		state->fields[EPS_TTE_S] = fuel.tte_s;
		state->fields[EPS_TTF_S] = fuel.ttf_s;
		enum eps_power_flag next = eps_next_power_flag(state->power_flag, fuel.soc_pct);

		if (next != EPS_NONE) {
			state->power_flag = next;
		}
	}
	state->fields[EPS_POWER_FLAG] = state->power_flag;

	/* An unreadable temperature is not a zero-degree measurement. If neither
	 * source is available, request both heaters off instead of heating on zero.
	 */
	bool heater1 = false;
	bool heater2 = false;

	if (fuel_valid || cpu_valid) {
		int32_t t1 = fuel_valid ? fuel.ain1_temp_cc : cpu_cc - 200;
		int32_t t2 = fuel_valid ? fuel.ain2_temp_cc : cpu_cc - 200;

		heater1 = eps_next_heater(state->heater1, t1);
		heater2 = eps_next_heater(state->heater2, t2);
	}
	if (io->set_heaters(io->context, heater1, heater2) == 0) {
		state->heater1 = heater1;
		state->heater2 = heater2;
	}
	state->fields[EPS_HEATER1] = state->heater1;
	state->fields[EPS_HEATER2] = state->heater2;
	return (cpu_valid ? EPS_VALID_CPU_TEMP : 0U) | (fuel_valid ? EPS_VALID_FUEL : 0U);
}

bool eps_step(struct eps_state *state, const struct eps_io *io, int32_t time_s)
{
	uint16_t valid_reads = 0;

	state->fields[EPS_TIME] = time_s;
	memset(state->warnings, 0, sizeof(state->warnings));
	state->warnings[0] = time_s;
	for (enum eps_monitor i = 0; i < EPS_MONITOR_COUNT; ++i) {
		struct eps_vi reading = {0};

		if (io->read_monitor(io->context, i, &reading) == 0) {
			valid_reads |= EPS_VALID_MONITOR(i);
			state->warnings[1 + i] = eps_power_alert(reading, monitor_limits[i]);
		} else {
			reading = (struct eps_vi){0};
		}
		store_vi(state, monitor_fields[i], reading);
	}
	for (enum eps_axis i = 0; i < EPS_COIL_COUNT; ++i) {
		struct eps_vi reading = {0};

		if (io->read_coil(io->context, i, &reading) == 0) {
			valid_reads |= EPS_VALID_COIL(i);
			state->warnings[5 + i] = eps_power_alert(reading, EPS_COIL_LIMIT_MW);
		} else {
			reading = (struct eps_vi){0};
		}
		store_vi(state, EPS_XP_MV + 2 * i, reading);
	}

	bool full = ++state->phase == EPS_SLOW_PERIOD;
	uint16_t full_valid_reads = valid_reads;

	if (full) {
		state->phase = 0;
		full_valid_reads |= slow_step(state, io);
	}
	io->publish(io->context, false, state->warnings, EPS_WARNING_COUNT, valid_reads);
	if (full) {
		io->publish(io->context, true, state->fields, EPS_FIELD_COUNT, full_valid_reads);
	}
	return full;
}

int eps_format_record(char *buffer, size_t size, bool full, const int32_t *fields,
		      size_t count)
{
	if (buffer == NULL || fields == NULL || size == 0 ||
	    count != (full ? EPS_FIELD_COUNT : EPS_WARNING_COUNT)) {
		return -EINVAL;
	}
	int used = snprintf(buffer, size, "%s", full ? "EPS" : "EPS_WARNING");

	if (used < 0 || (size_t)used >= size) {
		return -ENOSPC;
	}
	for (size_t i = 0; i < count; ++i) {
		int written = snprintf(buffer + used, size - used, ",%ld", (long)fields[i]);

		if (written < 0 || (size_t)written >= size - used) {
			return -ENOSPC;
		}
		used += written;
	}
	return used;
}
