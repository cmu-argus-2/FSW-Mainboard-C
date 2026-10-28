/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static const enum eps_field monitor_fields[EPS_MONITOR_COUNT] = {
	EPS_MAIN_MV, EPS_PERIPHERAL_MV, EPS_RADIO_MV, EPS_JETSON_MV,
};
static const int32_t monitor_limits[EPS_MONITOR_COUNT] = {1000, 1000, 3300, 16000};

_Static_assert(EPS_FIELD_COUNT == 44, "EPS telemetry field order changed");
_Static_assert(EPS_HEATER1 == 42 && EPS_SOLAR_XP_MV == 34, "EPS indices changed");

void eps_init(struct eps_state *state)
{
	memset(state, 0, sizeof(*state));
}

struct eps_vi eps_adm1176_convert(uint16_t raw_v, uint16_t raw_i)
{
	return (struct eps_vi){
		.mv = (int32_t)((uint32_t)raw_v * 26350U / 4096U),
		.ma = (int32_t)((uint32_t)raw_i * 10584U / 4096U),
	};
}

struct eps_vi eps_drv8235_convert(uint8_t raw_v, uint8_t raw_i, bool reverse)
{
	const int32_t sign = reverse ? -1 : 1;

	/* V = raw * 0.16733 V. Apply the document's sign convention even though
	 * IMTR itself is unsigned winding current, not signed battery current.
	 */
	return (struct eps_vi){
		.mv = sign * ((int32_t)raw_v * 16733 / 100),
		.ma = sign * ((int32_t)raw_i * 3700 / 192),
	};
}

int32_t eps_pack_temperature_cc(uint16_t raw)
{
	/* The specification intentionally preserves Python's UNSIGNED reading. */
	return (int32_t)((uint32_t)raw * 100U / 256U);
}

int32_t eps_shadow_temperature_cc(uint16_t raw)
{
	/* Preserve the spec's 2731 offset, including its 0.05 C approximation. */
	return ((int32_t)raw - 2731) * 10;
}

enum eps_power_flag eps_next_power_flag(enum eps_power_flag previous, int32_t soc)
{
	if (soc < 0 || soc > 100 || previous < EPS_NONE || previous > EPS_NOMINAL) {
		return EPS_NONE;
	}
	if (soc <= 30) {
		return EPS_LOW_POWER;
	}
	if (soc >= 40) {
		return EPS_NOMINAL;
	}
	return previous == EPS_LOW_POWER ? EPS_LOW_POWER : EPS_NOMINAL;
}

bool eps_next_heater(bool enabled, int32_t temp_cc)
{
	if (!enabled && temp_cc <= 0) {
		return true;
	}
	if (enabled && temp_cc >= 500) {
		return false;
	}
	return enabled;
}

bool eps_power_alert(struct eps_vi reading, int32_t threshold_mw)
{
	/* Compare before dividing: fractional mW must not change a threshold test. */
	return (int64_t)reading.mv * reading.ma >= (int64_t)threshold_mw * 1000;
}

static void store_vi(struct eps_state *state, enum eps_field field, struct eps_vi reading)
{
	state->fields[field] = reading.mv;
	state->fields[field + 1] = reading.ma;
}

static void slow_step(struct eps_state *state, const struct eps_io *io)
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
}

bool eps_step(struct eps_state *state, const struct eps_io *io, int32_t time_s)
{
	state->fields[EPS_TIME] = time_s;
	memset(state->warnings, 0, sizeof(state->warnings));
	state->warnings[0] = time_s;
	for (enum eps_monitor i = 0; i < EPS_MONITOR_COUNT; ++i) {
		struct eps_vi reading = {0};

		if (io->read_monitor(io->context, i, &reading) != 0) {
			reading = (struct eps_vi){0};
		}
		store_vi(state, monitor_fields[i], reading);
		state->warnings[1 + i] = eps_power_alert(reading, monitor_limits[i]);
	}
	for (enum eps_axis i = 0; i < EPS_COIL_COUNT; ++i) {
		struct eps_vi reading = {0};

		if (io->read_coil(io->context, i, &reading) != 0) {
			reading = (struct eps_vi){0};
		}
		store_vi(state, EPS_XP_MV + 2 * i, reading);
		state->warnings[5 + i] = eps_power_alert(reading, 1500);
	}

	bool full = ++state->phase == EPS_SLOW_PERIOD;

	if (full) {
		state->phase = 0;
		slow_step(state, io);
	}
	io->publish(io->context, false, state->warnings, EPS_WARNING_COUNT);
	if (full) {
		io->publish(io->context, true, state->fields, EPS_FIELD_COUNT);
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
