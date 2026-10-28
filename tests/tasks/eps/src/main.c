/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/ztest.h>

struct fake_hw {
	struct eps_vi monitors[EPS_MONITOR_COUNT];
	struct eps_vi coils[EPS_COIL_COUNT];
	struct eps_fuel fuel;
	int32_t cpu;
	int bad_monitor;
	int bad_coil;
	bool bad_fuel;
	bool bad_cpu;
	bool bad_heaters;
	bool first;
	bool second;
	unsigned int monitor_reads;
	unsigned int coil_reads;
	unsigned int fuel_reads;
	unsigned int cpu_reads;
	unsigned int heater_writes;
	unsigned int full_records;
	unsigned int warning_records;
	int32_t last_full[EPS_FIELD_COUNT];
	int32_t last_warning[EPS_WARNING_COUNT];
};
static struct fake_hw hw;
static struct eps_state state;

static int monitor(void *ctx, enum eps_monitor which, struct eps_vi *out)
{
	struct fake_hw *h = ctx;

	++h->monitor_reads;
	*out = h->monitors[which]; /* Even a failing callback may touch its output. */
	return which == h->bad_monitor ? -ENODEV : 0;
}
static int coil(void *ctx, enum eps_axis which, struct eps_vi *out)
{
	struct fake_hw *h = ctx;

	++h->coil_reads;
	*out = h->coils[which];
	return which == h->bad_coil ? -EIO : 0;
}
static int fuel(void *ctx, struct eps_fuel *out)
{
	struct fake_hw *h = ctx;

	++h->fuel_reads;
	*out = h->fuel;
	return h->bad_fuel ? -EIO : 0;
}
static int cpu(void *ctx, int32_t *out)
{
	struct fake_hw *h = ctx;

	++h->cpu_reads;
	*out = h->cpu;
	return h->bad_cpu ? -EIO : 0;
}
static int heaters(void *ctx, bool first, bool second)
{
	struct fake_hw *h = ctx;

	++h->heater_writes;
	if (h->bad_heaters) { return -EIO; }
	h->first = first;
	h->second = second;
	return 0;
}
static void publish(void *ctx, bool full, const int32_t *values, size_t count)
{
	struct fake_hw *h = ctx;

	zassert_equal(count, full ? EPS_FIELD_COUNT : EPS_WARNING_COUNT);
	if (full) {
		++h->full_records;
		memcpy(h->last_full, values, sizeof(h->last_full));
	} else {
		++h->warning_records;
		memcpy(h->last_warning, values, sizeof(h->last_warning));
	}
}
static const struct eps_io io = {
	.context = &hw, .read_monitor = monitor, .read_coil = coil,
	.read_fuel = fuel, .read_cpu_temp = cpu, .set_heaters = heaters, .publish = publish,
};
static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&hw, 0, sizeof(hw));
	hw.bad_monitor = -1;
	hw.bad_coil = -1;
	hw.cpu = 2500;
	hw.fuel.soc_pct = 50;
	hw.fuel.ain1_temp_cc = 1000;
	hw.fuel.ain2_temp_cc = 1000;
	eps_init(&state);
}
static void five_runs(void)
{
	for (int i = 1; i <= 5; ++i) {
		(void)eps_step(&state, &io, i);
	}
}

ZTEST(eps, test_first_slow_work_is_exactly_the_fifth_run)
{
	for (int i = 1; i <= 4; ++i) {
		zassert_false(eps_step(&state, &io, i));
	}
	zassert_equal(hw.fuel_reads, 0);
	zassert_equal(hw.cpu_reads, 0);
	zassert_equal(hw.heater_writes, 0);
	zassert_equal(hw.warning_records, 4);
	zassert_true(eps_step(&state, &io, 5));
	zassert_equal(hw.full_records, 1);
	zassert_equal(hw.fuel_reads, 1);
	zassert_equal(hw.cpu_reads, 1);
	zassert_equal(hw.monitor_reads, 20);
	zassert_equal(hw.coil_reads, 30);
	zassert_equal(hw.last_full[EPS_TIME], 5);
	zassert_equal(hw.last_warning[0], 5);
	five_runs();
	zassert_equal(hw.full_records, 2);
	zassert_equal(hw.warning_records, 10);
}

ZTEST(eps, test_complete_44_field_order_and_latest_fast_readings)
{
	hw.monitors[EPS_MAIN] = (struct eps_vi){3000, 101};
	hw.monitors[EPS_PERIPHERAL] = (struct eps_vi){3300, 102};
	hw.monitors[EPS_RADIO] = (struct eps_vi){5000, 103};
	hw.monitors[EPS_JETSON] = (struct eps_vi){12000, 104};
	for (int i = 0; i < 6; ++i) {
		hw.coils[i] = (struct eps_vi){1000 + i, 200 + i};
	}
	hw.fuel = (struct eps_fuel){2100, -10, 700, 2400, 55, 1500, -40, 8000, 4000, 3600, 1800};
	five_runs();
	const int32_t expected[] = {
		5, 2, 2500, 3000, 101, 2100, -10, 700, 2400, 55, 1500, -40,
		8000, 4000, 3600, 1800,
		1000, 200, 1001, 201, 1002, 202, 1003, 203, 1004, 204, 1005, 205,
		12000, 104, 5000, 103, 3300, 102, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	};
	zassert_mem_equal(hw.last_full, expected, sizeof(expected));
}

ZTEST(eps, test_alert_order_and_exact_thresholds)
{
	hw.monitors[EPS_MAIN] = (struct eps_vi){1000, 1000};
	hw.monitors[EPS_PERIPHERAL] = (struct eps_vi){1000, 999};
	hw.monitors[EPS_RADIO] = (struct eps_vi){3300, 1000};
	hw.monitors[EPS_JETSON] = (struct eps_vi){16000, 999};
	for (int i = 0; i < 6; ++i) {
		hw.coils[i] = (struct eps_vi){-1500, i % 2 == 0 ? -1000 : -999};
	}
	(void)eps_step(&state, &io, 17);
	const int32_t expected[] = {17, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

	zassert_mem_equal(hw.last_warning, expected, sizeof(expected));
	zassert_true(eps_power_alert((struct eps_vi){INT32_MAX, INT32_MAX}, 16000));
	zassert_false(eps_power_alert((struct eps_vi){1000, -1000}, 1000));
}

ZTEST(eps, test_invalid_read_does_not_publish_partial_or_stale_values)
{
	hw.monitors[EPS_MAIN] = (struct eps_vi){5000, 1000};
	hw.coils[EPS_XP] = (struct eps_vi){6000, 2000};
	(void)eps_step(&state, &io, 1);
	hw.bad_monitor = EPS_MAIN;
	hw.bad_coil = EPS_XP;
	(void)eps_step(&state, &io, 2);
	zassert_equal(state.fields[EPS_MAIN_MV], 0);
	zassert_equal(state.fields[EPS_XP_MA], 0);
	zassert_equal(state.warnings[1], 0);
	zassert_equal(state.warnings[5], 0);
	zassert_equal(hw.monitor_reads, 8);
	zassert_equal(hw.coil_reads, 12);
}

ZTEST(eps, test_soc_hysteresis_and_invalid_inputs)
{
	for (int flag = EPS_NONE; flag <= EPS_NOMINAL; ++flag) {
		zassert_equal(eps_next_power_flag(flag, 0), EPS_LOW_POWER);
		zassert_equal(eps_next_power_flag(flag, 30), EPS_LOW_POWER);
		zassert_equal(eps_next_power_flag(flag, 40), EPS_NOMINAL);
		zassert_equal(eps_next_power_flag(flag, 100), EPS_NOMINAL);
		zassert_equal(eps_next_power_flag(flag, 35),
			      flag == EPS_LOW_POWER ? EPS_LOW_POWER : EPS_NOMINAL);
		zassert_equal(eps_next_power_flag(flag, -1), EPS_NONE);
		zassert_equal(eps_next_power_flag(flag, 101), EPS_NONE);
	}
	zassert_equal(eps_next_power_flag(99, 50), EPS_NONE);
	hw.fuel.soc_pct = 30;
	five_runs();
	zassert_equal(state.power_flag, EPS_LOW_POWER);
	hw.fuel.soc_pct = 35;
	five_runs();
	zassert_equal(state.power_flag, EPS_LOW_POWER);
	hw.fuel.soc_pct = 40;
	five_runs();
	zassert_equal(state.power_flag, EPS_NOMINAL);
}

ZTEST(eps, test_heater_hysteresis_and_independent_ain_channels)
{
	zassert_true(eps_next_heater(false, 0));
	zassert_false(eps_next_heater(false, 1));
	zassert_true(eps_next_heater(true, 499));
	zassert_false(eps_next_heater(true, 500));
	hw.fuel.ain1_temp_cc = -1;
	hw.fuel.ain2_temp_cc = 500;
	five_runs();
	zassert_true(hw.first);
	zassert_false(hw.second);
	hw.fuel.ain1_temp_cc = 499;
	hw.fuel.ain2_temp_cc = 0;
	five_runs();
	zassert_true(hw.first);
	zassert_true(hw.second);
	hw.fuel.ain1_temp_cc = 500;
	five_runs();
	zassert_false(hw.first);
	zassert_true(hw.second);
}

ZTEST(eps, test_missing_gauge_falls_back_to_cpu_minus_two_degrees)
{
	hw.bad_fuel = true;
	hw.cpu = 200;
	five_runs();
	zassert_true(hw.first);
	zassert_true(hw.second);
	zassert_equal(state.fields[EPS_CPU_TEMP_CC], 200);
	zassert_equal(state.fields[EPS_AIN1_TEMP_CC], 0);
	zassert_equal(state.power_flag, EPS_NONE);
	hw.cpu = 699;
	five_runs();
	zassert_true(hw.first);
	hw.cpu = 700;
	five_runs();
	zassert_false(hw.first);
	zassert_false(hw.second);
}

ZTEST(eps, test_missing_cpu_still_uses_valid_gauge)
{
	hw.bad_cpu = true;
	hw.fuel.ain1_temp_cc = 0;
	five_runs();
	zassert_true(hw.first);
	zassert_false(hw.second);
	zassert_equal(state.fields[EPS_CPU_TEMP_CC], 0);
}

ZTEST(eps, test_missing_both_temperature_sources_do_not_heat_on_zero)
{
	hw.fuel.ain1_temp_cc = 0;
	five_runs();
	zassert_true(hw.first);
	hw.bad_fuel = true;
	hw.bad_cpu = true;
	five_runs();
	zassert_false(hw.first);
	zassert_false(hw.second);
	zassert_equal(state.fields[EPS_SOC_PCT], 0);
	zassert_equal(state.power_flag, EPS_NOMINAL);
}

ZTEST(eps, test_failed_heater_write_does_not_claim_success)
{
	hw.bad_heaters = true;
	hw.fuel.ain1_temp_cc = -100;
	five_runs();
	zassert_false(state.heater1);
	zassert_equal(state.fields[EPS_HEATER1], 0);
}

ZTEST(eps, test_adm1176_conversions_match_spec_truncation)
{
	struct eps_vi v = eps_adm1176_convert(2048, 2048);

	zassert_equal(v.mv, 13175);
	zassert_equal(v.ma, 5292);
	v = eps_adm1176_convert(1, 1);
	zassert_equal(v.mv, 6);
	zassert_equal(v.ma, 2);
	v = eps_adm1176_convert(4095, 4095);
	zassert_equal(v.mv, 26343);
	zassert_equal(v.ma, 10581);
}

ZTEST(eps, test_drv8235_spec_voltage_and_192_current_exception)
{
	struct eps_vi v = eps_drv8235_convert(30, 96, false);

	zassert_equal(v.mv, 5019); /* int(30 * 0.16733 * 1000), not 5000 */
	zassert_equal(v.ma, 1850); /* 96 * 3700 / 192, not /255 */
	v = eps_drv8235_convert(1, 1, true);
	zassert_equal(v.mv, -167);
	zassert_equal(v.ma, -19); /* toward zero, not floor(-19.27) */
	v = eps_drv8235_convert(255, 192, false);
	zassert_equal(v.mv, 42669);
	zassert_equal(v.ma, 3700);
}

ZTEST(eps, test_document_temperature_encodings_and_offset)
{
	zassert_equal(eps_pack_temperature_cc(0x1900), 2500);
	zassert_equal(eps_pack_temperature_cc(1), 0);
	zassert_equal(eps_pack_temperature_cc(0xFF00), 25500); /* unsigned per spec */
	zassert_equal(eps_shadow_temperature_cc(2731), 0);
	zassert_equal(eps_shadow_temperature_cc(2730), -10);
	zassert_equal(eps_shadow_temperature_cc(2982), 2510);
}

ZTEST(eps, test_csv_field_order_and_negative_numbers)
{
	char line[EPS_RECORD_BUFFER_SIZE];
	int32_t values[EPS_FIELD_COUNT];

	for (int i = 0; i < EPS_FIELD_COUNT; ++i) { values[i] = i - 20; }
	zassert_true(eps_format_record(line, sizeof(line), true, values, EPS_FIELD_COUNT) > 0);
	zassert_true(strncmp(line, "EPS,", 4) == 0);
	char *cursor = line + 3;

	for (int i = 0; i < EPS_FIELD_COUNT; ++i) {
		zassert_equal(*cursor++, ',');
		char *end;

		zassert_equal(strtol(cursor, &end, 10), values[i]);
		cursor = end;
	}
	zassert_equal(*cursor, '\0');
	zassert_true(eps_format_record(line, sizeof(line), false, values, EPS_WARNING_COUNT) > 0);
	zassert_true(strncmp(line, "EPS_WARNING,", 12) == 0);
}

ZTEST(eps, test_csv_handles_extreme_values_and_rejects_small_buffers)
{
	char line[EPS_RECORD_BUFFER_SIZE];
	int32_t values[EPS_FIELD_COUNT];

	for (int i = 0; i < EPS_FIELD_COUNT; ++i) { values[i] = INT32_MIN; }
	zassert_true(eps_format_record(line, sizeof(line), true, values, EPS_FIELD_COUNT) > 0);
	zassert_equal(eps_format_record(line, 4, true, values, EPS_FIELD_COUNT), -ENOSPC);
	zassert_equal(eps_format_record(NULL, 4, true, values, EPS_FIELD_COUNT), -EINVAL);
	zassert_equal(eps_format_record(line, sizeof(line), true, values, 43), -EINVAL);
}

ZTEST(eps, test_console_only_provider_reports_unavailable_without_fake_measurements)
{
	const struct eps_io *unconnected = eps_console_only_io();
	struct eps_vi reading = {123, 456};
	struct eps_fuel pack = {.soc_pct = 55};
	int32_t temperature = 999;

	zassert_equal(unconnected->read_monitor(NULL, EPS_MAIN, &reading), -ENODEV);
	zassert_equal(unconnected->read_coil(NULL, EPS_XP, &reading), -ENODEV);
	zassert_equal(unconnected->read_fuel(NULL, &pack), -ENODEV);
	zassert_equal(unconnected->read_cpu_temp(NULL, &temperature), -ENODEV);
	zassert_equal(unconnected->set_heaters(NULL, true, true), -ENODEV);
	zassert_equal(reading.mv, 123);
	zassert_equal(pack.soc_pct, 55);
	zassert_equal(temperature, 999);
	for (int i = 1; i <= 5; ++i) {
		(void)eps_step(&state, unconnected, i);
	}
	zassert_equal(state.fields[EPS_TIME], 5);
	zassert_equal(state.fields[EPS_SOC_PCT], 0);
	zassert_equal(state.power_flag, EPS_NONE);
	zassert_false(state.heater1);
	zassert_false(state.heater2);
}

ZTEST_SUITE(eps, NULL, NULL, before, NULL, NULL);
