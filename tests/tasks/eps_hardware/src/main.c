/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps_hardware.h"
#include "adm1176_emul.h"
#include "drv8235_emul.h"
#include "max17205_emul.h"
#include "argus/drivers/adm1176.h"
#include "argus/drivers/max17205.h"

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/ztest.h>

static const struct emul *const main_emul = EMUL_DT_GET(DT_NODELABEL(main_power_monitor));
static const struct emul *const gps_emul = EMUL_DT_GET(DT_NODELABEL(gps_power_monitor));
static const struct emul *const radio_emul = EMUL_DT_GET(DT_NODELABEL(radio_power_monitor));
static const struct emul *const jetson_emul = EMUL_DT_GET(DT_NODELABEL(jetson_power_monitor));
static const struct emul *const gauge_emul = EMUL_DT_GET(DT_NODELABEL(fuel_gauge));
static const struct emul *const coil_emuls[EPS_COIL_COUNT] = {
	[EPS_XP] = EMUL_DT_GET(DT_NODELABEL(torque_xp)),
	[EPS_XM] = EMUL_DT_GET(DT_NODELABEL(torque_xm)),
	[EPS_YP] = EMUL_DT_GET(DT_NODELABEL(torque_yp)),
	[EPS_YM] = EMUL_DT_GET(DT_NODELABEL(torque_ym)),
	[EPS_ZP] = EMUL_DT_GET(DT_NODELABEL(torque_zp)),
	[EPS_ZM] = EMUL_DT_GET(DT_NODELABEL(torque_zm)),
};
static const struct device *const gpio = DEVICE_DT_GET(DT_NODELABEL(eps_test_gpio));
static const struct gpio_dt_spec gps_power =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), gps_en_gpios);
static struct eps_state state;
static struct eps_io io;
static int32_t full_fields[EPS_FIELD_COUNT];
static uint16_t full_valid;
static uint16_t warning_valid;

static void publish(void *context, bool full, const int32_t *fields, size_t count,
		    uint16_t valid)
{
	ARG_UNUSED(context);
	zassert_equal(count, full ? EPS_FIELD_COUNT : EPS_WARNING_COUNT);
	if (full) {
		memcpy(full_fields, fields, sizeof(full_fields));
		full_valid = valid;
	} else {
		warning_valid = valid;
	}
}

static void set_monitor(const struct emul *emul, uint16_t voltage, uint16_t current)
{
	struct adm1176_emul_state *s = adm1176_emul_state(emul);

	s->sample[0] = voltage >> 4;
	s->sample[1] = current >> 4;
	s->sample[2] = ((voltage & 0xf) << 4) | (current & 0xf);
}

static void *setup(void)
{
	zassert_false(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(radio_power_monitor))));
	zassert_false(regulator_is_enabled(DEVICE_DT_GET(DT_NODELABEL(coil_power))));
	io = eps_hardware_io();
	zassert_true(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(radio_power_monitor))));
	zassert_equal(gpio_emul_output_get(gpio, 3), 1);
	zassert_equal(gpio_emul_output_get(gps_power.port, gps_power.pin), 1);
	zassert_true(regulator_is_enabled(DEVICE_DT_GET(DT_NODELABEL(coil_power))));
	zassert_equal(gpio_emul_output_get(gpio, 4), 1);
	zassert_equal(gpio_emul_output_get(gpio, 0), 0);
	zassert_equal(gpio_emul_output_get(gpio, 1), 0);
	zassert_equal(gpio_emul_output_get(gpio, 2), 0);
	io.publish = publish;
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	adm1176_emul_reset(main_emul);
	adm1176_emul_reset(gps_emul);
	zassert_ok(adm1176_config(DEVICE_DT_GET(DT_NODELABEL(main_power_monitor)), 0x05));
	zassert_ok(adm1176_config(DEVICE_DT_GET(DT_NODELABEL(gps_power_monitor)), 0x05));
	set_monitor(main_emul, 514, 14);
	set_monitor(gps_emul, 514, 1);
	set_monitor(radio_emul, 600, 20);
	set_monitor(jetson_emul, 1300, 24);
	for (enum eps_axis axis = EPS_XM; axis < EPS_COIL_COUNT; ++axis) {
		drv8235_emul_set_reg(coil_emuls[axis], 0x04, axis);
	}
	drv8235_emul_set_reg(coil_emuls[EPS_XP], 0x04, 30);
	drv8235_emul_set_reg(coil_emuls[EPS_XP], 0x05, 192);
	drv8235_emul_set_reg(coil_emuls[EPS_XP], 0x0d, 0x0d); /* reverse */
	max17205_emul_reset(gauge_emul);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x06, 27 << 8);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x05, 5626);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x0a, (uint16_t)-102);
	max17205_emul_set_reg(gauge_emul, 0x36, 0xda, 6451);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x09, 51536);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x11, 1000);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x20, 320);
	max17205_emul_set_reg(gauge_emul, 0x36, 0x08, 5811);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x34, 2959);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x3b, 2958);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x35, 2952);
	eps_init(&state);
	zassert_ok(io.set_heaters(NULL, false, false));
	memset(full_fields, 0, sizeof(full_fields));
	full_valid = 0;
	warning_valid = 0;
}

static void five_runs(void)
{
	for (int i = 1; i <= 5; ++i) {
		(void)eps_step(&state, &io, i);
	}
}

ZTEST(eps_hardware, test_drivers_to_complete_eps_record)
{
	five_runs();
	const int32_t expected[EPS_FIELD_COUNT] = {
		5, 1, 0, 3306, 36, 2260, 2275, 2265, 2205, 27, 2813, -15,
		8063, 4026, 5580, 1800,
		-5019, -3700, 167, 0, 334, 0, 501, 0, 669, 0, 836, 0,
		8363, 62, 3859, 51, 3306, 2,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	};

	for (size_t i = 0; i < ARRAY_SIZE(expected); ++i) {
		zassert_equal(full_fields[i], expected[i], "field %u: got %ld, expected %ld",
			      (unsigned int)i, (long)full_fields[i], (long)expected[i]);
	}
	zassert_equal(warning_valid, 0x3ff);
	zassert_equal(full_valid, 0xbff);
	zassert_equal(state.power_flag, EPS_LOW_POWER);
}

ZTEST(eps_hardware, test_monitor_error_and_recovery_to_true_zero)
{
	(void)eps_step(&state, &io, 1);
	adm1176_emul_state(main_emul)->error = -EIO;
	(void)eps_step(&state, &io, 2);
	zassert_equal(warning_valid, 0x3fe);
	zassert_equal(state.fields[EPS_MAIN_MV], 0);
	set_monitor(main_emul, 0, 0);
	(void)eps_step(&state, &io, 3);
	zassert_equal(warning_valid, 0x3ff);
	zassert_equal(state.fields[EPS_MAIN_MV], 0);
	zassert_equal(state.fields[EPS_MAIN_MA], 0);
}

ZTEST(eps_hardware, test_gauge_failure_then_recovery)
{
	five_runs();
	max17205_emul_set_nack(gauge_emul, true);
	five_runs();
	zassert_equal(full_valid, 0x3ff);
	zassert_equal(full_fields[EPS_SOC_PCT], 0);
	zassert_equal(state.power_flag, EPS_LOW_POWER);
	max17205_emul_set_nack(gauge_emul, false);
	five_runs();
	zassert_equal(full_valid, 0xbff);
	zassert_equal(full_fields[EPS_SOC_PCT], 27);
}

ZTEST(eps_hardware, test_signed_pack_and_exact_shadow_offset)
{
	max17205_emul_set_reg(gauge_emul, 0x36, 0x08, 0xff00);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x34, 2732);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x3b, 2731);
	five_runs();
	zassert_equal(full_fields[EPS_PACK_TEMP_CC], -100);
	zassert_equal(full_fields[EPS_AIN1_TEMP_CC], 5);
	zassert_equal(full_fields[EPS_AIN2_TEMP_CC], -5);
	zassert_equal(full_fields[EPS_HEATER1], 0);
	zassert_equal(full_fields[EPS_HEATER2], 1);
	zassert_equal(gpio_emul_output_get(gpio, 0), 1);
	zassert_equal(gpio_emul_output_get(gpio, 1), 0);
	zassert_equal(gpio_emul_output_get(gpio, 2), 1);
}

ZTEST(eps_hardware, test_heater_shared_enable_and_hysteresis)
{
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x34, 2731);
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x3b, 2731);
	five_runs();
	zassert_equal(gpio_emul_output_get(gpio, 0), 1);
	zassert_equal(gpio_emul_output_get(gpio, 1), 1);
	zassert_equal(gpio_emul_output_get(gpio, 2), 1);

	/* 2782 dK is 5.05 C, above the 5 C exit threshold. */
	max17205_emul_set_reg(gauge_emul, 0x0b, 0x34, 2782);
	five_runs();
	zassert_equal(gpio_emul_output_get(gpio, 0), 1);
	zassert_equal(gpio_emul_output_get(gpio, 1), 0);
	zassert_equal(gpio_emul_output_get(gpio, 2), 1);

	max17205_emul_set_reg(gauge_emul, 0x0b, 0x3b, 2782);
	five_runs();
	zassert_equal(gpio_emul_output_get(gpio, 0), 0);
	zassert_equal(gpio_emul_output_get(gpio, 2), 0);
	zassert_equal(full_fields[EPS_HEATER1], 0);
	zassert_equal(full_fields[EPS_HEATER2], 0);
}

ZTEST(eps_hardware, test_coil_read_error_marks_only_affected_axis_invalid)
{
	drv8235_emul_set_error(coil_emuls[EPS_XP], -EIO);
	(void)eps_step(&state, &io, 1);
	zassert_equal(warning_valid, 0x3ef);
	zassert_equal(state.fields[EPS_XP_MV], 0);
	(void)eps_step(&state, &io, 2);
	zassert_equal(warning_valid, 0x3ef);
	drv8235_emul_set_error(coil_emuls[EPS_XP], 0);
	(void)eps_step(&state, &io, 3);
	zassert_equal(warning_valid, 0x3ff);
	zassert_equal(state.fields[EPS_XP_MV], -5019);
	drv8235_emul_set_error(coil_emuls[EPS_XP], -EIO);
	(void)eps_step(&state, &io, 4);
	zassert_equal(warning_valid, 0x3ef);
	drv8235_emul_set_error(coil_emuls[EPS_XP], 0);
}

/* Ten 1 Hz invocations produce ten warning records and two full records. */
static struct {
	int64_t uptime_ms;
	bool full;
} thread_records[12];
static size_t record_count;
static k_tid_t publisher;
static K_SEM_DEFINE(records_ready, 0, 1);

static void observe_publish(void *context, bool full, const int32_t *fields, size_t count,
			    uint16_t valid)
{
	if (record_count < ARRAY_SIZE(thread_records)) {
		publisher = k_current_get();
		thread_records[record_count].uptime_ms = k_uptime_get();
		thread_records[record_count].full = full;
		eps_console_publish(context, full, fields, count, valid);
		if (++record_count == ARRAY_SIZE(thread_records)) {
			k_sem_give(&records_ready);
		}
	}
}

ZTEST(eps_hardware, test_thread_priority_and_periodic_records)
{
	struct eps_io observed_io = io;

	observed_io.publish = observe_publish;
	eps_start(observed_io);
	int ret = k_sem_take(&records_ready, K_SECONDS(12));

	zassert_not_null(publisher);
	zassert_not_equal(publisher, k_current_get());
	int priority = k_thread_priority_get(publisher);

	if (IS_ENABLED(CONFIG_ARM)) {
		size_t unused;

		zassert_ok(k_thread_stack_space_get(publisher, &unused));
		printk("EPS test thread stack high-water mark: %zu / %zu bytes\n",
		       publisher->stack_info.size - unused, publisher->stack_info.size);
	}
	k_thread_abort(publisher);
	zassert_ok(ret);
	zassert_equal(priority, K_PRIO_PREEMPT(2));
	size_t index = 0;

	for (int run = 1; run <= 10; ++run) {
		zassert_false(thread_records[index].full);
		int64_t elapsed = thread_records[index].uptime_ms - thread_records[0].uptime_ms;

		zassert_within(elapsed, (run - 1) * 1000, 100);
		++index;
		if (run % 5 == 0) {
			zassert_true(thread_records[index].full);
			++index;
		}
	}
}

ZTEST_SUITE(eps_hardware, NULL, setup, before, NULL, NULL);
