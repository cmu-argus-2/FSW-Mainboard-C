/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps.h"
#include "argus/drivers/adm1176.h"
#include "argus/drivers/drv8235.h"
#include "argus/drivers/max17205.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

static const struct device *const monitors[EPS_MONITOR_COUNT] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(main_power_monitor)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gps_power_monitor)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(radio_power_monitor)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(jetson_power_monitor)),
};
static const struct device *const coils[EPS_COIL_COUNT] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_xp)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_xm)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_yp)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_ym)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_zp)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_zm)),
};
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17205)
static const struct device *const gauge = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(fuel_gauge));
#endif
static const struct device *const cpu_temp = DEVICE_DT_GET(DT_NODELABEL(die_temp));
static const struct gpio_dt_spec heat_enable =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), heat_en_gpios);
static const struct gpio_dt_spec heat1 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), heat0_on_gpios);
static const struct gpio_dt_spec heat2 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), heat1_on_gpios);

struct eps_hardware {
	bool monitors[EPS_MONITOR_COUNT];
	bool coils[EPS_COIL_COUNT];
	bool gauge;
	bool heaters;
};
static struct eps_hardware hardware;

static int read_monitor(void *context, enum eps_monitor monitor, struct eps_vi *reading)
{
#if DT_HAS_COMPAT_STATUS_OKAY(adi_adm1176)
	struct eps_hardware *hw = context;
	struct adm1176_sample sample;

	if (!hw->monitors[monitor]) {
		return -ENODEV;
	}
	int rc = adm1176_read_sample(monitors[monitor], &sample);

	if (rc == 0) {
		*reading = eps_adm1176_convert(sample.raw_voltage, sample.raw_current);
	}
	return rc;
#else
	ARG_UNUSED(context);
	ARG_UNUSED(monitor);
	ARG_UNUSED(reading);
	return -ENODEV;
#endif
}

static int read_coil(void *context, enum eps_axis axis, struct eps_vi *reading)
{
#if DT_HAS_COMPAT_STATUS_OKAY(ti_drv8235)
	struct eps_hardware *hw = context;
	struct drv8235_telemetry telemetry;

	if (!hw->coils[axis]) {
		return -ENODEV;
	}
	int rc = drv8235_get_telemetry(coils[axis], &telemetry);

	if (rc == 0) {
		/* Deliberate legacy EPS presentation: no driver change to sign, VMTR
		 * scaling, or validity semantics. Use the spec's raw conversion here.
		 */
		*reading = eps_drv8235_convert(telemetry.raw_voltage, telemetry.raw_current,
				telemetry.bridge_mode == DRV8235_BRIDGE_REVERSE);
	}
	return rc;
#else
	ARG_UNUSED(context);
	ARG_UNUSED(axis);
	ARG_UNUSED(reading);
	return -ENODEV;
#endif
}

static int read_fuel(void *context, struct eps_fuel *reading)
{
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17205)
	struct eps_hardware *hw = context;
	struct eps_fuel result;
	struct max17205_raw_temperatures temperatures;
	union fuel_gauge_prop_val value;
	int rc;

	if (!hw->gauge) {
		return -ENODEV;
	}
	/* Standard properties keep the gauge's existing Zephyr interface. TTE/TTF
	 * intentionally retain its integer-minute resolution, converted to seconds.
	 */
#define READ_PROP(prop, destination, expression) \
	do { \
		rc = fuel_gauge_get_prop(gauge, (prop), &value); \
		if (rc != 0) { return rc; } \
		(destination) = (expression); \
	} while (0)
	READ_PROP(FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT, result.soc_pct,
		  value.relative_state_of_charge_pct);
	READ_PROP(FUEL_GAUGE_REMAINING_CAPACITY_UAH, result.capacity_mah,
		  value.remaining_capacity_uah / 1000);
	READ_PROP(FUEL_GAUGE_CURRENT_UA, result.current_ma, value.current_ua / 1000);
	READ_PROP(FUEL_GAUGE_VOLTAGE_UV, result.voltage_mv, value.voltage_uv / 1000);
	READ_PROP(MAX17205_PROP_MID_VOLTAGE_UV, result.midpoint_mv, value.custom_uint / 1000);
	READ_PROP(FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS, result.tte_s,
		  value.runtime_to_empty_mins * 60);
	READ_PROP(FUEL_GAUGE_RUNTIME_TO_FULL_MINS, result.ttf_s,
		  value.runtime_to_full_mins * 60);
#undef READ_PROP
	rc = max17205_read_raw_temperatures(gauge, &temperatures);
	if (rc != 0) {
		return rc;
	}
	result.pack_temp_cc = eps_pack_temperature_cc(temperatures.pack);
	result.ain1_temp_cc = eps_shadow_temperature_cc(temperatures.ain1);
	result.ain2_temp_cc = eps_shadow_temperature_cc(temperatures.ain2);
	result.die_temp_cc = eps_shadow_temperature_cc(temperatures.die);
	*reading = result;
	return 0;
#else
	ARG_UNUSED(context);
	ARG_UNUSED(reading);
	return -ENODEV;
#endif
}

static int read_cpu(void *context, int32_t *temp_cc)
{
	struct sensor_value value;

	ARG_UNUSED(context);
	if (!device_is_ready(cpu_temp)) {
		return -ENODEV;
	}
	int rc = sensor_sample_fetch(cpu_temp);

	if (rc == 0) {
		rc = sensor_channel_get(cpu_temp, SENSOR_CHAN_DIE_TEMP, &value);
	}
	if (rc == 0) {
		*temp_cc = (int32_t)(sensor_value_to_micro(&value) / 10000);
	}
	return rc;
}

static int set_heaters(void *context, bool first, bool second)
{
	struct eps_hardware *hw = context;

	if (!hw->heaters) {
		return -ENODEV;
	}
	/* Disable the common enable before changing either channel. Restore it
	 * only after both writes succeed. No brief energizing of an old channel.
	 */
	int rc = gpio_pin_set_dt(&heat_enable, 0);

	if (rc == 0) { rc = gpio_pin_set_dt(&heat1, first); }
	if (rc == 0) { rc = gpio_pin_set_dt(&heat2, second); }
	if (rc == 0) { rc = gpio_pin_set_dt(&heat_enable, first || second); }
	if (rc != 0) {
		(void)gpio_pin_set_dt(&heat_enable, 0);
		(void)gpio_pin_set_dt(&heat1, 0);
		(void)gpio_pin_set_dt(&heat2, 0);
	}
	return rc;
}

static void publish(void *context, bool full, const int32_t *fields, size_t count)
{
	char line[EPS_RECORD_BUFFER_SIZE];

	ARG_UNUSED(context);
	if (eps_format_record(line, sizeof(line), full, fields, count) >= 0) {
		printk("%s\n", line);
	}
}

const struct eps_io *eps_zephyr_init(void)
{
	static const struct eps_io io = {
		.context = &hardware,
		.read_monitor = read_monitor,
		.read_coil = read_coil,
		.read_fuel = read_fuel,
		.read_cpu_temp = read_cpu,
		.set_heaters = set_heaters,
		.publish = publish,
	};

	/* Board initialization owns rails, settling, buses and boot-low hogs.
	 * This task deliberately neither edits nor replaces the board definitions.
	 * A node that is not present yet is treated as an unavailable device.
	 */
	for (size_t i = 0; i < EPS_MONITOR_COUNT; ++i) {
		hardware.monitors[i] = monitors[i] != NULL && device_is_ready(monitors[i]);
		printk("EPS monitor %u: %s\n", (unsigned int)i,
		       hardware.monitors[i] ? "ready" : "unavailable");
	}
	for (size_t i = 0; i < EPS_COIL_COUNT; ++i) {
		hardware.coils[i] = coils[i] != NULL && device_is_ready(coils[i]);
#if DT_HAS_COMPAT_STATUS_OKAY(ti_drv8235)
		if (hardware.coils[i]) {
			/* Match Python's final startup state: zero target, coast,
			 * EN_OUT enabled. This never commands a nonzero coil voltage.
			 */
			int rc = drv8235_coast(coils[i]);

			if (rc == 0) { rc = drv8235_set_enabled(coils[i], true); }
			hardware.coils[i] = rc == 0;
		}
#endif
		printk("EPS coil %u: %s\n", (unsigned int)i,
		       hardware.coils[i] ? "ready" : "unavailable");
	}
	hardware.gauge = false;
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17205)
	union fuel_gauge_prop_val probe;
	struct max17205_raw_temperatures temperatures;

	/* The existing gauge init only checks bus readiness, not its two ACKs. */
	hardware.gauge = gauge != NULL && device_is_ready(gauge) &&
		fuel_gauge_get_prop(gauge, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT, &probe) == 0 &&
		max17205_read_raw_temperatures(gauge, &temperatures) == 0;
#endif
	hardware.heaters = gpio_is_ready_dt(&heat_enable) && gpio_is_ready_dt(&heat1) &&
		gpio_is_ready_dt(&heat2);
	if (hardware.heaters) {
		/* Execute every boot-low write even if an earlier one fails. */
		int enable_rc = gpio_pin_configure_dt(&heat_enable, GPIO_OUTPUT_INACTIVE);
		int first_rc = gpio_pin_configure_dt(&heat1, GPIO_OUTPUT_INACTIVE);
		int second_rc = gpio_pin_configure_dt(&heat2, GPIO_OUTPUT_INACTIVE);

		hardware.heaters = enable_rc == 0 && first_rc == 0 && second_rc == 0;
	}
	printk("EPS fuel gauge: %s; heaters: %s\n",
	       hardware.gauge ? "ready" : "unavailable",
	       hardware.heaters ? "ready" : "unavailable");
	return &io;
}
