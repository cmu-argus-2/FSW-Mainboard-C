/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/eps_task.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/drv8235.h"
#include "argus/drivers/max17205.h"

LOG_MODULE_REGISTER(eps_hardware);

static const struct device *const monitors[EPS_MONITOR_COUNT] = {
	[EPS_MAIN] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(main_power_monitor)),
	[EPS_PERIPHERAL] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gps_power_monitor)),
	[EPS_RADIO] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(radio_power_monitor)),
	[EPS_JETSON] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(jetson_power_monitor)),
};
static const char *const monitor_names[EPS_MONITOR_COUNT] = {
	[EPS_MAIN] = "ADM1176 main",
	[EPS_PERIPHERAL] = "ADM1176 GPS",
	[EPS_RADIO] = "ADM1176 radio",
	[EPS_JETSON] = "ADM1176 Jetson",
};
static const struct device *const coils[EPS_COIL_COUNT] = {
	[EPS_XP] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_xp)),
	[EPS_XM] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_xm)),
	[EPS_YP] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_yp)),
	[EPS_YM] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_ym)),
	[EPS_ZP] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_zp)),
	[EPS_ZM] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(torque_zm)),
};
static const char *const coil_names[EPS_COIL_COUNT] = {
	[EPS_XP] = "DRV8235 XP",
	[EPS_XM] = "DRV8235 XM",
	[EPS_YP] = "DRV8235 YP",
	[EPS_YM] = "DRV8235 YM",
	[EPS_ZP] = "DRV8235 ZP",
	[EPS_ZM] = "DRV8235 ZM",
};
static const struct device *const coil_power = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(coil_power));
static const struct device *const gauge = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(fuel_gauge));
static const struct device *const cpu_temp = DEVICE_DT_GET_OR_NULL(DT_ALIAS(die_temp0));

#define EPS_GPIO(prop) GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), prop, {0})
static const struct gpio_dt_spec gps_power = EPS_GPIO(gps_en_gpios);
static const struct gpio_dt_spec radio_power = EPS_GPIO(lora_en_gpios);
static const struct gpio_dt_spec heat_enable = EPS_GPIO(heat_en_gpios);
static const struct gpio_dt_spec heat0 = EPS_GPIO(heat0_on_gpios);
static const struct gpio_dt_spec heat1 = EPS_GPIO(heat1_on_gpios);

static struct {
	bool initialized;
	bool coils_ready[EPS_COIL_COUNT];
	bool heaters_ready;
	bool heater_state_known;
	bool first;
	bool second;
	int monitor_errors[EPS_MONITOR_COUNT];
	int coil_errors[EPS_COIL_COUNT];
	int cpu_error;
	int fuel_error;
	int heater_error;
} hardware;

static int report_io_result(const char *device, const char *operation, int ret, int *last_error)
{
	/* Log the first failure and changes in errno. A successful I/O clears
	 * the previous error so the next outage is logged again.
	 */
	if (ret != 0 && ret != *last_error) {
		LOG_WRN("%s %s failed (%d)", device, operation, ret);
	}
	*last_error = ret;
	return ret;
}

static int enable_power(const struct gpio_dt_spec *pin)
{
	return gpio_is_ready_dt(pin) ? gpio_pin_configure_dt(pin, GPIO_OUTPUT_ACTIVE) : -ENODEV;
}

static void init_radio_monitor(void)
{
	const struct device *dev = monitors[EPS_RADIO];

	if (dev == NULL || device_is_ready(dev)) {
		return;
	}
	int ret = enable_power(&radio_power);

	if (ret != 0) {
		LOG_WRN("Radio monitor supply enable failed (%d)", ret);
		return;
	}
	/* Allow the switched LoRa rail to settle before initializing U13. */
	k_msleep(100);
	ret = device_init(dev);

	if (ret != 0) {
		LOG_WRN("Radio monitor initialization failed (%d)", ret);
	}
}

static void init_coils(void)
{
	if (!device_is_ready(coil_power)) {
		LOG_INF("EPS coil supply is unavailable in this board configuration");
		return;
	}
	int ret = regulator_is_enabled(coil_power) ? 0 : regulator_enable(coil_power);

	if (ret != 0) {
		LOG_WRN("Coil supply enable failed (%d)", ret);
		return;
	}
	for (enum eps_axis axis = 0; axis < EPS_COIL_COUNT; ++axis) {
		if (!device_is_ready(coils[axis])) {
			continue;
		}
		ret = drv8235_init(coils[axis]);
		hardware.coils_ready[axis] = ret == 0;
		if (ret != 0) {
			LOG_WRN("%s initialization failed (%d)", coil_names[axis], ret);
		}
	}
}

static bool init_heaters(void)
{
	const struct gpio_dt_spec *pins[] = {&heat_enable, &heat0, &heat1};

	for (size_t i = 0; i < ARRAY_SIZE(pins); ++i) {
		if (pins[i]->port == NULL || !gpio_is_ready_dt(pins[i]) ||
		    gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE) != 0) {
			LOG_WRN("Heater GPIO initialization failed");
			return false;
		}
	}
	return true;
}

static int write_heaters(bool first, bool second)
{
	/* Gate off before clearing both channels. Program the channels before
	 * asserting the shared enable when either channel is requested.
	 */
	int ret = first || second ? 0 : gpio_pin_set_dt(&heat_enable, 0);

	if (ret == 0) {
		ret = gpio_pin_set_dt(&heat0, first);
	}
	if (ret == 0) {
		ret = gpio_pin_set_dt(&heat1, second);
	}
	if (ret == 0 && (first || second)) {
		ret = gpio_pin_set_dt(&heat_enable, 1);
	}
	return ret;
}

static int set_heaters(void *context, bool first, bool second)
{
	ARG_UNUSED(context);
	if (!hardware.heaters_ready) {
		return report_io_result("Battery heaters", "write", -ENODEV, &hardware.heater_error);
	}
	if (hardware.heater_state_known && first == hardware.first && second == hardware.second) {
		return report_io_result("Battery heaters", "write", 0, &hardware.heater_error);
	}
	int ret = write_heaters(first, second);

	if (ret == 0) {
		hardware.first = first;
		hardware.second = second;
		hardware.heater_state_known = true;
	} else {
		int restore = write_heaters(hardware.first, hardware.second);

		hardware.heater_state_known = restore == 0;
		if (!hardware.heater_state_known) {
			LOG_ERR("Could not restore heater GPIOs (%d)", restore);
		}
	}
	return report_io_result("Battery heaters", "write", ret, &hardware.heater_error);
}

static int read_monitor(void *context, enum eps_monitor monitor, struct eps_vi *out)
{
	struct sensor_value voltage;
	struct sensor_value current;

	ARG_UNUSED(context);
	if (monitor < EPS_MAIN || monitor >= EPS_MONITOR_COUNT) {
		return -EINVAL;
	}
	const struct device *dev = monitors[monitor];

	int ret = device_is_ready(dev) ? sensor_sample_fetch(dev) : -ENODEV;

	if (ret == 0) {
		ret = sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
	}
	if (ret == 0) {
		ret = sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
	}
	if (ret == 0) {
		*out = (struct eps_vi) {
			.mv = sensor_value_to_milli(&voltage),
			.ma = sensor_value_to_milli(&current),
		};
	}
	return report_io_result(monitor_names[monitor], "read", ret,
				&hardware.monitor_errors[monitor]);
}

static int read_coil(void *context, enum eps_axis axis, struct eps_vi *out)
{
	float voltage;
	float current;

	ARG_UNUSED(context);
	if (axis < EPS_XP || axis >= EPS_COIL_COUNT) {
		return -EINVAL;
	}
	if (coils[axis] == NULL) {
		return -ENODEV;
	}
	/* Like the Python EPS, report the driver's readings in every bridge
	 * mode. Validity denotes I/O success, including in coast.
	 */
	int ret = hardware.coils_ready[axis] ?
			  drv8235_read_voltage_current(coils[axis], &voltage, &current) : -ENODEV;

	if (ret == 0) {
		*out = (struct eps_vi) {
			.mv = (int32_t)(voltage * 1000),
			.ma = (int32_t)(current * 1000),
		};
	}
	return report_io_result(coil_names[axis], "read", ret, &hardware.coil_errors[axis]);
}

static int read_cpu_temp(void *context, int32_t *out)
{
	struct sensor_value reading;

	ARG_UNUSED(context);
	int ret = device_is_ready(cpu_temp) ?
			  sensor_sample_fetch_chan(cpu_temp, SENSOR_CHAN_DIE_TEMP) : -ENODEV;

	if (ret == 0) {
		ret = sensor_channel_get(cpu_temp, SENSOR_CHAN_DIE_TEMP, &reading);
	}
	if (ret == 0) {
		*out = sensor_value_to_micro(&reading) / 10000;
	}
	return report_io_result("RP2350 CPU temperature", "read", ret, &hardware.cpu_error);
}

enum fuel_reading {
	FUEL_SOC, FUEL_CAPACITY, FUEL_CURRENT, FUEL_VOLTAGE, FUEL_MIDPOINT,
	FUEL_TTE, FUEL_TTF, FUEL_PACK_TEMP, FUEL_AIN1, FUEL_AIN2, FUEL_DIE,
	FUEL_READING_COUNT,
};

static const fuel_gauge_prop_t fuel_properties[FUEL_READING_COUNT] = {
	[FUEL_SOC] = FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT,
	[FUEL_CAPACITY] = FUEL_GAUGE_REMAINING_CAPACITY_UAH,
	[FUEL_CURRENT] = FUEL_GAUGE_CURRENT_UA,
	[FUEL_VOLTAGE] = FUEL_GAUGE_VOLTAGE_UV,
	[FUEL_MIDPOINT] = MAX17205_PROP_MID_VOLTAGE_UV,
	[FUEL_TTE] = FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS,
	[FUEL_TTF] = FUEL_GAUGE_RUNTIME_TO_FULL_MINS,
	[FUEL_PACK_TEMP] = FUEL_GAUGE_TEMPERATURE_DK,
	[FUEL_AIN1] = MAX17205_PROP_TEMP_AIN1_DK,
	[FUEL_AIN2] = MAX17205_PROP_TEMP_AIN2_DK,
	[FUEL_DIE] = MAX17205_PROP_TEMP_DIE_DK,
};

static int read_fuel(void *context, struct eps_fuel *out)
{
	union fuel_gauge_prop_val values[FUEL_READING_COUNT] = {0};

	ARG_UNUSED(context);
	int ret = device_is_ready(gauge) ?
			  fuel_gauge_get_props(gauge, fuel_properties, values, ARRAY_SIZE(values)) :
			  -ENODEV;

	report_io_result("MAX17205", "read", ret, &hardware.fuel_error);
	if (ret != 0) {
		return ret;
	}
	/* The pack property adds 2732 after signed Celsius decoding. Undo that
	 * encoding. The shadow properties are absolute 0.1 K readings.
	 */
	*out = (struct eps_fuel) {
		.pack_temp_cc = ((int32_t)values[FUEL_PACK_TEMP].temperature_dk - 2732) * 10,
		.ain1_temp_cc = values[FUEL_AIN1].custom_int * 10 - 27315,
		.ain2_temp_cc = values[FUEL_AIN2].custom_int * 10 - 27315,
		.die_temp_cc = values[FUEL_DIE].custom_int * 10 - 27315,
		.soc_pct = values[FUEL_SOC].relative_state_of_charge_pct,
		.capacity_mah = values[FUEL_CAPACITY].remaining_capacity_uah / 1000,
		.current_ma = values[FUEL_CURRENT].current_ua / 1000,
		.voltage_mv = values[FUEL_VOLTAGE].voltage_uv / 1000,
		.midpoint_mv = values[FUEL_MIDPOINT].custom_uint / 1000,
		.tte_s = values[FUEL_TTE].runtime_to_empty_mins * 60,
		.ttf_s = values[FUEL_TTF].runtime_to_full_mins * 60,
	};
	return 0;
}

struct eps_io eps_hardware_io(void)
{
	if (!hardware.initialized) {
		hardware.heaters_ready = init_heaters();
		hardware.heater_state_known = hardware.heaters_ready;
		int ret = enable_power(&gps_power);

		if (ret != 0) {
			LOG_WRN("GPS supply enable failed (%d)", ret);
		}
		init_radio_monitor();
		init_coils();
		hardware.initialized = true;
	}
	return (struct eps_io) {
		.read_monitor = read_monitor,
		.read_coil = read_coil,
		.read_fuel = read_fuel,
		.read_cpu_temp = read_cpu_temp,
		.set_heaters = set_heaters,
		.publish = eps_console_publish,
	};
}
