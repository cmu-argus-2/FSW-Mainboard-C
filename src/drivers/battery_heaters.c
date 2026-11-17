/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT cmu_battery_heaters

#include "argus/drivers/battery_heaters.h"

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/* Lines in Python device_errors order. */
enum line { LINE_EN, LINE_HEAT0, LINE_HEAT1, LINE_COUNT };

static const uint8_t line_error[LINE_COUNT] = {
	BATT_HEATER_EN_GPIO_ERROR,
	BATT_HEATER_HEAT0_GPIO_ERROR,
	BATT_HEATER_HEAT1_GPIO_ERROR,
};

struct battery_heaters_config {
	struct gpio_dt_spec lines[LINE_COUNT];
	bool has_heat1;
};

struct battery_heaters_data {
	struct k_mutex lock;
	/* Last value written successfully (Python's __*_val error-handling shadows). */
	bool commanded[LINE_COUNT];
};

static bool line_present(const struct battery_heaters_config *config, enum line line)
{
	return line != LINE_HEAT1 || config->has_heat1;
}

static int write_line(const struct device *dev, enum line line, bool value)
{
	const struct battery_heaters_config *config = dev->config;
	struct battery_heaters_data *data = dev->data;
	int ret = gpio_pin_set_dt(&config->lines[line], value);

	if (ret == 0) {
		data->commanded[line] = value;
	}
	return ret;
}

/* Readback of the pad level, like CircuitPython's DigitalInOut.value. */
static int read_line(const struct device *dev, enum line line, bool *value)
{
	const struct battery_heaters_config *config = dev->config;
	int ret = gpio_pin_get_dt(&config->lines[line]);

	if (ret < 0) {
		return ret;
	}
	*value = ret != 0;
	return 0;
}

static int heater_line(const struct device *dev, enum battery_heater heater, enum line *line)
{
	const struct battery_heaters_config *config = dev->config;

	if (heater == BATTERY_HEATER_0) {
		*line = LINE_HEAT0;
		return 0;
	}
	if (heater == BATTERY_HEATER_1) {
		*line = LINE_HEAT1;
		return config->has_heat1 ? 0 : -ENOTSUP;
	}
	return -EINVAL;
}

static int enable_locked(const struct device *dev, enum line line)
{
	/* Python raises HEAT_EN only when it reads low; writing high
	 * unconditionally has the same effect and keeps its shadow in sync.
	 */
	int ret = write_line(dev, LINE_EN, true);

	if (ret != 0) {
		return ret;
	}
	return write_line(dev, line, true);
}

static int disable_locked(const struct device *dev, enum line line)
{
	const struct battery_heaters_config *config = dev->config;
	enum line other = line == LINE_HEAT0 ? LINE_HEAT1 : LINE_HEAT0;
	bool other_on = false;
	int ret = write_line(dev, line, false);

	if (ret != 0) {
		return ret;
	}
	/* Python checks the other heater's readback, but crashes in
	 * disable_heater0 when heater 1 is absent. An absent heater is off.
	 */
	if (line_present(config, other)) {
		ret = read_line(dev, other, &other_on);
		if (ret != 0) {
			return ret;
		}
	}
	return other_on ? 0 : write_line(dev, LINE_EN, false);
}

int battery_heaters_enable(const struct device *dev, enum battery_heater heater)
{
	struct battery_heaters_data *data = dev->data;
	enum line line;
	int ret = heater_line(dev, heater, &line);

	if (ret != 0) {
		return ret;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = enable_locked(dev, line);
	k_mutex_unlock(&data->lock);
	return ret;
}

int battery_heaters_disable(const struct device *dev, enum battery_heater heater)
{
	struct battery_heaters_data *data = dev->data;
	enum line line;
	int ret = heater_line(dev, heater, &line);

	if (ret != 0) {
		return ret;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = disable_locked(dev, line);
	k_mutex_unlock(&data->lock);
	return ret;
}

int battery_heaters_enabled(const struct device *dev, enum battery_heater heater,
			    bool *enabled)
{
	struct battery_heaters_data *data = dev->data;
	bool heater_on;
	bool en_on;
	enum line line;
	int ret = heater_line(dev, heater, &line);

	if (ret == -ENOTSUP) {
		*enabled = false;
		return 0;
	}
	if (ret != 0) {
		return ret;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = read_line(dev, line, &heater_on);
	if (ret == 0) {
		ret = read_line(dev, LINE_EN, &en_on);
	}
	k_mutex_unlock(&data->lock);
	if (ret == 0) {
		*enabled = heater_on && en_on;
	}
	return ret;
}

int battery_heaters_set(const struct device *dev, bool heater0, bool heater1)
{
	const struct battery_heaters_config *config = dev->config;
	struct battery_heaters_data *data = dev->data;
	int ret = 0;

	if (heater1 && !config->has_heat1) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	if (heater0) {
		ret = enable_locked(dev, LINE_HEAT0);
	}
	if (ret == 0 && heater1) {
		ret = enable_locked(dev, LINE_HEAT1);
	}
	if (ret == 0 && !heater0) {
		ret = disable_locked(dev, LINE_HEAT0);
	}
	if (ret == 0 && !heater1 && config->has_heat1) {
		ret = disable_locked(dev, LINE_HEAT1);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

/* TODO(HAL monitor): Python's tasks/hal_monitor.py polls device_errors for
 * every device and, for these three codes, only logs them (LOG_DATA_ERROR);
 * it does not reset or disable the heaters. When the C HAL monitor exists,
 * it should call this function periodically and apply that policy.
 */
int battery_heaters_device_errors(const struct device *dev,
				  struct battery_heaters_errors *errors)
{
	const struct battery_heaters_config *config = dev->config;
	struct battery_heaters_data *data = dev->data;
	int ret = 0;

	errors->count = 0;
	k_mutex_lock(&data->lock, K_FOREVER);
	for (enum line line = 0; line < LINE_COUNT && ret == 0; ++line) {
		bool value;

		if (!line_present(config, line)) {
			continue;
		}
		ret = read_line(dev, line, &value);
		if (ret == 0 && value != data->commanded[line]) {
			errors->codes[errors->count++] = line_error[line];
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int battery_heaters_init(const struct device *dev)
{
	const struct battery_heaters_config *config = dev->config;
	struct battery_heaters_data *data = dev->data;

	k_mutex_init(&data->lock);
	for (enum line line = 0; line < LINE_COUNT; ++line) {
		int ret;

		if (!line_present(config, line)) {
			continue;
		}
		if (!gpio_is_ready_dt(&config->lines[line])) {
			return -ENODEV;
		}
		/* Input stays enabled so the pad level can be read back; on
		 * RP2350 an output-only pin has its input buffer disabled.
		 */
		ret = gpio_pin_configure_dt(&config->lines[line],
					    GPIO_INPUT | GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
		data->commanded[line] = false;
	}
	return 0;
}

#define BATTERY_HEATERS_DEFINE(inst) \
	static struct battery_heaters_data data_##inst; \
	static const struct battery_heaters_config config_##inst = { \
		.lines = { \
			GPIO_DT_SPEC_INST_GET(inst, heat_en_gpios), \
			GPIO_DT_SPEC_INST_GET(inst, heat0_on_gpios), \
			GPIO_DT_SPEC_INST_GET_OR(inst, heat1_on_gpios, {0}), \
		}, \
		.has_heat1 = DT_INST_NODE_HAS_PROP(inst, heat1_on_gpios), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, battery_heaters_init, NULL, &data_##inst, \
			      &config_##inst, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(BATTERY_HEATERS_DEFINE)
