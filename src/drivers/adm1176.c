/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT adi_adm1176

#include "argus/drivers/adm1176.h"

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define V_MODES (ADM1176_V_CONT | ADM1176_V_ONCE)
#define I_MODES (ADM1176_I_CONT | ADM1176_I_ONCE)
#define STATUS_READ 0x40U
#define ALERT_EN 0x81U
#define ALERT_TH 0x82U
#define CONTROL 0x83U
#define ALERT_HOT_SWAP 0x04U
#define ALERT_ADC_OC4 0x02U
#define ALERT_CLEAR 0x10U

struct adm1176_config {
	struct i2c_dt_spec bus;
	uint32_t rshunt_micro_ohms;
	uint32_t voltage_range_microvolt;
};

struct adm1176_data {
	struct k_mutex lock;
	uint16_t raw_voltage;
	uint16_t raw_current;
	uint8_t command;
	uint8_t alert_enable;
	uint8_t overcurrent_level;
	bool restore_needed;
	bool sample_valid;
};

static int write_register(const struct adm1176_config *config, uint8_t reg, uint8_t value)
{
	const uint8_t bytes[] = {reg, value};

	return i2c_write_dt(&config->bus, bytes, sizeof(bytes));
}

static int configure_locked(const struct adm1176_config *config,
			    struct adm1176_data *data, uint8_t command)
{
	int ret = i2c_write_dt(&config->bus, &command, 1);

	data->sample_valid = false;
	data->restore_needed = ret != 0;
	if (ret == 0) {
		data->command = command;
	}
	return ret;
}

int adm1176_config(const struct device *dev, uint8_t command)
{
	struct adm1176_data *data = dev->data;
	int ret;

	if ((command & ~0x1FU) != 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = configure_locked(dev->config, data, command);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int adm1176_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	uint8_t bytes[3];
	int ret = 0;

	if (channel != SENSOR_CHAN_ALL && channel != SENSOR_CHAN_VOLTAGE &&
	    channel != SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	bool voltage = (data->command & V_MODES) != 0U;
	bool current = (data->command & I_MODES) != 0U;

	data->sample_valid = false;
	if ((channel == SENSOR_CHAN_VOLTAGE && !voltage) ||
	    (channel == SENSOR_CHAN_CURRENT && !current) || (!voltage && !current)) {
		ret = -ENOTSUP;
	} else {
		if (data->restore_needed) {
			ret = configure_locked(config, data, data->command);
		}
		if (ret == 0) {
			ret = i2c_read_dt(&config->bus, bytes, voltage && current ? 3 : 2);
		}
		if (ret == 0) {
			data->raw_voltage = ((uint16_t)bytes[0] << 4) |
					    (bytes[voltage && current ? 2 : 1] >> 4);
			data->raw_current = voltage && current ?
				((uint16_t)bytes[1] << 4) | (bytes[2] & 0x0FU) :
				((uint16_t)bytes[0] << 4) | (bytes[1] >> 4);
			data->sample_valid = true;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int adm1176_channel_get(const struct device *dev, enum sensor_channel channel,
			       struct sensor_value *value)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	if ((channel != SENSOR_CHAN_VOLTAGE && channel != SENSOR_CHAN_CURRENT) ||
	    (channel == SENSOR_CHAN_VOLTAGE && !(data->command & V_MODES)) ||
	    (channel == SENSOR_CHAN_CURRENT && !(data->command & I_MODES))) {
		ret = -ENOTSUP;
	} else if (!data->sample_valid) {
		ret = -ENODATA;
	} else {
		uint64_t micro = channel == SENSOR_CHAN_VOLTAGE ?
			(uint64_t)data->raw_voltage *
				((data->command & ADM1176_VRANGE) ? 6650000U : 26350000U) / 4096U :
			(uint64_t)data->raw_current * 105840000000ULL /
				(4096ULL * config->rshunt_micro_ohms);

		ret = sensor_value_from_micro(value, micro);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int adm1176_status(const struct device *dev, uint8_t *status)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	int ret, restore;

	if (status == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	uint8_t command = data->command | STATUS_READ;

	ret = i2c_write_dt(&config->bus, &command, 1);
	if (ret == 0) {
		ret = i2c_read_dt(&config->bus, status, 1);
	}
	/* Attempt restoration even after a failed read; keep the original error. */
	restore = i2c_write_dt(&config->bus, &data->command, 1);
	data->restore_needed = restore != 0;
	k_mutex_unlock(&data->lock);
	return ret != 0 ? ret : restore;
}

int adm1176_device_on(const struct device *dev, bool *on)
{
	uint8_t status;
	int ret;

	if (on == NULL) {
		return -EINVAL;
	}
	ret = adm1176_status(dev, &status);
	if (ret == 0) {
		*on = (status & ADM1176_STATUS_OFF) == 0U;
	}
	return ret;
}

int adm1176_set_device_on(const struct device *dev, bool on)
{
	struct adm1176_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = write_register(dev->config, CONTROL, on ? 0 : 1);
	if (ret == 0 && on) {
		ret = configure_locked(dev->config, data, ADM1176_V_CONT | ADM1176_I_CONT);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int adm1176_set_overcurrent_level(const struct device *dev, uint8_t level)
{
	struct adm1176_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	uint8_t enable = data->alert_enable | ALERT_ADC_OC4;

	ret = write_register(dev->config, ALERT_EN, enable);
	if (ret == 0) {
		data->alert_enable = enable;
		ret = write_register(dev->config, ALERT_TH, level);
		if (ret == 0) {
			data->overcurrent_level = level;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

uint8_t adm1176_overcurrent_level(const struct device *dev)
{
	struct adm1176_data *data = dev->data;
	uint8_t level;

	k_mutex_lock(&data->lock, K_FOREVER);
	level = data->overcurrent_level;
	k_mutex_unlock(&data->lock);
	return level;
}

int adm1176_clear(const struct device *dev)
{
	struct adm1176_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = write_register(dev->config, ALERT_EN, data->alert_enable | ALERT_CLEAR);
	k_mutex_unlock(&data->lock);
	return ret;
}

int adm1176_device_errors(const struct device *dev, struct adm1176_errors *errors)
{
	uint8_t status;
	int ret;

	if (errors == NULL) {
		return -EINVAL;
	}
	*errors = (struct adm1176_errors){0};
	ret = adm1176_status(dev, &status);
	if (ret == 0 && (status & ADM1176_STATUS_ADC_ALERT) != 0U) {
		errors->codes[0] = ADM1176_PWR_MON_ADC_ALERT_OVERCURRENT;
		errors->count = 1;
		ret = adm1176_clear(dev);
	}
	return ret;
}

static int adm1176_init(const struct device *dev)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}
	if (config->rshunt_micro_ohms == 0U) {
		return -EINVAL;
	}
	k_mutex_init(&data->lock);
	data->alert_enable = ALERT_HOT_SWAP;
	data->overcurrent_level = UINT8_MAX;
	return adm1176_config(dev, ADM1176_V_CONT | ADM1176_I_CONT |
		(config->voltage_range_microvolt == 6650000U ? ADM1176_VRANGE : 0));
}

static DEVICE_API(sensor, adm1176_api) = {
	.sample_fetch = adm1176_sample_fetch,
	.channel_get = adm1176_channel_get,
};

#define ADM1176_DEFINE(inst) \
	static struct adm1176_data data_##inst; \
	static const struct adm1176_config config_##inst = { \
		.bus = I2C_DT_SPEC_INST_GET(inst), \
		.rshunt_micro_ohms = DT_INST_PROP(inst, rshunt_micro_ohms), \
		.voltage_range_microvolt = DT_INST_PROP(inst, voltage_range_microvolt), \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, adm1176_init, NULL, &data_##inst, \
		&config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &adm1176_api);

DT_INST_FOREACH_STATUS_OKAY(ADM1176_DEFINE)
