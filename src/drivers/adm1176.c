/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adm1176.c
 * @brief Zephyr driver for the ADM1176 power monitor and hot-swap controller.
 */

#define DT_DRV_COMPAT adi_adm1176

#include "argus/drivers/adm1176.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(adm1176, CONFIG_SENSOR_LOG_LEVEL);

enum {
	ADM1176_COMMAND_V_CONT = 1U << 0,
	ADM1176_COMMAND_I_CONT = 1U << 2,
	ADM1176_COMMAND_VRANGE = 1U << 4,
	ADM1176_COMMAND_STATUS_READ = 1U << 6,

	ADM1176_REGISTER_ALERT_ENABLE = 0x81,
	ADM1176_REGISTER_ALERT_THRESHOLD = 0x82,
	ADM1176_REGISTER_CONTROL = 0x83,

	ADM1176_ALERT_ENABLE_ADC_OC4 = 1U << 1,
	ADM1176_ALERT_ENABLE_HOT_SWAP = 1U << 2,
	ADM1176_ALERT_CLEAR = 1U << 4,

	ADM1176_CONTROL_SWITCH_OFF = 1U << 0,
};

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
	uint8_t control;
	uint8_t overcurrent_threshold;
};

static int adm1176_write_extended_register(const struct adm1176_config *config,
					   uint8_t reg, uint8_t value)
{
	const uint8_t message[2] = {reg, value};

	return i2c_write_dt(&config->bus, message, sizeof(message));
}

int adm1176_read_sample(const struct device *dev, struct adm1176_sample *sample)
{
	if (dev == NULL || sample == NULL) {
		return -EINVAL;
	}
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	uint8_t buffer[3];
	struct adm1176_sample reading;

	k_mutex_lock(&data->lock, K_FOREVER);
	int result = i2c_read_dt(&config->bus, buffer, sizeof(buffer));

	if (result == 0) {
		result = adm1176_decode_measurement(buffer, config->rshunt_micro_ohms,
					    config->voltage_range_microvolt, &reading);
	}
	if (result == 0) {
		data->raw_voltage = reading.raw_voltage;
		data->raw_current = reading.raw_current;
		*sample = reading;
	}
	k_mutex_unlock(&data->lock);
	return result;
}

static int adm1176_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	uint8_t buffer[3];
	int result;

	if (channel != SENSOR_CHAN_ALL && channel != SENSOR_CHAN_VOLTAGE &&
	    channel != SENSOR_CHAN_CURRENT && channel != SENSOR_CHAN_POWER) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	result = i2c_read_dt(&config->bus, buffer, sizeof(buffer));
	if (result == 0) {
		data->raw_voltage = ((uint16_t)buffer[0] << 4) |
				    ((uint16_t)buffer[2] >> 4);
		data->raw_current = ((uint16_t)buffer[1] << 4) |
				    ((uint16_t)buffer[2] & 0x0FU);
	}
	k_mutex_unlock(&data->lock);

	return result;
}

static int adm1176_channel_get(const struct device *dev, enum sensor_channel channel,
			       struct sensor_value *value)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	struct adm1176_sample sample;
	uint8_t buffer[3];
	int result;

	if (value == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	buffer[0] = (uint8_t)(data->raw_voltage >> 4);
	buffer[1] = (uint8_t)(data->raw_current >> 4);
	buffer[2] = (uint8_t)(((data->raw_voltage & 0x0FU) << 4) |
				      (data->raw_current & 0x0FU));
	result = adm1176_decode_measurement(buffer, config->rshunt_micro_ohms,
					    config->voltage_range_microvolt, &sample);
	k_mutex_unlock(&data->lock);
	if (result != 0) {
		return result;
	}

	switch (channel) {
	case SENSOR_CHAN_VOLTAGE:
		return sensor_value_from_micro(value, (int64_t)sample.voltage_uv);
	case SENSOR_CHAN_CURRENT:
		return sensor_value_from_micro(value, (int64_t)sample.current_ua);
	case SENSOR_CHAN_POWER:
		return sensor_value_from_micro(value, (int64_t)sample.power_uw);
	default:
		return -ENOTSUP;
	}
}

int adm1176_read_status(const struct device *dev, uint8_t *status)
{
	if (dev == NULL || status == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	const uint8_t status_command = data->command | ADM1176_COMMAND_STATUS_READ;
	int result;
	int restore_result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = i2c_write_dt(&config->bus, &status_command, sizeof(status_command));
	if (result == 0) {
		result = i2c_read_dt(&config->bus, status, 1U);
	}
	restore_result = i2c_write_dt(&config->bus, &data->command,
				      sizeof(data->command));
	k_mutex_unlock(&data->lock);

	return result != 0 ? result : restore_result;
}

int adm1176_get_status_flags(const struct device *dev,
			     struct adm1176_status_flags *status)
{
	uint8_t raw;
	int result;

	if (status == NULL) {
		return -EINVAL;
	}

	result = adm1176_read_status(dev, &raw);
	if (result != 0) {
		return result;
	}

	return adm1176_decode_status(raw, status);
}

int adm1176_is_enabled(const struct device *dev, bool *enabled)
{
	uint8_t status;
	int result;

	if (enabled == NULL) {
		return -EINVAL;
	}

	result = adm1176_read_status(dev, &status);
	if (result == 0) {
		*enabled = (status & ADM1176_STATUS_OFF) == 0U;
	}

	return result;
}

int adm1176_set_enabled(const struct device *dev, bool enabled)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	const uint8_t new_control = enabled ? 0U : ADM1176_CONTROL_SWITCH_OFF;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = adm1176_write_extended_register(config, ADM1176_REGISTER_CONTROL,
						 new_control);
	if (result == 0) {
		data->control = new_control;
		if (enabled) {
			result = i2c_write_dt(&config->bus, &data->command,
					      sizeof(data->command));
		}
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int adm1176_set_overcurrent_threshold_raw(const struct device *dev,
					 uint8_t threshold)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	const uint8_t new_alert_enable =
		data->alert_enable | ADM1176_ALERT_ENABLE_ADC_OC4;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = adm1176_write_extended_register(config,
						 ADM1176_REGISTER_ALERT_ENABLE,
						 new_alert_enable);
	if (result == 0) {
		data->alert_enable = new_alert_enable;
		result = adm1176_write_extended_register(
			config, ADM1176_REGISTER_ALERT_THRESHOLD, threshold);
		if (result == 0) {
			data->overcurrent_threshold = threshold;
		}
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int adm1176_set_overcurrent_threshold(const struct device *dev,
				      uint64_t current_ua)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	uint8_t threshold;
	int result = adm1176_threshold_from_microamps(
		current_ua, config->rshunt_micro_ohms, &threshold);

	if (result != 0) {
		return result;
	}

	return adm1176_set_overcurrent_threshold_raw(dev, threshold);
}

int adm1176_get_overcurrent_threshold(const struct device *dev,
				      uint64_t *current_ua)
{
	if (dev == NULL || current_ua == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	uint8_t threshold;

	k_mutex_lock(&data->lock, K_FOREVER);
	threshold = data->overcurrent_threshold;
	k_mutex_unlock(&data->lock);

	return adm1176_threshold_to_microamps(
		threshold, config->rshunt_micro_ohms, current_ua);
}

int adm1176_clear_alerts(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = adm1176_write_extended_register(
		config, ADM1176_REGISTER_ALERT_ENABLE,
		data->alert_enable | ADM1176_ALERT_CLEAR);
	k_mutex_unlock(&data->lock);

	return result;
}

static int adm1176_init(const struct device *dev)
{
	const struct adm1176_config *config = dev->config;
	struct adm1176_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}
	if (config->rshunt_micro_ohms == 0U) {
		LOG_ERR("rshunt-micro-ohms must be nonzero");
		return -EINVAL;
	}

	k_mutex_init(&data->lock);
	data->command = ADM1176_COMMAND_V_CONT | ADM1176_COMMAND_I_CONT;
	if (config->voltage_range_microvolt ==
	    ADM1176_VOLTAGE_FULL_SCALE_LOW_UV) {
		data->command |= ADM1176_COMMAND_VRANGE;
	}
	data->alert_enable = ADM1176_ALERT_ENABLE_HOT_SWAP;
	data->control = 0U;
	data->overcurrent_threshold = UINT8_MAX;

	return i2c_write_dt(&config->bus, &data->command, sizeof(data->command));
}

static DEVICE_API(sensor, adm1176_driver_api) = {
	.sample_fetch = adm1176_sample_fetch,
	.channel_get = adm1176_channel_get,
};

#define ADM1176_DEFINE(inst)                                                   \
	static struct adm1176_data adm1176_data_##inst;                         \
	static const struct adm1176_config adm1176_config_##inst = {            \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                \
		.rshunt_micro_ohms = DT_INST_PROP(inst, rshunt_micro_ohms),       \
		.voltage_range_microvolt =                                        \
			DT_INST_PROP(inst, voltage_range_microvolt),                \
	};                                                                        \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, adm1176_init, NULL,                    \
				     &adm1176_data_##inst,                        \
				     &adm1176_config_##inst, POST_KERNEL,         \
				     CONFIG_SENSOR_INIT_PRIORITY,                  \
				     &adm1176_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADM1176_DEFINE)
