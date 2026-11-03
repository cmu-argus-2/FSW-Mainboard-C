/* SPDX-License-Identifier: Apache-2.0 */

/* Functional port of FSW-mainboard's drv8235.py and the Embassy Rust driver. */
#define DT_DRV_COMPAT ti_drv8235

#include "argus/drivers/drv8235.h"

#include <errno.h>
#include <math.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

enum {
	FAULT_STATUS = 0x00,
	VMTR = 0x04,
	IMTR = 0x05,
	CONFIG0 = 0x09,
	CONFIG3 = 0x0C,
	CONFIG4 = 0x0D,
	REG_CTRL0 = 0x0E,
	TARGET = 0x0F,
	RC_CTRL2 = 0x13,
	RC_CTRL3 = 0x14,
	EN_OUT = BIT(7),
	CLR_FLT = BIT(1),
	NPOR = BIT(1),
	FAULT = BIT(7),
	BRIDGE_MASK = 0x03,
};

#define VOLTS_PER_INDEX 0.16733f
#define INDEX_PER_VOLT  5.9761f
/* Team's EPS conversion exception to the original Python coefficient. */
#define AMPS_PER_INDEX  (3.7f / 192.0f)
#define THROTTLE_MAX    (6.0f / 42.67f)

struct drv8235_config {
	struct i2c_dt_spec bus;
};

struct drv8235_data {
	struct k_mutex lock;
	bool initialized;
};

static int read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	const struct drv8235_config *config = dev->config;

	return i2c_reg_read_byte_dt(&config->bus, reg, value);
}

static int write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct drv8235_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->bus, reg, value);
}

static int update_reg(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old;
	int ret = read_reg(dev, reg, &old);

	return ret == 0 ? write_reg(dev, reg, (old & (uint8_t)~mask) | (value & mask)) : ret;
}

/* Try both writes even if one fails, preserving the first error. */
static int coast_locked(const struct device *dev)
{
	int ret = write_reg(dev, TARGET, 0);
	int next = update_reg(dev, CONFIG4, BRIDGE_MASK, DRV8235_BRIDGE_COAST);

	return ret != 0 ? ret : next;
}

static int clear_faults_locked(const struct device *dev)
{
	struct drv8235_data *data = dev->data;
	uint8_t status;
	int ret = read_reg(dev, FAULT_STATUS, &status);

	if (ret != 0) {
		return ret;
	}
	if ((status & NPOR) == 0U) {
		data->initialized = false;
	}
	ret = coast_locked(dev);
	return ret == 0 ? update_reg(dev, CONFIG0, CLR_FLT, CLR_FLT) : ret;
}

int drv8235_init(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	/* Only the original Python/Rust settings; preserve unrelated device bits. */
	const struct {
		uint8_t reg, mask, value;
	} settings[] = {
		{CONFIG0, EN_OUT, 0}, /* Protected fields must be changed while disabled. */
		{CONFIG4, BIT(3) | BIT(2) | BRIDGE_MASK, BIT(3) | BIT(2)},
		{REG_CTRL0, BIT(4) | BIT(3), BIT(4) | BIT(3)},
		{TARGET, 0xFF, 0},
		{CONFIG3, BIT(4), BIT(4)},
		{RC_CTRL2, BIT(7) | BIT(6), BIT(7)},
		{RC_CTRL3, 0xFF, 41},
		{CONFIG0, CLR_FLT, CLR_FLT},
		{CONFIG0, EN_OUT, EN_OUT},
	};

	k_mutex_lock(&data->lock, K_FOREVER);
	data->initialized = false;
	int ret = i2c_is_ready_dt(&config->bus) ? 0 : -ENODEV;

	for (size_t i = 0; ret == 0 && i < ARRAY_SIZE(settings); ++i) {
		ret = update_reg(dev, settings[i].reg, settings[i].mask, settings[i].value);
	}
	if (ret == 0) {
		data->initialized = true;
	} else {
		(void)update_reg(dev, CONFIG0, EN_OUT, 0);
		(void)coast_locked(dev);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int drive(const struct device *dev, enum drv8235_bridge_mode mode, uint8_t index)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;
	uint8_t status;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = read_reg(dev, FAULT_STATUS, &status);

	if (ret == 0 && (status & NPOR) == 0U) {
		data->initialized = false;
	}
	if (ret == 0 && !data->initialized) {
		ret = -EAGAIN;
	}
	if (ret == 0 && (status & FAULT) != 0U) {
		ret = -EIO;
	}
	if (ret == 0) {
		ret = write_reg(dev, TARGET, index);
	}
	if (ret == 0) {
		ret = update_reg(dev, CONFIG4, BRIDGE_MASK, mode);
	}
	if (ret != 0) {
		(void)coast_locked(dev);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_set_throttle(const struct device *dev, float throttle)
{
	if (!isfinite(throttle)) {
		return -EINVAL;
	}
	float fraction = CLAMP(throttle, -1.0f, 1.0f) * THROTTLE_MAX;
	uint8_t index = (uint8_t)((fraction < 0 ? -fraction : fraction) * 255.0f);
	enum drv8235_bridge_mode mode = index == 0 ? DRV8235_BRIDGE_BRAKE :
		(throttle < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);

	return drive(dev, mode, index);
}

int drv8235_set_throttle_volts(const struct device *dev, float volts)
{
	if (!isfinite(volts)) {
		return -EINVAL;
	}
	volts = CLAMP(volts, -42.7f, 42.7f);
	unsigned int index = (unsigned int)((volts < 0 ? -volts : volts) * INDEX_PER_VOLT + 0.5f);
	enum drv8235_bridge_mode mode = volts == 0 ? DRV8235_BRIDGE_BRAKE :
		(volts < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);

	return drive(dev, mode, (uint8_t)MIN(index, 255U));
}

int drv8235_set_throttle_raw(const struct device *dev, int16_t raw)
{
	int value = CLAMP((int)raw, -255, 255);
	enum drv8235_bridge_mode mode = value == 0 ? DRV8235_BRIDGE_BRAKE :
		(value < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);

	/* Python's negative-raw write is fixed in Rust too: store the magnitude. */
	return drive(dev, mode, (uint8_t)(value < 0 ? -value : value));
}

int drv8235_coast(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = coast_locked(dev);

	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_get_throttle_raw(const struct device *dev, int16_t *raw)
{
	if (dev == NULL || raw == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;
	uint8_t index, bridge;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = read_reg(dev, TARGET, &index);

	if (ret == 0) {
		ret = read_reg(dev, CONFIG4, &bridge);
	}
	if (ret == 0) {
		switch (bridge & BRIDGE_MASK) {
		case DRV8235_BRIDGE_COAST:
			ret = -ENODATA;
			break;
		case DRV8235_BRIDGE_BRAKE:
			*raw = 0;
			break;
		case DRV8235_BRIDGE_REVERSE:
			*raw = -(int16_t)index;
			break;
		default:
			*raw = index;
			break;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_get_throttle(const struct device *dev, float *throttle)
{
	if (throttle == NULL) {
		return -EINVAL;
	}
	int16_t raw;
	int ret = drv8235_get_throttle_raw(dev, &raw);

	if (ret == 0) {
		int magnitude = raw < 0 ? -raw : raw;
		float rounded = ((magnitude * 1000 + 127) / 255) / 1000.0f;

		*throttle = (raw < 0 ? -rounded : rounded) / THROTTLE_MAX;
	}
	return ret;
}

int drv8235_get_throttle_volts(const struct device *dev, float *volts)
{
	if (volts == NULL) {
		return -EINVAL;
	}
	int16_t raw;
	int ret = drv8235_get_throttle_raw(dev, &raw);

	if (ret == 0) {
		*volts = raw * VOLTS_PER_INDEX;
	}
	return ret;
}

int drv8235_get_bridge_control(const struct device *dev, enum drv8235_bridge_mode *mode)
{
	if (dev == NULL || mode == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;
	uint8_t bridge;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = read_reg(dev, CONFIG4, &bridge);

	if (ret == 0) {
		*mode = bridge & BRIDGE_MASK;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_read_voltage_current(const struct device *dev, float *volts, float *amps)
{
	if (dev == NULL || volts == NULL || amps == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;
	uint8_t voltage, current, bridge;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = read_reg(dev, VMTR, &voltage);

	if (ret == 0) {
		ret = read_reg(dev, IMTR, &current);
	}
	if (ret == 0) {
		ret = read_reg(dev, CONFIG4, &bridge);
	}
	if (ret == 0) {
		int sign = (bridge & BRIDGE_MASK) == DRV8235_BRIDGE_REVERSE ? -1 : 1;

		*volts = sign * voltage * VOLTS_PER_INDEX;
		*amps = sign * current * AMPS_PER_INDEX;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_get_faults(const struct device *dev, struct drv8235_faults *faults)
{
	if (dev == NULL || faults == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;
	uint8_t status;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = read_reg(dev, FAULT_STATUS, &status);

	if (ret == 0) {
		struct drv8235_faults reading = {
			.raw = status,
			.stall = (status & BIT(5)) != 0,
			.overcurrent = (status & BIT(4)) != 0,
			.overvoltage = (status & BIT(3)) != 0,
			.thermal_shutdown = (status & BIT(2)) != 0,
			.power_on_reset = (status & NPOR) == 0,
		};

		if (reading.power_on_reset) {
			data->initialized = false;
		}
		if ((status & FAULT) != 0) {
			ret = clear_faults_locked(dev);
		}
		if (ret == 0) {
			*faults = reading;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int drv8235_clear_faults(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	struct drv8235_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = clear_faults_locked(dev);

	k_mutex_unlock(&data->lock);
	return ret;
}

static __maybe_unused int drv8235_device_init(const struct device *dev)
{
	struct drv8235_data *data = dev->data;

	k_mutex_init(&data->lock);
	return drv8235_init(dev);
}

#define DRV8235_DEFINE(inst)                                                         \
	static struct drv8235_data drv8235_data_##inst;                              \
	static const struct drv8235_config drv8235_config_##inst = {                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                   \
	};                                                                          \
	DEVICE_DT_INST_DEFINE(inst, drv8235_device_init, NULL, &drv8235_data_##inst,   \
			      &drv8235_config_##inst, POST_KERNEL,                     \
			      CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8235_DEFINE)
