/* SPDX-License-Identifier: Apache-2.0 */

/* Functional port of FSW-mainboard's drv8235.py and the Embassy Rust driver. */
#define DT_DRV_COMPAT ti_drv8235

#include "argus/drivers/drv8235.h"

#include <errno.h>
#include <math.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
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
#define TARGET_FULL_SCALE_UV 42670000U

struct drv8235_config {
	struct i2c_dt_spec bus;
	const struct device *supply;
	uint32_t max_output_uv;
	uint32_t resistance_milliohm;
	uint32_t imtr_full_scale_ua;
};

struct drv8235_data {
	struct k_mutex lock;
	bool initialized;
};

static uint8_t max_code(const struct drv8235_config *config)
{
	/* Floor the safety ceiling so rounding cannot command more than the limit. */
	return (uint64_t)config->max_output_uv * 255U / TARGET_FULL_SCALE_UV;
}

static int supply_ready(const struct drv8235_config *config)
{
	if (!device_is_ready(config->supply) || !i2c_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}
	return regulator_is_enabled(config->supply) ? 0 : -EAGAIN;
}

static int resistance_settings(uint32_t milliohm, uint8_t *scale, uint8_t *inv_r)
{
	const uint32_t scales[] = {2, 64, 1024, 8192};

	if (milliohm == 0U) {
		return -EINVAL;
	}
	for (int i = ARRAY_SIZE(scales) - 1; i >= 0; --i) {
		uint64_t value = ((uint64_t)scales[i] * 1000U + milliohm / 2U) / milliohm;

		if (value >= 1U && value <= 255U) {
			*scale = i;
			*inv_r = value;
			return 0;
		}
	}
	return -ERANGE;
}

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
	uint8_t scale, inv_r;
	int ret = resistance_settings(config->resistance_milliohm, &scale, &inv_r);

	if (ret != 0) {
		return ret;
	}
	/* Only the original Python/Rust settings; preserve unrelated device bits. */
	const struct {
		uint8_t reg, mask, value;
	} settings[] = {
		{CONFIG0, EN_OUT, 0}, /* Protected fields must be changed while disabled. */
		{CONFIG4, BIT(3) | BIT(2) | BRIDGE_MASK, BIT(3) | BIT(2)},
		{REG_CTRL0, BIT(4) | BIT(3), BIT(4) | BIT(3)},
		{TARGET, 0xFF, 0},
		{CONFIG3, BIT(4), BIT(4)},
		{RC_CTRL2, BIT(7) | BIT(6), scale << 6},
		{RC_CTRL3, 0xFF, inv_r},
		{CONFIG0, CLR_FLT, CLR_FLT},
		{CONFIG0, EN_OUT, EN_OUT},
	};

	k_mutex_lock(&data->lock, K_FOREVER);
	data->initialized = false;
	ret = supply_ready(config);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

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
	const struct drv8235_config *config = dev->config;
	uint8_t status, bridge;

	if (index > max_code(config)) {
		return -ERANGE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = supply_ready(config);

	if (ret != 0) {
		data->initialized = false;
		k_mutex_unlock(&data->lock);
		return ret;
	}
	ret = read_reg(dev, FAULT_STATUS, &status);

	if (ret == 0 && (status & NPOR) == 0U) {
		data->initialized = false;
	}
	if (ret == 0 && !data->initialized) {
		ret = -EAGAIN;
	}
	if (ret == 0) {
		ret = read_reg(dev, CONFIG4, &bridge);
	}
	if (ret == 0 &&
	    (((bridge & BRIDGE_MASK) == DRV8235_BRIDGE_FORWARD &&
	      mode == DRV8235_BRIDGE_REVERSE) ||
	     ((bridge & BRIDGE_MASK) == DRV8235_BRIDGE_REVERSE &&
	      mode == DRV8235_BRIDGE_FORWARD))) {
		/* Remove the old polarity before changing the target magnitude. */
		ret = update_reg(dev, CONFIG4, BRIDGE_MASK, DRV8235_BRIDGE_COAST);
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
	if (dev == NULL || !isfinite(throttle)) {
		return -EINVAL;
	}
	const struct drv8235_config *config = dev->config;
	float fraction = CLAMP(throttle, -1.0f, 1.0f) *
		((float)config->max_output_uv / TARGET_FULL_SCALE_UV);
	uint8_t index = (uint8_t)((fraction < 0 ? -fraction : fraction) * 255.0f);

	index = MIN(index, max_code(config));
	enum drv8235_bridge_mode mode = index == 0 ? DRV8235_BRIDGE_BRAKE :
		(throttle < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);

	return drive(dev, mode, index);
}

int drv8235_set_throttle_volts(const struct device *dev, float volts)
{
	if (dev == NULL || !isfinite(volts)) {
		return -EINVAL;
	}
	const struct drv8235_config *config = dev->config;
	float magnitude = volts < 0 ? -volts : volts;

	if (magnitude > config->max_output_uv / 1000000.0f) {
		return -ERANGE;
	}
	unsigned int index = (unsigned int)((volts < 0 ? -volts : volts) * INDEX_PER_VOLT + 0.5f);
	enum drv8235_bridge_mode mode = volts == 0 ? DRV8235_BRIDGE_BRAKE :
		(volts < 0 ? DRV8235_BRIDGE_REVERSE : DRV8235_BRIDGE_FORWARD);

	return drive(dev, mode, (uint8_t)MIN(index, max_code(config)));
}

int drv8235_set_throttle_raw(const struct device *dev, int16_t raw)
{
	int value = raw;

	if (dev == NULL) {
		return -EINVAL;
	}
	if ((value < 0 ? -value : value) > max_code(dev->config)) {
		return -ERANGE;
	}
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

		const struct drv8235_config *config = dev->config;

		*throttle = (raw < 0 ? -rounded : rounded) /
			((float)config->max_output_uv / TARGET_FULL_SCALE_UV);
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
		const struct drv8235_config *config = dev->config;

		/* Preserve the team's EPS /192 convention with the DT calibration. */
		*amps = sign * current * (config->imtr_full_scale_ua / 1000000.0f / 192.0f);
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
	return 0; /* The application powers VM and explicitly calls drv8235_init(). */
}

#define DRV8235_DEFINE(inst)                                                         \
	BUILD_ASSERT(DT_INST_PROP(inst, max_output_microvolt) > 0 &&                 \
		     DT_INST_PROP(inst, max_output_microvolt) <= TARGET_FULL_SCALE_UV); \
	BUILD_ASSERT(DT_INST_PROP(inst, coil_resistance_milliohm) > 0);               \
	BUILD_ASSERT(DT_INST_PROP(inst, imtr_full_scale_microamp) > 0);               \
	static struct drv8235_data drv8235_data_##inst;                              \
	static const struct drv8235_config drv8235_config_##inst = {                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                   \
		.supply = DEVICE_DT_GET(DT_INST_PHANDLE(inst, vm_supply)),            \
		.max_output_uv = DT_INST_PROP(inst, max_output_microvolt),           \
		.resistance_milliohm = DT_INST_PROP(inst, coil_resistance_milliohm),  \
		.imtr_full_scale_ua = DT_INST_PROP(inst, imtr_full_scale_microamp),   \
	};                                                                          \
	DEVICE_DT_INST_DEFINE(inst, drv8235_device_init, NULL, &drv8235_data_##inst,   \
			      &drv8235_config_##inst, POST_KERNEL,                     \
			      CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8235_DEFINE)
