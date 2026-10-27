/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file drv8235.c
 * @brief Zephyr driver for the six Argus DRV8235 magnetorquer H-bridges.
 */

#define DT_DRV_COMPAT ti_drv8235

#include "argus/drivers/drv8235.h"

#include <errno.h>
#include <limits.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(drv8235, CONFIG_LOG_DEFAULT_LEVEL);

enum drv8235_register {
	DRV8235_REG_FAULT_STATUS = 0x00,
	DRV8235_REG_VOLTAGE_STATUS = 0x04,
	DRV8235_REG_CURRENT_STATUS = 0x05,
	DRV8235_REG_DUTY_STATUS = 0x06,
	DRV8235_REG_CONFIG0 = 0x09,
	DRV8235_REG_CONFIG3 = 0x0C,
	DRV8235_REG_CONFIG4 = 0x0D,
	DRV8235_REG_REG_CTRL0 = 0x0E,
	DRV8235_REG_TARGET = 0x0F,
	DRV8235_REG_RC_CTRL2 = 0x13,
	DRV8235_REG_RC_CTRL3 = 0x14,
};

enum drv8235_register_bits {
	DRV8235_CONFIG0_ENABLE_OUTPUT = BIT(7),
	DRV8235_CONFIG0_ENABLE_STALL = BIT(5),
	DRV8235_CONFIG0_CLEAR_FAULT = BIT(1),

	DRV8235_CONFIG3_INTERNAL_VREF = BIT(4),
	DRV8235_CONFIG3_AUTO_RETRY = BIT(1) | BIT(0),

	DRV8235_CONFIG4_STALL_REPORT = BIT(5),
	DRV8235_CONFIG4_PWM_MODE = BIT(3),
	DRV8235_CONFIG4_I2C_BRIDGE_CONTROL = BIT(2),
	DRV8235_CONFIG4_BRIDGE_MASK = BIT(1) | BIT(0),

	DRV8235_REG_CTRL0_REGULATION_MASK = BIT(4) | BIT(3),
	DRV8235_REG_CTRL0_VOLTAGE_REGULATION = BIT(4) | BIT(3),
	DRV8235_REG_CTRL0_SOFT_START = BIT(5),

	DRV8235_RC_CTRL2_INV_R_SCALE_MASK = BIT(7) | BIT(6),
};

struct drv8235_config {
	struct i2c_dt_spec bus;
	const struct device *vm_supply;
	uint32_t max_output_uv;
	uint32_t coil_resistance_milliohm;
	uint32_t imtr_full_scale_ua;
};

struct drv8235_data {
	struct k_mutex lock;
	bool configured;
};

static int drv8235_reg_read(const struct drv8235_config *config, uint8_t reg, uint8_t *value)
{
	return i2c_write_read_dt(&config->bus, &reg, 1U, value, 1U);
}

static int drv8235_reg_write(const struct drv8235_config *config, uint8_t reg, uint8_t value)
{
	const uint8_t message[2] = {reg, value};

	return i2c_write_dt(&config->bus, message, sizeof(message));
}

static int drv8235_reg_update(const struct drv8235_config *config, uint8_t reg, uint8_t mask,
			      uint8_t value)
{
	uint8_t old_value;
	int result = drv8235_reg_read(config, reg, &old_value);

	if (result != 0) {
		return result;
	}

	const uint8_t new_value = (old_value & (uint8_t)~mask) | (value & mask);

	if (new_value == old_value) {
		return 0;
	}

	return drv8235_reg_write(config, reg, new_value);
}

static int drv8235_set_bridge_locked(const struct drv8235_config *config,
				     enum drv8235_bridge_mode mode)
{
	return drv8235_reg_update(config, DRV8235_REG_CONFIG4, DRV8235_CONFIG4_BRIDGE_MASK,
				  (uint8_t)mode);
}

/* Observe the application's rail ownership; the driver never enables the rail. */
static int drv8235_check_supply(const struct drv8235_config *config)
{
	if (!device_is_ready(config->vm_supply)) {
		return -ENODEV;
	}

	return regulator_is_enabled(config->vm_supply) ? 0 : -EACCES;
}

/* Best-effort hardware safe state. The first error is preserved. */
static int drv8235_safe_locked(const struct drv8235_config *config, struct drv8235_data *data)
{
	int result = drv8235_check_supply(config);

	if (result != 0) {
		data->configured = false;
		return result;
	}
	result = drv8235_reg_update(config, DRV8235_REG_CONFIG0,
				    DRV8235_CONFIG0_ENABLE_OUTPUT, 0U);
	if (result == 0) {
		uint8_t config0;

		result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &config0);
		if (result == 0 && (config0 & DRV8235_CONFIG0_ENABLE_OUTPUT) != 0U) {
			result = -EIO;
		}
	}
	int step = drv8235_reg_write(config, DRV8235_REG_TARGET, 0U);

	if (result == 0) {
		result = step;
	}
	step = drv8235_set_bridge_locked(config, DRV8235_BRIDGE_COAST);
	if (result == 0) {
		result = step;
	}

	if (result != 0) {
		data->configured = false;
	}

	return result;
}

/* Read back safety-critical settings, including after a separately cleared NPOR. */
static int drv8235_verify_settings(const struct drv8235_config *config, uint8_t *bridge)
{
	uint8_t scale;
	uint8_t inv_r;
	int result = drv8235_calculate_inv_r(config->coil_resistance_milliohm, &scale, &inv_r);

	if (result != 0) {
		return result;
	}
	const struct {
		uint8_t reg;
		uint8_t mask;
		uint8_t expected;
	} settings[] = {
		{DRV8235_REG_CONFIG0, DRV8235_CONFIG0_ENABLE_STALL, 0U},
		{DRV8235_REG_CONFIG3,
		 DRV8235_CONFIG3_INTERNAL_VREF | DRV8235_CONFIG3_AUTO_RETRY,
		 DRV8235_CONFIG3_INTERNAL_VREF},
		{DRV8235_REG_CONFIG4,
		 DRV8235_CONFIG4_PWM_MODE | DRV8235_CONFIG4_I2C_BRIDGE_CONTROL |
			 DRV8235_CONFIG4_STALL_REPORT,
		 DRV8235_CONFIG4_PWM_MODE | DRV8235_CONFIG4_I2C_BRIDGE_CONTROL},
		{DRV8235_REG_REG_CTRL0,
		 DRV8235_REG_CTRL0_REGULATION_MASK | DRV8235_REG_CTRL0_SOFT_START,
		 DRV8235_REG_CTRL0_VOLTAGE_REGULATION},
		{DRV8235_REG_RC_CTRL2, DRV8235_RC_CTRL2_INV_R_SCALE_MASK, (uint8_t)(scale << 6)},
		{DRV8235_REG_RC_CTRL3, UINT8_MAX, inv_r},
	};

	for (size_t i = 0; i < ARRAY_SIZE(settings); ++i) {
		uint8_t value;

		result = drv8235_reg_read(config, settings[i].reg, &value);
		if (result != 0) {
			return result;
		}
		if ((value & settings[i].mask) != settings[i].expected) {
			return -EAGAIN;
		}
		if (bridge != NULL && settings[i].reg == DRV8235_REG_CONFIG4) {
			*bridge = value & DRV8235_CONFIG4_BRIDGE_MASK;
		}
	}

	/* A stale or externally changed target must not bypass the voltage limit. */
	uint8_t target;

	result = drv8235_reg_read(config, DRV8235_REG_TARGET, &target);
	if (result == 0 && (uint64_t)target * DRV8235_TARGET_FULL_SCALE_UV >
			  (uint64_t)config->max_output_uv * DRV8235_TARGET_FULL_SCALE_CODE) {
		return -EAGAIN;
	}
	return result;
}

static int drv8235_check_ready_locked(const struct drv8235_config *config,
				     struct drv8235_data *data, uint8_t *bridge)
{
	struct drv8235_faults faults;
	uint8_t raw;
	int result = drv8235_check_supply(config);

	if (result == 0 && !data->configured) {
		result = -EAGAIN;
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_FAULT_STATUS, &raw);
	}
	if (result == 0) {
		(void)drv8235_decode_faults(raw, &faults);
		if (faults.power_on_reset) {
			result = -EAGAIN;
		} else if (faults.any) {
			return -EIO;
		}
	}
	if (result == 0) {
		result = drv8235_verify_settings(config, bridge);
	}
	if (result != 0) {
		data->configured = false;
	}
	return result;
}

int drv8235_set_enabled(const struct device *dev, bool enabled)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!enabled) {
		result = drv8235_safe_locked(config, data);
	} else {
		result = drv8235_check_ready_locked(config, data, NULL);
		if (result == 0) {
			result = drv8235_reg_update(config, DRV8235_REG_CONFIG0,
						    DRV8235_CONFIG0_ENABLE_OUTPUT,
						    DRV8235_CONFIG0_ENABLE_OUTPUT);
			if (result != 0) {
				data->configured = false;
			}
		}
		if (result != 0) {
			(void)drv8235_safe_locked(config, data);
		}
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_is_enabled(const struct device *dev, bool *enabled)
{
	if (dev == NULL || enabled == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	uint8_t value;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = drv8235_check_supply(config);
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &value);
	}
	if (result == 0) {
		*enabled = (value & DRV8235_CONFIG0_ENABLE_OUTPUT) != 0U;
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_set_output_uv(const struct device *dev, int32_t voltage_uv)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	uint8_t code;
	enum drv8235_bridge_mode mode;
	int result = drv8235_voltage_to_code(voltage_uv, config->max_output_uv, &code, &mode);

	if (result != 0) {
		return result;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	uint8_t previous_mode;

	result = drv8235_check_ready_locked(config, data, &previous_mode);
	/* Soft start/stop is disabled. Same-direction updates keep driving. */
	if (result == 0 && previous_mode != mode && previous_mode != DRV8235_BRIDGE_COAST) {
		result = drv8235_set_bridge_locked(config, DRV8235_BRIDGE_COAST);
	}
	if (result == 0) {
		result = drv8235_reg_write(config, DRV8235_REG_TARGET, code);
	}
	if (result == 0) {
		result = drv8235_set_bridge_locked(config, mode);
	}
	if (result != 0) {
		data->configured = false;
		(void)drv8235_safe_locked(config, data);
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_coast(const struct device *dev)
{
	return drv8235_set_output_uv(dev, 0);
}

int drv8235_brake(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = drv8235_check_ready_locked(config, data, NULL);
	if (result == 0) {
		result = drv8235_reg_write(config, DRV8235_REG_TARGET, 0U);
	}
	if (result == 0) {
		result = drv8235_set_bridge_locked(config, DRV8235_BRIDGE_BRAKE);
	}
	if (result != 0) {
		data->configured = false;
		(void)drv8235_safe_locked(config, data);
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_get_telemetry(const struct device *dev, struct drv8235_telemetry *telemetry)
{
	if (dev == NULL || telemetry == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	uint8_t raw_voltage;
	uint8_t raw_current;
	uint8_t raw_duty;
	uint8_t config0;
	uint8_t config4;
	uint8_t raw_fault;
	struct drv8235_faults faults;
	struct drv8235_telemetry reading = {0};
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = drv8235_check_supply(config);
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_VOLTAGE_STATUS, &raw_voltage);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_CURRENT_STATUS, &raw_current);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_DUTY_STATUS, &raw_duty);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &config0);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_CONFIG4, &config4);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_FAULT_STATUS, &raw_fault);
	}

	if (result == 0) {
		(void)drv8235_decode_faults(raw_fault, &faults);
		if (faults.power_on_reset) {
			data->configured = false;
		}
		reading.raw_voltage = raw_voltage;
		reading.raw_current = raw_current;
		reading.raw_duty = raw_duty & DRV8235_DUTY_FULL_SCALE_CODE;
		reading.bridge_mode =
			(enum drv8235_bridge_mode)(config4 & DRV8235_CONFIG4_BRIDGE_MASK);
		reading.output_enabled = (config0 & DRV8235_CONFIG0_ENABLE_OUTPUT) != 0U;
		result = drv8235_voltage_from_code(raw_voltage, reading.bridge_mode,
						   &reading.voltage_uv);
	}
	/* Low-side brake still senses winding current. Hi-Z/disabled paths do not. */
	const uint8_t required_control = DRV8235_CONFIG4_PWM_MODE |
					 DRV8235_CONFIG4_I2C_BRIDGE_CONTROL;

	if (result == 0 && data->configured && !faults.any && reading.output_enabled &&
	    reading.bridge_mode != DRV8235_BRIDGE_COAST &&
	    (config4 & required_control) == required_control &&
	    raw_current <= DRV8235_IMTR_FULL_SCALE_CODE) {
		result = drv8235_current_from_code(raw_current, config->imtr_full_scale_ua,
						   &reading.current_ua);
		reading.current_valid = result == 0;
	}
	if (result == 0) {
		result = drv8235_duty_from_code(reading.raw_duty, &reading.duty_permille);
	}
	if (result == 0) {
		*telemetry = reading;
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_get_faults(const struct device *dev, struct drv8235_faults *faults)
{
	if (dev == NULL || faults == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	uint8_t raw;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = drv8235_check_supply(config);
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_FAULT_STATUS, &raw);
	}
	if (result == 0) {
		(void)drv8235_decode_faults(raw, faults);
		if (faults->power_on_reset) {
			data->configured = false;
		}
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_clear_faults(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	uint8_t config0;
	uint8_t fault;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	result = drv8235_check_supply(config);
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_FAULT_STATUS, &fault);
	}
	if (result == 0 && (fault & BIT(1)) == 0U) {
		data->configured = false;
	}
	/* Do not clear a latch while EN_OUT can still restart the coil. */
	if (result == 0) {
		result = drv8235_safe_locked(config, data);
	} else {
		(void)drv8235_safe_locked(config, data);
	}
	if (result == 0) {
		result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &config0);
	}
	if (result == 0) {
		result = drv8235_reg_write(config, DRV8235_REG_CONFIG0,
					   config0 | DRV8235_CONFIG0_CLEAR_FAULT);
	}
	if (result != 0) {
		data->configured = false;
	}
	k_mutex_unlock(&data->lock);

	return result;
}

int drv8235_get_limits(const struct device *dev, struct drv8235_limits *limits)
{
	if (dev == NULL || limits == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;

	limits->max_output_uv = config->max_output_uv;
	limits->coil_resistance_milliohm = config->coil_resistance_milliohm;
	limits->imtr_full_scale_ua = config->imtr_full_scale_ua;
	return 0;
}

static int drv8235_configure_locked(const struct drv8235_config *config, struct drv8235_data *data)
{
	uint8_t config0;
	uint8_t config3;
	uint8_t config4;
	uint8_t reg_ctrl0;
	uint8_t rc_ctrl2;
	uint8_t scale_code;
	uint8_t inv_r;
	int result;

	data->configured = false;
	if (config->max_output_uv == 0U || config->max_output_uv > DRV8235_TARGET_FULL_SCALE_UV ||
	    config->imtr_full_scale_ua == 0U || config->imtr_full_scale_ua > INT32_MAX) {
		LOG_ERR("invalid devicetree limits");
		return -EINVAL;
	}

	result = drv8235_calculate_inv_r(config->coil_resistance_milliohm, &scale_code, &inv_r);
	if (result != 0) {
		LOG_ERR("coil resistance cannot be represented");
		return result;
	}

	result = drv8235_check_supply(config);
	if (result != 0) {
		return result;
	}

	/* A successful read also proves that a device ACKs this I2C address. */
	result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &config0);
	if (result != 0) {
		return result;
	}
	/* A coil has no mechanical stall; retain overvoltage protection. */
	config0 &= (uint8_t)~(DRV8235_CONFIG0_ENABLE_OUTPUT | DRV8235_CONFIG0_ENABLE_STALL);
	result = drv8235_reg_write(config, DRV8235_REG_CONFIG0, config0);
	if (result != 0) {
		return result;
	}
	/* Do not touch EN_OUT-restricted settings until shutdown is confirmed. */
	uint8_t readback;

	result = drv8235_reg_read(config, DRV8235_REG_CONFIG0, &readback);
	if (result != 0) {
		return result;
	}
	if ((readback & DRV8235_CONFIG0_ENABLE_OUTPUT) != 0U) {
		return -EIO;
	}
	result = drv8235_reg_write(config, DRV8235_REG_TARGET, 0U);
	if (result != 0) {
		return result;
	}

	result = drv8235_reg_read(config, DRV8235_REG_CONFIG4, &config4);
	if (result != 0) {
		return result;
	}
	config4 &= (uint8_t)~(DRV8235_CONFIG4_BRIDGE_MASK | DRV8235_CONFIG4_STALL_REPORT);
	config4 |= DRV8235_CONFIG4_PWM_MODE | DRV8235_CONFIG4_I2C_BRIDGE_CONTROL;
	result = drv8235_reg_write(config, DRV8235_REG_CONFIG4, config4);
	if (result != 0) {
		return result;
	}

	result = drv8235_reg_read(config, DRV8235_REG_REG_CTRL0, &reg_ctrl0);
	if (result != 0) {
		return result;
	}
	/* Deterministic coil commands: no motor soft-start/soft-stop ramp. */
	reg_ctrl0 = (reg_ctrl0 & (uint8_t)~(DRV8235_REG_CTRL0_REGULATION_MASK |
						 DRV8235_REG_CTRL0_SOFT_START)) |
		    DRV8235_REG_CTRL0_VOLTAGE_REGULATION;
	result = drv8235_reg_write(config, DRV8235_REG_REG_CTRL0, reg_ctrl0);
	if (result != 0) {
		return result;
	}

	result = drv8235_reg_read(config, DRV8235_REG_CONFIG3, &config3);
	if (result != 0) {
		return result;
	}
	/* OCP and TSD latch off until an explicit, disabled-output fault clear. */
	config3 = (config3 & (uint8_t)~DRV8235_CONFIG3_AUTO_RETRY) |
		  DRV8235_CONFIG3_INTERNAL_VREF;
	result = drv8235_reg_write(config, DRV8235_REG_CONFIG3, config3);
	if (result != 0) {
		return result;
	}

	result = drv8235_reg_read(config, DRV8235_REG_RC_CTRL2, &rc_ctrl2);
	if (result != 0) {
		return result;
	}
	rc_ctrl2 = (rc_ctrl2 & (uint8_t)~DRV8235_RC_CTRL2_INV_R_SCALE_MASK) |
		   (uint8_t)(scale_code << 6);
	result = drv8235_reg_write(config, DRV8235_REG_RC_CTRL2, rc_ctrl2);
	if (result != 0) {
		return result;
	}
	result = drv8235_reg_write(config, DRV8235_REG_RC_CTRL3, inv_r);
	if (result != 0) {
		return result;
	}

	result = drv8235_reg_write(config, DRV8235_REG_CONFIG0,
				   config0 | DRV8235_CONFIG0_CLEAR_FAULT);
	if (result != 0) {
		return result;
	}

	result = drv8235_verify_settings(config, NULL);
	if (result == 0) {
		struct drv8235_faults faults;
		uint8_t raw;

		result = drv8235_reg_read(config, DRV8235_REG_FAULT_STATUS, &raw);
		if (result == 0) {
			(void)drv8235_decode_faults(raw, &faults);
			result = faults.any ? -EIO : 0;
		}
	}
	data->configured = result == 0;
	return result;
}

int drv8235_reconfigure(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	int result;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->configured = false;
	result = i2c_is_ready_dt(&config->bus) ? drv8235_configure_locked(config, data) : -ENODEV;
	if (result != 0) {
		(void)drv8235_safe_locked(config, data);
	}
	k_mutex_unlock(&data->lock);

	return result;
}

static __maybe_unused int drv8235_init(const struct device *dev)
{
	const struct drv8235_config *config = dev->config;
	struct drv8235_data *data = dev->data;
	int result;

	k_mutex_init(&data->lock);
	data->configured = false;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	result = drv8235_reconfigure(dev);
	if (result == 0) {
		LOG_INF("ready at I2C 0x%02x; output disabled", config->bus.addr);
	}

	return result;
}

#define DRV8235_DEFINE(inst)                                                                       \
	static struct drv8235_data drv8235_data_##inst;                                            \
	static const struct drv8235_config drv8235_config_##inst = {                               \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.vm_supply = DEVICE_DT_GET(DT_INST_PHANDLE(inst, vm_supply)),                      \
		.max_output_uv = DT_INST_PROP(inst, max_output_microvolt),                         \
		.coil_resistance_milliohm = DT_INST_PROP(inst, coil_resistance_milliohm),          \
		.imtr_full_scale_ua = DT_INST_PROP(inst, imtr_full_scale_microamp),                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, drv8235_init, NULL, &drv8235_data_##inst,                      \
			      &drv8235_config_##inst, POST_KERNEL,                                 \
			      CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8235_DEFINE)
