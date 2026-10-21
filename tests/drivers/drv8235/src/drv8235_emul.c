/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_drv8235

#include "drv8235_emul.h"

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/util.h>

enum {
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
	DRV8235_REGISTER_COUNT = 0x1A,

	DRV8235_CONFIG0_CLEAR_FAULT = BIT(1),
};

struct drv8235_emul_data {
	uint8_t registers[DRV8235_REGISTER_COUNT];
	uint8_t pointer;
	int error;
};

static void drv8235_emul_power_on_reset(struct drv8235_emul_data *data)
{
	memset(data, 0, sizeof(*data));
	data->registers[DRV8235_REG_CONFIG0] = 0x60U;
	data->registers[DRV8235_REG_CONFIG3] = 0x63U;
	data->registers[DRV8235_REG_CONFIG4] = 0x38U;
	data->registers[DRV8235_REG_REG_CTRL0] = 0x27U;
	data->registers[DRV8235_REG_TARGET] = 0xFFU;
	data->registers[DRV8235_REG_RC_CTRL2] = 0x73U;
}

void drv8235_emul_reset(const struct emul *target)
{
	struct drv8235_emul_data *data = target->data;

	/* State produced by a successful driver init for a 25-ohm coil. */
	drv8235_emul_power_on_reset(data);
	data->registers[DRV8235_REG_CONFIG0] = 0x60U;
	data->registers[DRV8235_REG_CONFIG3] = 0x73U;
	data->registers[DRV8235_REG_CONFIG4] = 0x3CU;
	data->registers[DRV8235_REG_REG_CTRL0] = 0x3FU;
	data->registers[DRV8235_REG_TARGET] = 0U;
	data->registers[DRV8235_REG_RC_CTRL2] = 0xB3U;
	data->registers[DRV8235_REG_RC_CTRL3] = 41U;
}

void drv8235_emul_power_cycle(const struct emul *target)
{
	drv8235_emul_power_on_reset(target->data);
}

void drv8235_emul_set_status(const struct emul *target, uint8_t voltage, uint8_t current,
			     uint8_t duty)
{
	struct drv8235_emul_data *data = target->data;

	data->registers[DRV8235_REG_VOLTAGE_STATUS] = voltage;
	data->registers[DRV8235_REG_CURRENT_STATUS] = current;
	data->registers[DRV8235_REG_DUTY_STATUS] = duty;
}

void drv8235_emul_set_faults(const struct emul *target, uint8_t faults)
{
	struct drv8235_emul_data *data = target->data;

	data->registers[DRV8235_REG_FAULT_STATUS] = faults;
}

void drv8235_emul_set_error(const struct emul *target, int error)
{
	struct drv8235_emul_data *data = target->data;

	data->error = error;
}

uint8_t drv8235_emul_get_reg(const struct emul *target, uint8_t reg)
{
	const struct drv8235_emul_data *data = target->data;

	return reg < ARRAY_SIZE(data->registers) ? data->registers[reg] : 0U;
}

static int drv8235_emul_transfer(const struct emul *target, struct i2c_msg *messages,
				 int num_messages, int address)
{
	struct drv8235_emul_data *data = target->data;

	ARG_UNUSED(address);
	if (data->error != 0) {
		return data->error;
	}

	if (num_messages == 2 && messages[0].len == 1U &&
	    (messages[0].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE &&
	    (messages[1].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		const uint8_t reg = messages[0].buf[0];

		if (messages[1].len != 1U || reg >= ARRAY_SIZE(data->registers)) {
			return -EINVAL;
		}
		messages[1].buf[0] = data->registers[reg];
		data->pointer = reg;
		return 0;
	}

	if (num_messages != 1 || (messages[0].flags & I2C_MSG_RW_MASK) != I2C_MSG_WRITE) {
		return -EINVAL;
	}
	if (messages[0].len == 1U) {
		data->pointer = messages[0].buf[0];
		return 0;
	}
	if (messages[0].len != 2U || messages[0].buf[0] >= ARRAY_SIZE(data->registers)) {
		return -EINVAL;
	}

	const uint8_t reg = messages[0].buf[0];
	uint8_t value = messages[0].buf[1];

	if (reg == DRV8235_REG_CONFIG0 && (value & DRV8235_CONFIG0_CLEAR_FAULT) != 0U) {
		data->registers[DRV8235_REG_FAULT_STATUS] = 0U;
		value &= (uint8_t)~DRV8235_CONFIG0_CLEAR_FAULT;
	}
	data->registers[reg] = value;
	data->pointer = reg;
	return 0;
}

static const struct i2c_emul_api drv8235_emul_api = {
	.transfer = drv8235_emul_transfer,
};

static int drv8235_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	drv8235_emul_power_on_reset(target->data);
	return 0;
}

#define DRV8235_EMUL_DEFINE(inst)                                                                  \
	static struct drv8235_emul_data drv8235_emul_data_##inst;                                  \
	EMUL_DT_INST_DEFINE(inst, drv8235_emul_init, &drv8235_emul_data_##inst, NULL,              \
			    &drv8235_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(DRV8235_EMUL_DEFINE)
