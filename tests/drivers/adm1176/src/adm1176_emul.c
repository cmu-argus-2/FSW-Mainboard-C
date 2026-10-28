/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adm1176

#include "adm1176_emul.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>

enum {
	ADM1176_COMMAND_V_CONT = 1U << 0,
	ADM1176_COMMAND_I_CONT = 1U << 2,
	ADM1176_COMMAND_STATUS_READ = 1U << 6,
	ADM1176_ALERT_ENABLE_HOT_SWAP = 1U << 2,

	ADM1176_REGISTER_ALERT_ENABLE = 0x81,
	ADM1176_REGISTER_ALERT_THRESHOLD = 0x82,
	ADM1176_REGISTER_CONTROL = 0x83,
};

struct adm1176_emul_data {
	uint8_t measurement[3];
	uint8_t status;
	uint8_t command;
	uint8_t alert_enable;
	uint8_t threshold;
	uint8_t control;
};

static void adm1176_emul_power_on_reset(struct adm1176_emul_data *data)
{
	memset(data, 0, sizeof(*data));
	data->alert_enable = ADM1176_ALERT_ENABLE_HOT_SWAP;
	data->threshold = UINT8_MAX;
}

void adm1176_emul_reset(const struct emul *target)
{
	struct adm1176_emul_data *data = target->data;

	adm1176_emul_power_on_reset(data);
	data->command = ADM1176_COMMAND_V_CONT | ADM1176_COMMAND_I_CONT;
}

void adm1176_emul_set_measurement(const struct emul *target,
				  const uint8_t measurement[3])
{
	struct adm1176_emul_data *data = target->data;

	memcpy(data->measurement, measurement, sizeof(data->measurement));
}

void adm1176_emul_set_status(const struct emul *target, uint8_t status)
{
	struct adm1176_emul_data *data = target->data;

	data->status = status;
}

uint8_t adm1176_emul_get_command(const struct emul *target)
{
	const struct adm1176_emul_data *data = target->data;

	return data->command;
}

uint8_t adm1176_emul_get_alert_enable(const struct emul *target)
{
	const struct adm1176_emul_data *data = target->data;

	return data->alert_enable;
}

uint8_t adm1176_emul_get_threshold(const struct emul *target)
{
	const struct adm1176_emul_data *data = target->data;

	return data->threshold;
}

uint8_t adm1176_emul_get_control(const struct emul *target)
{
	const struct adm1176_emul_data *data = target->data;

	return data->control;
}

static int adm1176_emul_write(struct adm1176_emul_data *data,
			      const struct i2c_msg *message)
{
	if (message->len == 1U) {
		data->command = message->buf[0];
		return 0;
	}
	if (message->len != 2U) {
		return -EINVAL;
	}

	switch (message->buf[0]) {
	case ADM1176_REGISTER_ALERT_ENABLE:
		data->alert_enable = message->buf[1];
		return 0;
	case ADM1176_REGISTER_ALERT_THRESHOLD:
		data->threshold = message->buf[1];
		return 0;
	case ADM1176_REGISTER_CONTROL:
		data->control = message->buf[1];
		return 0;
	default:
		return -EINVAL;
	}
}

static int adm1176_emul_read(struct adm1176_emul_data *data,
			     struct i2c_msg *message)
{
	if (message->len == 1U &&
	    (data->command & ADM1176_COMMAND_STATUS_READ) != 0U) {
		message->buf[0] = data->status;
		return 0;
	}
	if (message->len == sizeof(data->measurement)) {
		memcpy(message->buf, data->measurement, sizeof(data->measurement));
		return 0;
	}

	return -EINVAL;
}

static int adm1176_emul_transfer(const struct emul *target,
				 struct i2c_msg *messages, int num_messages,
				 int address)
{
	struct adm1176_emul_data *data = target->data;

	ARG_UNUSED(address);

	if (num_messages != 1) {
		return -EINVAL;
	}
	if ((messages[0].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		return adm1176_emul_read(data, &messages[0]);
	}

	return adm1176_emul_write(data, &messages[0]);
}

static const struct i2c_emul_api adm1176_emul_api = {
	.transfer = adm1176_emul_transfer,
};

static int adm1176_emul_init(const struct emul *target,
			     const struct device *parent)
{
	ARG_UNUSED(parent);
	adm1176_emul_power_on_reset(target->data);
	return 0;
}

#define ADM1176_EMUL_DEFINE(inst)                                             \
	static struct adm1176_emul_data adm1176_emul_data_##inst;               \
	EMUL_DT_INST_DEFINE(inst, adm1176_emul_init,                             \
			    &adm1176_emul_data_##inst, NULL,                     \
			    &adm1176_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ADM1176_EMUL_DEFINE)
