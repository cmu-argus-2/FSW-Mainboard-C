/* SPDX-License-Identifier: Apache-2.0 */
#define DT_DRV_COMPAT ti_drv8235

#include "drv8235_emul.h"

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/util.h>

struct drv8235_emul_data {
	uint8_t regs[0x1A];
	int error;
	int write_error;
	uint8_t fail_reg;
	struct drv8235_emul_write writes[128];
	size_t write_count;
};

void drv8235_emul_reset(const struct emul *emul)
{
	struct drv8235_emul_data *data = emul->data;

	memset(data, 0, sizeof(*data));
	/* Power-on defaults, including NPOR=0 and EN_OUT=0. */
	data->regs[0x09] = 0x60;
	data->regs[0x0C] = 0x63;
	data->regs[0x0D] = 0x38;
	data->regs[0x0E] = 0x27;
	data->regs[0x0F] = 0xFF;
	data->regs[0x13] = 0x73;
}

void drv8235_emul_set_reg(const struct emul *emul, uint8_t reg, uint8_t value)
{
	struct drv8235_emul_data *data = emul->data;

	data->regs[reg] = value;
}

uint8_t drv8235_emul_get_reg(const struct emul *emul, uint8_t reg)
{
	const struct drv8235_emul_data *data = emul->data;

	return data->regs[reg];
}

void drv8235_emul_set_error(const struct emul *emul, int error)
{
	struct drv8235_emul_data *data = emul->data;

	data->error = error;
}

void drv8235_emul_fail_next_write(const struct emul *emul, uint8_t reg, int error)
{
	struct drv8235_emul_data *data = emul->data;

	data->fail_reg = reg;
	data->write_error = error;
}

void drv8235_emul_clear_writes(const struct emul *emul)
{
	struct drv8235_emul_data *data = emul->data;

	data->write_count = 0;
}

size_t drv8235_emul_write_count(const struct emul *emul)
{
	const struct drv8235_emul_data *data = emul->data;

	return data->write_count;
}

const struct drv8235_emul_write *drv8235_emul_get_write(const struct emul *emul, size_t index)
{
	const struct drv8235_emul_data *data = emul->data;

	return index < data->write_count ? &data->writes[index] : NULL;
}

static int transfer(const struct emul *emul, struct i2c_msg *msgs, int count, int addr)
{
	struct drv8235_emul_data *data = emul->data;

	ARG_UNUSED(addr);
	if (data->error != 0) {
		return data->error;
	}
	if (count == 2 && msgs[0].len == 1 && msgs[1].len == 1 &&
	    (msgs[0].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE &&
	    (msgs[1].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ &&
	    msgs[0].buf[0] < ARRAY_SIZE(data->regs)) {
		msgs[1].buf[0] = data->regs[msgs[0].buf[0]];
		return 0;
	}
	if (count != 1 || msgs[0].len != 2 ||
	    (msgs[0].flags & I2C_MSG_RW_MASK) != I2C_MSG_WRITE) {
		return -EINVAL;
	}
	uint8_t reg = msgs[0].buf[0];
	uint8_t value = msgs[0].buf[1];

	if (reg < 0x09 || reg >= ARRAY_SIZE(data->regs)) {
		return -EINVAL;
	}
	if (data->write_error != 0 && reg == data->fail_reg) {
		int ret = data->write_error;

		data->write_error = 0;
		return ret;
	}
	/* Enforce protected fields so reinitialization must first disable EN_OUT. */
	uint8_t protected_mask = reg == 0x0C ? 0xFF : reg == 0x0D ? 0x0C :
				 reg == 0x0E ? 0x1C : 0;

	if ((data->regs[0x09] & BIT(7)) &&
	    ((value ^ data->regs[reg]) & protected_mask)) {
		return -EACCES;
	}
	if (data->write_count == ARRAY_SIZE(data->writes)) {
		return -ENOMEM;
	}
	data->writes[data->write_count++] = (struct drv8235_emul_write){reg, value};
	if (reg == 0x09 && (value & BIT(1))) {
		data->regs[0x00] = BIT(1); /* CLR_FLT latches NPOR high. */
		value &= (uint8_t)~BIT(1);
	}
	data->regs[reg] = value;
	return 0;
}

static const struct i2c_emul_api api = {.transfer = transfer};

static int emulator_init(const struct emul *emul, const struct device *parent)
{
	ARG_UNUSED(parent);
	drv8235_emul_reset(emul);
	return 0;
}

#define DRV8235_EMUL_DEFINE(inst)                                                   \
	static struct drv8235_emul_data data_##inst;                                \
	EMUL_DT_INST_DEFINE(inst, emulator_init, &data_##inst, NULL, &api, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8235_EMUL_DEFINE)
