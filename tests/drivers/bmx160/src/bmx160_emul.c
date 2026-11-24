/**
 * @brief Fake BMX160 for tests.
 *
 * Holds the BMX160's 8-bit registers and the BMM150's behind it. The first byte written in
 * a transfer sets the register pointer; following bytes are written or read one register at
 * a time, stepping the pointer. On top of that it models what the driver depends on:
 *
 * - CMD: soft reset restores register defaults; power-mode commands set PMU_STATUS.
 * - MAG_IF in manual mode: writing MAG_IF[2] copies 1/2/6/8 BMM150 registers into
 *   DATA_MAG; writing MAG_IF[3] writes MAG_IF[4] into a BMM150 register. Transfers
 *   finish at once, so STATUS.mag_man_op is never set.
 *
 * In data mode, tests put magnetometer samples straight into DATA_MAG.
 */

#define DT_DRV_COMPAT bosch_bmx160

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "bmx160_emul.h"

#define REG_MAG_IF3 0x4E
#define REG_MAG_IF4 0x4F
#define MAG_IF1_MANUAL BIT(7)
#define MAX_CMDS 32

struct bmx160_emul_data {
	uint8_t regs[BMX160_EMUL_NUM_REGS];
	uint8_t bmm150[0x80];
	uint8_t reg_ptr;
	uint8_t chip_id;
	uint8_t forced_err;
	uint8_t ignored_reg;
	bool fail;
	uint8_t cmds[MAX_CMDS];
	size_t num_cmds;
};

/* Power-on values of the registers the driver writes or reads */
static void bmx160_emul_reset(struct bmx160_emul_data *data)
{
	memset(data->regs, 0, sizeof(data->regs));
	data->regs[BMX160_EMUL_REG_CHIP_ID] = data->chip_id;
	data->regs[BMX160_EMUL_REG_ACC_CONF] = 0x28;
	data->regs[BMX160_EMUL_REG_ACC_RANGE] = 0x03;
	data->regs[BMX160_EMUL_REG_GYR_CONF] = 0x28;
	data->regs[BMX160_EMUL_REG_MAG_CONF] = 0x0B;
	data->regs[BMX160_EMUL_REG_MAG_IF1] = 0x80;
}

static void bmx160_emul_cmd(struct bmx160_emul_data *data, uint8_t cmd)
{
	uint8_t *pmu = &data->regs[BMX160_EMUL_REG_PMU_STATUS];

	if (data->num_cmds < MAX_CMDS) {
		data->cmds[data->num_cmds++] = cmd;
	}

	if (cmd == 0xB6) {
		bmx160_emul_reset(data);
	} else if ((cmd & 0xFC) == 0x10) { /* accel: 0 suspend, 1 normal, 2 low power */
		*pmu = (*pmu & ~0x30) | ((cmd & 0x03) << 4);
	} else if ((cmd & 0xFC) == 0x14) { /* gyro: 0 suspend, 1 normal, 3 fast start-up */
		*pmu = (*pmu & ~0x0C) | ((cmd & 0x03) << 2);
	} else if ((cmd & 0xFC) == 0x18) { /* mag interface */
		*pmu = (*pmu & ~0x03) | (cmd & 0x03);
	}
}

static void bmx160_emul_write(struct bmx160_emul_data *data, uint8_t reg, uint8_t val)
{
	static const uint8_t burst_len[] = {1, 2, 6, 8};
	bool manual = data->regs[BMX160_EMUL_REG_MAG_IF1] & MAG_IF1_MANUAL;

	if (reg == data->ignored_reg) {
		return;
	}
	if (reg == BMX160_EMUL_REG_CMD) {
		bmx160_emul_cmd(data, val);
		return;
	}

	data->regs[reg] = val;

	if (manual && reg == BMX160_EMUL_REG_MAG_IF2) {
		uint8_t len = burst_len[data->regs[BMX160_EMUL_REG_MAG_IF1] & 0x03];

		for (uint8_t i = 0; i < len; i++) {
			data->regs[BMX160_EMUL_REG_DATA_MAG + i] = data->bmm150[(val + i) & 0x7F];
		}
	} else if (manual && reg == REG_MAG_IF3) {
		data->bmm150[val & 0x7F] = data->regs[REG_MAG_IF4];
	}
}

static int bmx160_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				int addr)
{
	struct bmx160_emul_data *data = target->data;
	bool have_ptr = false;

	ARG_UNUSED(addr);

	if (data->fail) {
		return -EIO;
	}

	/* i2c_burst_read_dt() sends the register, then reads, as two messages of one transfer */
	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool is_read = (msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;

		for (uint32_t j = 0; j < msg->len; j++) {
			if (!is_read && !have_ptr) {
				data->reg_ptr = msg->buf[j];
				have_ptr = true;
				continue;
			}
			if (data->reg_ptr >= BMX160_EMUL_NUM_REGS) {
				return -EIO;
			}
			if (is_read) {
				bool err = data->reg_ptr == BMX160_EMUL_REG_ERR && data->forced_err;

				msg->buf[j] = err ? data->forced_err : data->regs[data->reg_ptr];
			} else {
				bmx160_emul_write(data, data->reg_ptr, msg->buf[j]);
			}
			data->reg_ptr++;
		}
	}

	return 0;
}

void bmx160_emul_set_reg(const struct emul *target, uint8_t reg, uint8_t val)
{
	struct bmx160_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg < BMX160_EMUL_NUM_REGS);
	data->regs[reg] = val;
}

uint8_t bmx160_emul_get_reg(const struct emul *target, uint8_t reg)
{
	struct bmx160_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg < BMX160_EMUL_NUM_REGS);
	return data->regs[reg];
}

void bmx160_emul_set_reg16(const struct emul *target, uint8_t reg, int16_t val)
{
	struct bmx160_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg + 1 < BMX160_EMUL_NUM_REGS);
	sys_put_le16((uint16_t)val, &data->regs[reg]);
}

void bmx160_emul_set_bmm150_reg(const struct emul *target, uint8_t reg, uint8_t val)
{
	struct bmx160_emul_data *data = target->data;

	data->bmm150[reg & 0x7F] = val;
}

uint8_t bmx160_emul_get_bmm150_reg(const struct emul *target, uint8_t reg)
{
	struct bmx160_emul_data *data = target->data;

	return data->bmm150[reg & 0x7F];
}

void bmx160_emul_set_chip_id(const struct emul *target, uint8_t id)
{
	struct bmx160_emul_data *data = target->data;

	data->chip_id = id;
	data->regs[BMX160_EMUL_REG_CHIP_ID] = id;
}

void bmx160_emul_force_err(const struct emul *target, uint8_t err)
{
	struct bmx160_emul_data *data = target->data;

	data->forced_err = err;
}

void bmx160_emul_ignore_writes(const struct emul *target, uint8_t reg)
{
	struct bmx160_emul_data *data = target->data;

	data->ignored_reg = reg;
}

void bmx160_emul_set_fail(const struct emul *target, bool fail)
{
	struct bmx160_emul_data *data = target->data;

	data->fail = fail;
}

size_t bmx160_emul_get_cmds(const struct emul *target, const uint8_t **cmds)
{
	struct bmx160_emul_data *data = target->data;

	*cmds = data->cmds;
	return data->num_cmds;
}

static const struct i2c_emul_api bmx160_emul_api = {
	.transfer = bmx160_emul_transfer,
};

/* Little-endian 16-bit trim value into two BMM150 registers */
static void put_trim16(struct bmx160_emul_data *data, uint8_t reg, uint16_t val)
{
	sys_put_le16(val, &data->bmm150[reg]);
}

/* Runs when the emulated I2C controller starts, before the driver's init */
static int bmx160_emul_init(const struct emul *target, const struct device *parent)
{
	struct bmx160_emul_data *data = target->data;

	ARG_UNUSED(parent);

	memset(data, 0, sizeof(*data));
	data->chip_id = BMX160_EMUL_CHIP_ID;
	data->ignored_reg = BMX160_EMUL_NO_REG;
	bmx160_emul_reset(data);

	data->bmm150[0x5D] = (uint8_t)BMX160_EMUL_TRIM_X1;
	data->bmm150[0x5E] = (uint8_t)BMX160_EMUL_TRIM_Y1;
	put_trim16(data, 0x62, (uint16_t)BMX160_EMUL_TRIM_Z4);
	data->bmm150[0x64] = (uint8_t)BMX160_EMUL_TRIM_X2;
	data->bmm150[0x65] = (uint8_t)BMX160_EMUL_TRIM_Y2;
	put_trim16(data, 0x68, (uint16_t)BMX160_EMUL_TRIM_Z2);
	put_trim16(data, 0x6A, BMX160_EMUL_TRIM_Z1);
	put_trim16(data, 0x6C, BMX160_EMUL_TRIM_XYZ1 | 0x8000);
	put_trim16(data, 0x6E, (uint16_t)BMX160_EMUL_TRIM_Z3);
	data->bmm150[0x70] = (uint8_t)BMX160_EMUL_TRIM_XY2;
	data->bmm150[0x71] = BMX160_EMUL_TRIM_XY1;

	return 0;
}

#define BMX160_EMUL_DEFINE(inst)                                                             \
	static struct bmx160_emul_data bmx160_emul_data_##inst;                              \
	EMUL_DT_INST_DEFINE(inst, bmx160_emul_init, &bmx160_emul_data_##inst, NULL,          \
			    &bmx160_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(BMX160_EMUL_DEFINE)
