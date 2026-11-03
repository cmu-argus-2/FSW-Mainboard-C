/**
 * @brief Fake OPT4003 for tests.
 *
 * Holds the sensor's 16-bit registers and answers I2C transfers the way the real part does:
 * the first byte written sets the register pointer, further bytes are written MSB first,
 * and reads return registers MSB first. The pointer steps to the next register after each
 * one, so the driver's 4-byte result read gets RESULT_MSB_CH0 then RESULT_LSB_CH0.
 */

#define DT_DRV_COMPAT ti_opt4003

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>

#include "opt4003_emul.h"

struct opt4003_emul_data {
	uint16_t regs[OPT4003_EMUL_NUM_REGS];
	uint8_t reg_ptr;
};

void opt4003_emul_set_reg(const struct emul *target, uint8_t reg, uint16_t val)
{
	struct opt4003_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg < OPT4003_EMUL_NUM_REGS);
	data->regs[reg] = val;
}

uint16_t opt4003_emul_get_reg(const struct emul *target, uint8_t reg)
{
	struct opt4003_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg < OPT4003_EMUL_NUM_REGS);
	return data->regs[reg];
}

static int opt4003_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				 int addr)
{
	struct opt4003_emul_data *data = target->data;
	bool have_ptr = false;
	bool have_msb = false;
	uint8_t msb = 0;

	ARG_UNUSED(addr);

	/* i2c_burst_write_dt() sends the register and the value as two write messages, so the
	 * write state carries across messages within one transfer.
	 */
	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool is_read = (msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;

		for (uint32_t j = 0; j < msg->len; j++) {
			if (!is_read && !have_ptr) {
				data->reg_ptr = msg->buf[j];
				have_ptr = true;
				continue;
			}

			if (data->reg_ptr >= OPT4003_EMUL_NUM_REGS) {
				return -EIO;
			}

			if (is_read) {
				uint16_t reg = data->regs[data->reg_ptr];

				msg->buf[j] = (j % 2 == 0) ? reg >> 8 : reg & 0xFF;
				if (j % 2 == 1) {
					data->reg_ptr++;
				}
			} else if (!have_msb) {
				msb = msg->buf[j];
				have_msb = true;
			} else {
				data->regs[data->reg_ptr++] = (msb << 8) | msg->buf[j];
				have_msb = false;
			}
		}
	}

	return 0;
}

static const struct i2c_emul_api opt4003_emul_api = {
	.transfer = opt4003_emul_transfer,
};

/* Runs when the emulated I2C controller starts, before the driver's init */
static int opt4003_emul_init(const struct emul *target, const struct device *parent)
{
	struct opt4003_emul_data *data = target->data;

	ARG_UNUSED(parent);

	memset(data->regs, 0, sizeof(data->regs));
	data->regs[OPT4003_EMUL_REG_DEVICE_ID] = OPT4003_EMUL_DEVICE_ID;
	data->reg_ptr = 0;

	return 0;
}

#define OPT4003_EMUL_DEFINE(inst)                                                            \
	static struct opt4003_emul_data opt4003_emul_data_##inst;                            \
	EMUL_DT_INST_DEFINE(inst, opt4003_emul_init, &opt4003_emul_data_##inst, NULL,        \
			    &opt4003_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(OPT4003_EMUL_DEFINE)
