/**
 * @brief Fake MAX17205 for tests.
 *
 * Holds the gauge's 16-bit registers and answers I2C transfers the way the real part does:
 * the first byte written sets the register pointer, further bytes are written LSB first,
 * and reads return registers LSB first. The pointer steps to the next register after each one.
 *
 * The real chip answers at two addresses: the main register block at 0x36 and shadow RAM at
 * 0x0B. The devicetree node only registers 0x36 with the emulated bus, so init registers 0x0B
 * by hand, and the transfer picks the register file from the address it was sent to.
 */

#define DT_DRV_COMPAT maxim_max17205

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>

#include <argus/drivers/max17205.h>

#include "max17205_emul.h"

#define MAX17205_EMUL_NUM_REGS 256

struct max17205_emul_data {
	uint16_t main[MAX17205_EMUL_NUM_REGS];
	uint16_t shadow[MAX17205_EMUL_NUM_REGS];
	uint8_t reg_ptr;
	bool nack;
	/* Puts this emulator on the bus a second time, at the shadow-RAM address */
	struct i2c_emul shadow_bus;
};

static uint16_t *max17205_emul_regs(struct max17205_emul_data *data, uint16_t addr)
{
	__ASSERT(addr == MAX17205_MAIN_I2C_ADDR || addr == MAX17205_SHADOW_I2C_ADDR,
		 "no MAX17205 register file at 0x%02x", addr);
	return addr == MAX17205_SHADOW_I2C_ADDR ? data->shadow : data->main;
}

void max17205_emul_set_reg(const struct emul *target, uint16_t addr, uint8_t reg, uint16_t val)
{
	max17205_emul_regs(target->data, addr)[reg] = val;
}

uint16_t max17205_emul_get_reg(const struct emul *target, uint16_t addr, uint8_t reg)
{
	return max17205_emul_regs(target->data, addr)[reg];
}

void max17205_emul_set_nack(const struct emul *target, bool nack)
{
	struct max17205_emul_data *data = target->data;

	data->nack = nack;
}

void max17205_emul_reset(const struct emul *target)
{
	struct max17205_emul_data *data = target->data;

	memset(data->main, 0, sizeof(data->main));
	memset(data->shadow, 0, sizeof(data->shadow));
	data->reg_ptr = 0;
	data->nack = false;
}

static int max17205_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				  int addr)
{
	struct max17205_emul_data *data = target->data;
	uint16_t *regs = max17205_emul_regs(data, addr);
	bool have_ptr = false;
	bool have_lsb = false;
	uint8_t lsb = 0;

	if (data->nack) {
		return -EIO;
	}

	/* The write state carries across messages, in case a write is split over several */
	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool is_read = (msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;

		for (uint32_t j = 0; j < msg->len; j++) {
			if (!is_read && !have_ptr) {
				data->reg_ptr = msg->buf[j];
				have_ptr = true;
				continue;
			}

			if (is_read) {
				uint16_t reg = regs[data->reg_ptr];

				msg->buf[j] = (j % 2 == 0) ? reg & 0xFF : reg >> 8;
				if (j % 2 == 1) {
					data->reg_ptr++;
				}
			} else if (!have_lsb) {
				lsb = msg->buf[j];
				have_lsb = true;
			} else {
				regs[data->reg_ptr++] = (msg->buf[j] << 8) | lsb;
				have_lsb = false;
			}
		}
	}

	return 0;
}

static const struct i2c_emul_api max17205_emul_api = {
	.transfer = max17205_emul_transfer,
};

/* Runs when the emulated I2C controller starts, before the driver's init */
static int max17205_emul_init(const struct emul *target, const struct device *parent)
{
	struct max17205_emul_data *data = target->data;

	max17205_emul_reset(target);

	data->shadow_bus.target = target;
	data->shadow_bus.api = &max17205_emul_api;
	data->shadow_bus.addr = MAX17205_SHADOW_I2C_ADDR;

	return i2c_emul_register(parent, &data->shadow_bus);
}

#define MAX17205_EMUL_DEFINE(inst)                                                           \
	static struct max17205_emul_data max17205_emul_data_##inst;                          \
	EMUL_DT_INST_DEFINE(inst, max17205_emul_init, &max17205_emul_data_##inst, NULL,      \
			    &max17205_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_EMUL_DEFINE)
