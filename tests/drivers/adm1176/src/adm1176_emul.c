/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT adi_adm1176

#include "adm1176_emul.h"

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>

struct adm1176_emul_state *adm1176_emul_state(const struct emul *emul)
{
	return emul->data;
}

void adm1176_emul_reset(const struct emul *emul)
{
	struct adm1176_emul_state *s = emul->data;

	memset(s, 0, sizeof(*s));
	s->alert_enable = 0x04;
	s->threshold = 0xFF;
}

static int transfer(const struct emul *emul, struct i2c_msg *msgs, int count, int addr)
{
	struct adm1176_emul_state *s = emul->data;
	struct i2c_msg *m = msgs;

	ARG_UNUSED(addr);
	if (s->error != 0) {
		if (s->skip == 0) {
			int error = s->error;

			s->error = 0;
			return error;
		}
		s->skip--;
	}
	if (count != 1) {
		return -EINVAL;
	}
	if (m->flags & I2C_MSG_READ) {
		bool voltage = (s->command & 0x03) != 0;
		bool current = (s->command & 0x0C) != 0;

		if (s->command & 0x40) {
			if (m->len != 1) {
				return -EIO;
			}
			m->buf[0] = s->status;
		} else if (voltage && current && m->len == 3) {
			memcpy(m->buf, s->sample, 3);
		} else if ((voltage || current) && m->len == 2) {
			m->buf[0] = s->sample[voltage ? 0 : 1];
			m->buf[1] = voltage ? (s->sample[2] & 0xF0) :
				((s->sample[2] & 0x0F) << 4);
		} else {
			return -EIO;
		}
		return 0;
	}
	if (m->len == 1) {
		s->command = m->buf[0];
		return 0;
	}
	if (m->len != 2) {
		return -EINVAL;
	}
	switch (m->buf[0]) {
	case 0x81:
		if (m->buf[1] & 0x10) {
			s->status &= ~0x2A; /* Clear latched alerts, with no active fault. */
		}
		s->alert_enable = m->buf[1] & ~0x10; /* CLEAR self-clears. */
		break;
	case 0x82:
		s->threshold = m->buf[1];
		break;
	case 0x83:
		s->control = m->buf[1];
		s->status = (s->status & ~0x10) | ((s->control & 1) << 4);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int init(const struct emul *emul, const struct device *parent)
{
	ARG_UNUSED(parent);
	adm1176_emul_reset(emul);
	return 0;
}

static const struct i2c_emul_api api = {.transfer = transfer};

#define ADM1176_EMUL_DEFINE(inst) \
	static struct adm1176_emul_state state_##inst; \
	EMUL_DT_INST_DEFINE(inst, init, &state_##inst, NULL, &api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ADM1176_EMUL_DEFINE)
