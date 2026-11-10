/* SPDX-License-Identifier: Apache-2.0 */
#ifndef DRV8235_EMUL_H_
#define DRV8235_EMUL_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/emul.h>

struct drv8235_emul_write {
	uint8_t reg;
	uint8_t value;
};

void drv8235_emul_reset(const struct emul *emul);
void drv8235_emul_set_reg(const struct emul *emul, uint8_t reg, uint8_t value);
uint8_t drv8235_emul_get_reg(const struct emul *emul, uint8_t reg);
void drv8235_emul_set_error(const struct emul *emul, int error);
void drv8235_emul_fail_next_write(const struct emul *emul, uint8_t reg, int error);
void drv8235_emul_clear_writes(const struct emul *emul);
size_t drv8235_emul_write_count(const struct emul *emul);
const struct drv8235_emul_write *drv8235_emul_get_write(const struct emul *emul, size_t index);

#endif
