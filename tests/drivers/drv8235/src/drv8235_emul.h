/* SPDX-License-Identifier: Apache-2.0 */

#ifndef DRV8235_EMUL_H_
#define DRV8235_EMUL_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <zephyr/drivers/emul.h>

struct drv8235_emul_write {
	uint8_t reg;
	uint8_t value;
	bool output_was_enabled;
};

void drv8235_emul_power_cycle(const struct emul *target);
void drv8235_emul_set_status(const struct emul *target, uint8_t voltage, uint8_t current,
			     uint8_t duty);
void drv8235_emul_set_faults(const struct emul *target, uint8_t faults);
void drv8235_emul_set_error(const struct emul *target, int error);
void drv8235_emul_fail_next_write(const struct emul *target, uint8_t reg, int error);
void drv8235_emul_ignore_next_write(const struct emul *target, uint8_t reg);
void drv8235_emul_set_reg(const struct emul *target, uint8_t reg, uint8_t value);
void drv8235_emul_set_active_faults(const struct emul *target, uint8_t faults);
uint8_t drv8235_emul_get_reg(const struct emul *target, uint8_t reg);
void drv8235_emul_clear_writes(const struct emul *target);
size_t drv8235_emul_write_count(const struct emul *target);
const struct drv8235_emul_write *drv8235_emul_get_write(const struct emul *target, size_t index);
unsigned int drv8235_emul_protected_violations(const struct emul *target);

#endif /* DRV8235_EMUL_H_ */
