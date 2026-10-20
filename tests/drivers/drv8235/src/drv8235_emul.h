/* SPDX-License-Identifier: Apache-2.0 */

#ifndef DRV8235_EMUL_H_
#define DRV8235_EMUL_H_

#include <stdint.h>

#include <zephyr/drivers/emul.h>

void drv8235_emul_reset(const struct emul *target);
void drv8235_emul_power_cycle(const struct emul *target);
void drv8235_emul_set_status(const struct emul *target, uint8_t voltage, uint8_t current,
			     uint8_t duty);
void drv8235_emul_set_faults(const struct emul *target, uint8_t faults);
void drv8235_emul_set_error(const struct emul *target, int error);
uint8_t drv8235_emul_get_reg(const struct emul *target, uint8_t reg);

#endif /* DRV8235_EMUL_H_ */
