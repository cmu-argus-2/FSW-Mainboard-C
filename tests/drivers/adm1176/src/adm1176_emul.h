/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ADM1176_EMUL_H_
#define ADM1176_EMUL_H_

#include <stdint.h>

#include <zephyr/drivers/emul.h>

void adm1176_emul_reset(const struct emul *target);
void adm1176_emul_set_measurement(const struct emul *target,
				  const uint8_t measurement[3]);
void adm1176_emul_set_status(const struct emul *target, uint8_t status);
uint8_t adm1176_emul_get_command(const struct emul *target);
uint8_t adm1176_emul_get_alert_enable(const struct emul *target);
uint8_t adm1176_emul_get_threshold(const struct emul *target);
uint8_t adm1176_emul_get_control(const struct emul *target);

#endif /* ADM1176_EMUL_H_ */
