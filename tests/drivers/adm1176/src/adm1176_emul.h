/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ADM1176_EMUL_H_
#define ADM1176_EMUL_H_

#include <stdint.h>
#include <zephyr/drivers/emul.h>

/* Test-only device state; skip successful transfers before injecting error. */
struct adm1176_emul_state {
	uint8_t sample[3];
	uint8_t command, status, alert_enable, threshold, control;
	unsigned int skip;
	int error;
};

struct adm1176_emul_state *adm1176_emul_state(const struct emul *emul);
void adm1176_emul_reset(const struct emul *emul);

#endif
