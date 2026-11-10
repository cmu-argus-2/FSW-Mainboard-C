/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_APPS_EPS_EPS_H_
#define ARGUS_APPS_EPS_EPS_H_

#include <stdbool.h>
#include <stdint.h>

enum eps_power_flag { EPS_NONE, EPS_LOW_POWER, EPS_NOMINAL };

enum eps_power_limit_mw {
	EPS_MAIN_LIMIT_MW = 1000,
	EPS_PERIPHERAL_LIMIT_MW = 1000,
	EPS_RADIO_LIMIT_MW = 3300,
	EPS_JETSON_LIMIT_MW = 16000,
	EPS_COIL_LIMIT_MW = 1500,
};

struct eps_vi {
	int32_t mv;
	int32_t ma;
};

enum eps_power_flag eps_next_power_flag(enum eps_power_flag previous, int32_t soc);
bool eps_next_heater(bool enabled, int32_t temp_cc);
bool eps_power_alert(struct eps_vi reading, int32_t threshold_mw);

#endif
