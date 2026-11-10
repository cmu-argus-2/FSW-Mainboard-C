/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/apps/eps/eps.h"

enum eps_power_flag eps_next_power_flag(enum eps_power_flag previous, int32_t soc)
{
	if (soc < 0 || soc > 100 || previous < EPS_NONE || previous > EPS_NOMINAL) {
		return EPS_NONE;
	}
	if (soc <= 30) {
		return EPS_LOW_POWER;
	}
	if (soc >= 40) {
		return EPS_NOMINAL;
	}
	return previous == EPS_LOW_POWER ? EPS_LOW_POWER : EPS_NOMINAL;
}

bool eps_next_heater(bool enabled, int32_t temp_cc)
{
	if (!enabled && temp_cc <= 0) {
		return true;
	}
	if (enabled && temp_cc >= 500) {
		return false;
	}
	return enabled;
}

bool eps_power_alert(struct eps_vi reading, int32_t threshold_mw)
{
	/* Compare before dividing: fractional mW must not change a threshold test. */
	return (int64_t)reading.mv * reading.ma >= (int64_t)threshold_mw * 1000;
}
