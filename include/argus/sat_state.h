/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_SAT_STATE_H_
#define ARGUS_SAT_STATE_H_

/* Satellite operating states. Values match Python core/states.py STATES. */
enum argus_sat_state {
	ARGUS_STATE_STARTUP = 0,
	ARGUS_STATE_DETUMBLING = 1,
	ARGUS_STATE_NOMINAL = 2,
	ARGUS_STATE_LOW_POWER = 3,
	ARGUS_STATE_EXPERIMENT = 4,
};

#endif
