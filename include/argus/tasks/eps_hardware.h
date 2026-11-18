/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_TASKS_EPS_HARDWARE_H_
#define ARGUS_TASKS_EPS_HARDWARE_H_

#include "argus/tasks/eps.h"

/* Initialize the EPS hardware once, before starting the periodic task. */
struct eps_io eps_hardware_io(void);

#endif
