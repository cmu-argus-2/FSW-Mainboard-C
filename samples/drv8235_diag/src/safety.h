/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ARGUS_DRV8235_DIAG_SAFETY_H_
#define ARGUS_DRV8235_DIAG_SAFETY_H_

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>

#include "argus/drivers/drv8235.h"

/* A failed shutdown must be resolved before taking another rail reference. */
static inline int drv8235_diag_enable_rail(const struct device *supply)
{
	return regulator_is_enabled(supply) ? -EBUSY : regulator_enable(supply);
}

/*
 * A failed bridge shutdown must fall back to removing the shared supply.
 * Preserve the bridge error; the caller must check/report whether the rail
 * actually turned off, since regulator_disable can also fail.
 */
static inline int drv8235_diag_stop_axis(const struct device *dev,
				       const struct device *supply, bool *rail_armed)
{
	const int result = device_is_ready(dev) ? drv8235_set_enabled(dev, false) : -ENODEV;

	if (result != 0) {
		*rail_armed = false;
		if (regulator_is_enabled(supply)) {
			(void)regulator_disable(supply);
		}
	}
	return result;
}

#endif /* ARGUS_DRV8235_DIAG_SAFETY_H_ */
