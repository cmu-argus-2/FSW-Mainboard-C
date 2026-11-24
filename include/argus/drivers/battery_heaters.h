/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ARGUS_DRIVERS_BATTERY_HEATERS_H_
#define ARGUS_DRIVERS_BATTERY_HEATERS_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Port of flight/hal/drivers/batteryheaters.py. HEAT_EN is a shared master
 * enable: it is raised before a heater line is raised, and lowered only after
 * both heater lines are low. Heater 1 is optional (DT heat1-on-gpios).
 *
 * All functions return 0 or a negative errno: -EINVAL for a bad index,
 * -ENOTSUP for heater 1 on a single-heater board, or a GPIO error. A failed
 * write leaves the remaining lines untouched; the commanded state is updated
 * only for writes that succeeded, so device_errors flags any line left
 * out of step.
 */

enum battery_heater {
	BATTERY_HEATER_0 = 0,
	BATTERY_HEATER_1 = 1,
};

/* Same numeric codes as Python Errors.BATT_HEATER_*_GPIO_ERROR. */
enum battery_heaters_error_code {
	BATT_HEATER_EN_GPIO_ERROR = 0x26,
	BATT_HEATER_HEAT0_GPIO_ERROR = 0x27,
	BATT_HEATER_HEAT1_GPIO_ERROR = 0x28,
};

/* Python device_errors returns a list of zero or more of the codes above,
 * in EN, HEAT0, HEAT1 order.
 */
struct battery_heaters_errors {
	uint8_t count;
	uint8_t codes[3];
};

/* Python enable_heaterN(): raise HEAT_EN, then the heater line. */
int battery_heaters_enable(const struct device *dev, enum battery_heater heater);

/* Python disable_heaterN(): lower the heater line, then lower HEAT_EN if the
 * other heater line reads back low (or is absent).
 */
int battery_heaters_disable(const struct device *dev, enum battery_heater heater);

/* Python heaterN_enabled(): true when both the heater line and HEAT_EN read
 * back high. Heater 1 on a single-heater board reports false, as in Python.
 */
int battery_heaters_enabled(const struct device *dev, enum battery_heater heater,
			    bool *enabled);

/* Enable or disable both heaters in one call, in the shape of the EPS task's
 * set_heaters callback. Enables are applied before disables. Requesting
 * heater 1 on a single-heater board returns -ENOTSUP without changing anything.
 * set(false, false) lowers HEAT_EN first, then both heater lines, without
 * consulting readback; every line is attempted and the first error returned.
 */
int battery_heaters_set(const struct device *dev, bool heater0, bool heater1);

/* Python device_errors: compare each line's readback with the last value
 * written successfully. Return 0 with the list filled, or a negative GPIO
 * read error. Reporting only: the caller decides what to do with the codes.
 */
int battery_heaters_device_errors(const struct device *dev,
				  struct battery_heaters_errors *errors);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_BATTERY_HEATERS_H_ */
