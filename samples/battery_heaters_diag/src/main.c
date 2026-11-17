/* SPDX-License-Identifier: Apache-2.0 */

/* Based on python_implementation/scripts/battery_heater_test.py: turn both
 * heaters on, print the line states, then turn them off again. Unlike the
 * Python script, the heaters are never left on: after ON_TIME_MS (or right
 * away if enabling fails) both are disabled and checked. Also prints the
 * driver's device_errors, which the Python script did not have.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "argus/drivers/battery_heaters.h"

/* Keep short: the heaters draw real power from the battery board. */
#define ON_TIME_MS 5000
#define SAMPLE_PERIOD_MS 500
#define DISABLE_RETRY_MS 10

static const struct device *const heaters = DEVICE_DT_GET(DT_NODELABEL(battery_heaters));

static void print_status(void)
{
	bool on0 = false;
	bool on1 = false;
	struct battery_heaters_errors errors = {0};
	int ret0 = battery_heaters_enabled(heaters, BATTERY_HEATER_0, &on0);
	int ret1 = battery_heaters_enabled(heaters, BATTERY_HEATER_1, &on1);
	int ret_err = battery_heaters_device_errors(heaters, &errors);

	printk("Heater 0: %d (%d)  Heater 1: %d (%d)  errors:", on0, ret0, on1, ret1);
	if (ret_err != 0) {
		printk(" read failed (%d)", ret_err);
	}
	for (uint8_t i = 0; i < errors.count; ++i) {
		printk(" 0x%02x", errors.codes[i]);
	}
	printk(errors.count == 0 && ret_err == 0 ? " none\n" : "\n");
}

int main(void)
{
	int ret;

	if (!device_is_ready(heaters)) {
		printk("Battery heaters not ready\n");
		return 0;
	}

	ret = battery_heaters_set(heaters, true, true);
	printk("Enabling both heaters for %d ms: %d\n", ON_TIME_MS, ret);

	/* On an enable error some lines may already be high, so skip the
	 * on-period but still fall through to the disable below.
	 */
	if (ret == 0) {
		int64_t end = k_uptime_get() + ON_TIME_MS;

		while (k_uptime_get() < end) {
			print_status();
			k_msleep(SAMPLE_PERIOD_MS);
		}
	}

	/* Do not give up until both heaters are off. */
	while ((ret = battery_heaters_set(heaters, false, false)) != 0) {
		printk("Disabling both heaters failed: %d, retrying\n", ret);
		k_msleep(DISABLE_RETRY_MS);
	}
	printk("Disabled both heaters\n");
	print_status();
	return 0;
}
