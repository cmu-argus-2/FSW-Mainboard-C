/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "argus/drivers/adm1176.h"

static const struct {
	const char *name;
	const struct device *dev;
} monitors[] = {
	{"main", DEVICE_DT_GET(DT_NODELABEL(main_power_monitor))},
	{"GPS", DEVICE_DT_GET(DT_NODELABEL(gps_power_monitor))},
	{"radio", DEVICE_DT_GET(DT_NODELABEL(radio_power_monitor))},
	{"Jetson", DEVICE_DT_GET(DT_NODELABEL(jetson_power_monitor))},
};

static void print_monitor(const char *name, const struct device *dev)
{
	struct sensor_value voltage, current;
	uint8_t status;
	int ret;

	if (!device_is_ready(dev)) {
		printk("%s: NOT READY\n", name);
		return;
	}
	ret = sensor_sample_fetch(dev);
	if (ret == 0) {
		ret = sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
	}
	if (ret == 0) {
		ret = sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
	}
	if (ret == 0) {
		ret = adm1176_status(dev, &status);
	}
	if (ret != 0) {
		printk("%s: read failed: %d\n", name, ret);
		return;
	}
	printk("%s: %d.%06d V, %d.%06d A, status=0x%02x\n",
	       name, voltage.val1, voltage.val2, current.val1, current.val2, status);
}

int main(void)
{
	printk("ADM1176 telemetry test: output and alert settings are unchanged.\n");
	for (;;) {
		for (size_t i = 0; i < ARRAY_SIZE(monitors); i++) {
			print_monitor(monitors[i].name, monitors[i].dev);
		}
		k_sleep(K_SECONDS(2));
	}
	return 0;
}
