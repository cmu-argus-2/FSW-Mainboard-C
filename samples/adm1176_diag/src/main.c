/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "argus/drivers/adm1176.h"

struct monitor {
	const char *name;
	uint8_t address;
	const struct device *device;
};

static const struct monitor monitors[] = {
	{
		.name = "main",
		.address = 0x40,
		.device = DEVICE_DT_GET(DT_NODELABEL(main_power_monitor)),
	},
	{
		.name = "GPS",
		.address = 0x41,
		.device = DEVICE_DT_GET(DT_NODELABEL(gps_power_monitor)),
	},
	{
		.name = "radio",
		.address = 0x42,
		.device = DEVICE_DT_GET(DT_NODELABEL(radio_power_monitor)),
	},
};

static void print_sensor_value(const struct sensor_value *value, const char *unit)
{
	int32_t fraction = value->val2;

	if (fraction < 0) {
		fraction = -fraction;
	}
	printk("%d.%03d %s", value->val1, fraction / 1000, unit);
}

static void print_monitor(const struct monitor *monitor)
{
	struct sensor_value voltage;
	struct sensor_value current;
	struct sensor_value power;
	struct adm1176_status_flags status;
	int result;

	printk("%-14s [0x%02x]: ", monitor->name, monitor->address);
	if (!device_is_ready(monitor->device)) {
		printk("NOT READY\n");
		return;
	}

	result = sensor_sample_fetch(monitor->device);
	if (result != 0) {
		printk("sample read failed: %d\n", result);
		return;
	}

	result = sensor_channel_get(monitor->device, SENSOR_CHAN_VOLTAGE, &voltage);
	result = result != 0 ? result :
		 sensor_channel_get(monitor->device, SENSOR_CHAN_CURRENT, &current);
	result = result != 0 ? result :
		 sensor_channel_get(monitor->device, SENSOR_CHAN_POWER, &power);
	if (result != 0) {
		printk("conversion failed: %d\n", result);
		return;
	}

	print_sensor_value(&voltage, "V");
	printk(", ");
	print_sensor_value(&current, "A");
	printk(", ");
	print_sensor_value(&power, "W");

	result = adm1176_get_status_flags(monitor->device, &status);
	if (result == 0) {
		printk(", status=0x%02x", status.raw);
	} else {
		printk(", status read failed: %d", result);
	}
	printk("\n");
}

int main(void)
{
	printk("ADM1176 read-only hardware test\n");
	printk("No output switching or alert thresholds will be changed.\n\n");

	for (;;) {
		for (size_t i = 0; i < ARRAY_SIZE(monitors); ++i) {
			print_monitor(&monitors[i]);
		}
		printk("\n");
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
