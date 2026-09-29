#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/sys/printk.h>

#include "argus_version.h"

static const struct device *const neopixel = DEVICE_DT_GET(DT_ALIAS(led_strip));

static const struct led_rgb colors[] = {
	{ .r = 0x10 }, { .g = 0x10 }, { .b = 0x10 }, { 0 },
};

int main(void)
{
	printk("Argus FSW mainboard rev %s (%s)\n", CONFIG_BOARD_REVISION, ARGUS_GIT_VERSION);

	if (!device_is_ready(neopixel)) {
		printk("neopixel not ready\n");
		return 0;
	}

	for (unsigned int i = 0;; i++) {
		struct led_rgb pixel = colors[i % ARRAY_SIZE(colors)];

		led_strip_update_rgb(neopixel, &pixel, 1);
		printk("argus alive, tick %u\n", i);
		k_msleep(500);
	}
}
