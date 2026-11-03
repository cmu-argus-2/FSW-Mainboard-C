/*
 * On-board check for the OPT4003 light sensors: prints every sensor's state and lux once a
 * second. Cover a sensor or shine a light on it and its reading should follow.
 *
 * For a sensor whose init failed, it reads the device ID register again to show why.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/opt4003.h"

#define OPT4003_REG_DEVICE_ID 0x11

struct light_sensor {
	const char *name;
	const struct device *dev;
	struct i2c_dt_spec i2c; /* for probing directly when init failed */
};

#define LIGHT_SENSOR(_name, _label)                                                          \
	{_name, DEVICE_DT_GET(DT_NODELABEL(_label)), I2C_DT_SPEC_GET(DT_NODELABEL(_label))}

/* Names and order from flight/hal/argus_v4.py */
static const struct light_sensor sensors[] = {
	LIGHT_SENSOR("LIGHT_XM", light_xm),
	LIGHT_SENSOR("LIGHT_YM", light_ym),
	LIGHT_SENSOR("LIGHT_ZM", light_zm),
	LIGHT_SENSOR("LIGHT_XP", light_xp),
	LIGHT_SENSOR("LIGHT_YP", light_yp),
	LIGHT_SENSOR("LIGHT_ZP_XP", light_zp_xp),
	LIGHT_SENSOR("LIGHT_ZP_YM", light_zp_ym),
	LIGHT_SENSOR("LIGHT_ZP_XM", light_zp_xm),
	LIGHT_SENSOR("LIGHT_ZP_YP", light_zp_yp),
};

/* Repeat the driver's ID read to tell an I2C failure from "wrong part" */
static void print_init_failure(const struct light_sensor *s)
{
	uint8_t buf[2];
	uint16_t id;
	int ret;

	/*
	 * -EIO covers both "nothing answered" and "arbitration lost"; the I2C driver logs
	 * "ARB lost" for the second, so look for that line next to this one.
	 */
	ret = i2c_burst_read_dt(&s->i2c, OPT4003_REG_DEVICE_ID, buf, sizeof(buf));
	if (ret < 0) {
		printk("%-12s init failed: I2C error at 0x%02x (%d)\n", s->name, s->i2c.addr, ret);
		return;
	}

	id = sys_get_be16(buf);
	if (id == 0x0221) {
		/* It answers correctly now, so it wasn't ready when init ran at boot */
		printk("%-12s init failed at boot, but answers now with ID 0x0221 (power-up timing?)\n",
		       s->name);
		return;
	}

	printk("%-12s init failed: device ID 0x%04x at 0x%02x, expected 0x0221\n", s->name, id,
	       s->i2c.addr);
}

static void print_sensor(const struct light_sensor *s)
{
	struct sensor_value lux;
	uint16_t flags;
	int ret;

	if (!device_is_ready(s->dev)) {
		print_init_failure(s);
		return;
	}

	ret = sensor_sample_fetch(s->dev);
	if (ret == 0) {
		ret = sensor_channel_get(s->dev, SENSOR_CHAN_LIGHT, &lux);
	}
	if (ret < 0) {
		printk("%-12s read failed (%d)\n", s->name, ret);
		return;
	}

	flags = opt4003_get_flags(s->dev);
	printk("%-12s %7d.%03d lux%s\n", s->name, lux.val1, lux.val2 / 1000,
	       (flags & BIT(OPT4003_FLAG_OVERLOAD)) ? "  OVERLOAD" : "");
}

int main(void)
{
	for (unsigned int tick = 0;; tick++) {
		printk("--- OPT4003 demo, tick %u\n", tick);
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
			print_sensor(&sensors[i]);
		}
		k_msleep(1000);
	}
}
