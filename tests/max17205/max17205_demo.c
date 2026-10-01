/**
 * @authors Ailisi Bao, John Buttles, Jacob Rohozen
 * @file max17205_demo.c
 *
 * @brief On-target demo for the MAX17205 fuel gauge driver (src/drivers/max17205.c).
 *
 * Gets the gauge from the `fuel_gauge` devicetree node, then prints every property the
 * driver supports once per DEMO_PERIOD_MS over the console. A property that fails to read
 * prints its errno instead of a value, so a missing battery board, a NACK on the shadow-RAM
 * address (0x0B), or a wrong register all show up on their own line.
 *
 * This is a Zephyr application main(), not a host test: it is not built by
 * tests/max17205/CMakeLists.txt. To run it on the board, swap it in for src/main.c in the
 * top-level CMakeLists.txt:
 *
 *     target_sources(app PRIVATE
 *             tests/max17205/max17205_demo.c
 *             src/drivers/max17205.c
 *     )
 *
 * then `west build` and `west flash` as usual.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>

#include "argus/drivers/max17205.h"

/* Time between snapshots. */
#define DEMO_PERIOD_MS 2000

/* Set to 1 to issue max17205_reset() once at boot. A reset restarts the gauge's
 * ModelGauge learning, so it is off by default. */
#define DEMO_RESET_ON_BOOT 0

/* The USB console sends printk output from a work item a little later. If the board locks up
 * right after a printk, that text is never sent. Sleeping this long before each I2C access lets
 * the text go out first, so the last line on screen shows where it stopped. Set to 0 to disable. */
#define DEMO_FLUSH_MS 20

/* 0.1 K -> 0.1 degC */
#define DK_TO_DECI_C(dk) ((int32_t)(dk) - 2732)

/* PERIPH_PWR_EN (GPIO42): the enable pin of the periph_3v3 regulator in the board devicetree.
 * The regulator driver already drives it high at boot; main() drives it again and reads it back
 * so the demo does not depend on that. */
static const struct gpio_dt_spec periph_pwr = GPIO_DT_SPEC_GET(DT_NODELABEL(periph_3v3), enable_gpios);

static const struct device *const gauge = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));

/* How to turn a property value into text. */
enum demo_fmt {
	FMT_PCT,         /* relative_state_of_charge_pct */
	FMT_UAH,         /* remaining_capacity_uah */
	FMT_UA,          /* current_ua */
	FMT_UV,          /* voltage_uv */
	FMT_CUSTOM_UV,   /* custom_uint, microvolts */
	FMT_COUNT,       /* cycle_count */
	FMT_TTE_MINS,    /* runtime_to_empty_mins */
	FMT_TTF_MINS,    /* runtime_to_full_mins */
	FMT_CUSTOM_RAW,  /* custom_uint, unscaled */
	FMT_DK,          /* temperature_dk */
	FMT_CUSTOM_DK,   /* custom_int, 0.1 K */
};

struct demo_prop {
	fuel_gauge_prop_t prop;
	const char *name;
	enum demo_fmt fmt;
};

static const struct demo_prop props[] = {
	{ FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT, "State of charge", FMT_PCT },
	{ FUEL_GAUGE_REMAINING_CAPACITY_UAH, "Remaining capacity", FMT_UAH },
	{ FUEL_GAUGE_CURRENT_UA, "Current", FMT_UA },
	{ FUEL_GAUGE_VOLTAGE_UV, "Pack voltage", FMT_UV },
	{ MAX17205_PROP_MID_VOLTAGE_UV, "Lowest cell voltage", FMT_CUSTOM_UV },
	{ FUEL_GAUGE_CYCLE_COUNT, "Cycle count", FMT_COUNT },
	{ FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS, "Time to empty", FMT_TTE_MINS },
	{ FUEL_GAUGE_RUNTIME_TO_FULL_MINS, "Time to full", FMT_TTF_MINS },
	{ MAX17205_PROP_TIME_PWRUP_S, "TimerH (raw)", FMT_CUSTOM_RAW },
	{ FUEL_GAUGE_TEMPERATURE_DK, "Temperature", FMT_DK },
	{ MAX17205_PROP_TEMP_AIN1_DK, "Thermistor AIN1", FMT_CUSTOM_DK },
	{ MAX17205_PROP_TEMP_AIN2_DK, "Thermistor AIN2", FMT_CUSTOM_DK },
	{ MAX17205_PROP_TEMP_DIE_DK, "Die temperature", FMT_CUSTOM_DK },
};

/* printk has no %f: print a signed fixed-point value with `decimals` digits after the point. */
static void print_fixed(int64_t value, uint32_t scale, int decimals)
{
	const char *sign = value < 0 ? "-" : "";
	uint64_t mag = value < 0 ? (uint64_t)(-value) : (uint64_t)value;

	/* 32-bit args only: printk drops 64-bit conversions unless CBPRINTF_FULL_INTEGRAL. */
	printk("%s%u.%0*u", sign, (uint32_t)(mag / scale), decimals, (uint32_t)(mag % scale));
}

static void print_value(enum demo_fmt fmt, const union fuel_gauge_prop_val *val)
{
	switch (fmt) {
	case FMT_PCT:
		printk("%u %%", val->relative_state_of_charge_pct);
		break;
	case FMT_UAH:
		print_fixed(val->remaining_capacity_uah, 1000, 3);
		printk(" mAh");
		break;
	case FMT_UA:
		print_fixed(val->current_ua, 1000, 3);
		printk(" mA%s", val->current_ua < 0 ? " (discharging)" : "");
		break;
	case FMT_UV:
		print_fixed(val->voltage_uv, 1000, 3);
		printk(" mV");
		break;
	case FMT_CUSTOM_UV:
		print_fixed(val->custom_uint, 1000, 3);
		printk(" mV");
		break;
	case FMT_COUNT:
		printk("%u", val->cycle_count);
		break;
	case FMT_TTE_MINS:
		printk("%u min", val->runtime_to_empty_mins);
		break;
	case FMT_TTF_MINS:
		printk("%u min", val->runtime_to_full_mins);
		break;
	case FMT_CUSTOM_RAW:
		printk("%u", val->custom_uint);
		break;
	case FMT_DK:
		print_fixed(DK_TO_DECI_C(val->temperature_dk), 10, 1);
		printk(" C");
		break;
	case FMT_CUSTOM_DK:
		print_fixed(DK_TO_DECI_C(val->custom_int), 10, 1);
		printk(" C");
		break;
	}
}

static void print_snapshot(unsigned int n)
{
	int failures = 0;

	printk("\n--- MAX17205 snapshot %u (uptime %u ms) ---\n", n, k_uptime_get_32());

	for (size_t i = 0; i < ARRAY_SIZE(props); i++) {
		union fuel_gauge_prop_val val = { 0 };
		int rc;

		printk("  %-20s ", props[i].name);
		k_msleep(DEMO_FLUSH_MS);
		rc = fuel_gauge_get_prop(gauge, props[i].prop, &val);
		if (rc) {
			printk("ERROR %d\n", rc);
			failures++;
			continue;
		}
		print_value(props[i].fmt, &val);
		printk("\n");
	}

	if (failures) {
		printk("  %d of %u reads failed\n", failures, (unsigned int)ARRAY_SIZE(props));
	}
}

int main(void)
{
	// Sleep 5s
	k_msleep(5000);

	// Enable peripheral power
	if (!gpio_is_ready_dt(&periph_pwr) ||
	    gpio_pin_configure_dt(&periph_pwr, GPIO_OUTPUT_ACTIVE) != 0) {
		printk("could not drive PERIPH_PWR_EN\n");
		return 0;
	}
	/* Let the rail come up, same as the regulator's startup-delay-us. */
	k_msleep(100);
	printk("PERIPH_PWR_EN = %d\n", gpio_pin_get_dt(&periph_pwr));

	printk("MAX17205 demo: %s at 0x%02x (shadow RAM 0x%02x)\n", gauge->name,
	       MAX17205_MAIN_I2C_ADDR, MAX17205_SHADOW_I2C_ADDR);
	k_msleep(DEMO_FLUSH_MS);

	if (!device_is_ready(gauge)) {
		printk("fuel gauge not ready (I2C bus down?)\n");
		return 0;
	}

	if (DEMO_RESET_ON_BOOT) {
		int rc = max17205_reset(gauge);

		printk("max17205_reset() -> %d\n", rc);
		/* Give the gauge time to come back from POR before reading. */
		k_msleep(100);
	}

	for (unsigned int n = 0;; n++) {
		print_snapshot(n);
		k_msleep(DEMO_PERIOD_MS);
	}
}
