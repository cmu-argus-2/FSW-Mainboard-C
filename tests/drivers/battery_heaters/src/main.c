/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/ztest.h>

#include "argus/drivers/battery_heaters.h"

#define DUAL DT_NODELABEL(heaters_dual)
#define SINGLE DT_NODELABEL(heaters_single)

static const struct device *const dual = DEVICE_DT_GET(DUAL);
static const struct device *const single = DEVICE_DT_GET(SINGLE);

static const struct gpio_dt_spec en = GPIO_DT_SPEC_GET(DUAL, heat_en_gpios);
static const struct gpio_dt_spec h0 = GPIO_DT_SPEC_GET(DUAL, heat0_on_gpios);
static const struct gpio_dt_spec h1 = GPIO_DT_SPEC_GET(DUAL, heat1_on_gpios);
static const struct gpio_dt_spec s_en = GPIO_DT_SPEC_GET(SINGLE, heat_en_gpios);
static const struct gpio_dt_spec s_h0 = GPIO_DT_SPEC_GET(SINGLE, heat0_on_gpios);

/* Edge log for checking write order. */
struct edge {
	gpio_pin_t pin;
	int value;
};
static struct edge edges[16];
static size_t edge_count;
static struct gpio_callback edge_cb;

static void on_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(cb);
	for (gpio_pin_t pin = 0; pin < 32; ++pin) {
		if ((pins & BIT(pin)) && edge_count < ARRAY_SIZE(edges)) {
			edges[edge_count].pin = pin;
			edges[edge_count].value = gpio_emul_output_get(port, pin);
			edge_count++;
		}
	}
}

static void assert_outputs(const struct gpio_dt_spec *lines, const int *expected, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		zassert_equal(gpio_emul_output_get_dt(&lines[i]), expected[i],
			      "pin %u", lines[i].pin);
	}
}

#define ASSERT_DUAL(e, a, b) \
	assert_outputs((const struct gpio_dt_spec[]){en, h0, h1}, (const int[]){e, a, b}, 3)

static bool enabled(const struct device *dev, enum battery_heater heater)
{
	bool value = true;

	zassert_ok(battery_heaters_enabled(dev, heater, &value));
	return value;
}

static void *setup(void)
{
	zassert_true(device_is_ready(dual));
	zassert_true(device_is_ready(single));
	/* Init drives every line low. */
	ASSERT_DUAL(0, 0, 0);
	zassert_equal(gpio_emul_output_get_dt(&s_en), 0);
	zassert_equal(gpio_emul_output_get_dt(&s_h0), 0);

	gpio_init_callback(&edge_cb, on_edge, BIT(en.pin) | BIT(h0.pin) | BIT(h1.pin));
	zassert_ok(gpio_add_callback(en.port, &edge_cb));
	zassert_ok(gpio_pin_interrupt_configure_dt(&en, GPIO_INT_EDGE_BOTH));
	zassert_ok(gpio_pin_interrupt_configure_dt(&h0, GPIO_INT_EDGE_BOTH));
	zassert_ok(gpio_pin_interrupt_configure_dt(&h1, GPIO_INT_EDGE_BOTH));
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(battery_heaters_set(dual, false, false));
	zassert_ok(battery_heaters_set(single, false, false));
	ASSERT_DUAL(0, 0, 0);
	edge_count = 0;
}

ZTEST(battery_heaters, test_enable_raises_master_before_heater)
{
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_0));
	ASSERT_DUAL(1, 1, 0);
	zassert_equal(edge_count, 2);
	zassert_equal(edges[0].pin, en.pin);
	zassert_equal(edges[0].value, 1);
	zassert_equal(edges[1].pin, h0.pin);
	zassert_equal(edges[1].value, 1);
	zassert_true(enabled(dual, BATTERY_HEATER_0));
	zassert_false(enabled(dual, BATTERY_HEATER_1));
}

ZTEST(battery_heaters, test_disable_lowers_heater_before_master)
{
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_1));
	edge_count = 0;
	zassert_ok(battery_heaters_disable(dual, BATTERY_HEATER_1));
	ASSERT_DUAL(0, 0, 0);
	zassert_equal(edge_count, 2);
	zassert_equal(edges[0].pin, h1.pin);
	zassert_equal(edges[0].value, 0);
	zassert_equal(edges[1].pin, en.pin);
	zassert_equal(edges[1].value, 0);
	zassert_false(enabled(dual, BATTERY_HEATER_1));
}

ZTEST(battery_heaters, test_master_stays_on_while_either_heater_on)
{
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_0));
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_1));
	ASSERT_DUAL(1, 1, 1);

	zassert_ok(battery_heaters_disable(dual, BATTERY_HEATER_0));
	ASSERT_DUAL(1, 0, 1);
	zassert_false(enabled(dual, BATTERY_HEATER_0));
	zassert_true(enabled(dual, BATTERY_HEATER_1));

	zassert_ok(battery_heaters_disable(dual, BATTERY_HEATER_1));
	ASSERT_DUAL(0, 0, 0);
}

ZTEST(battery_heaters, test_disable_already_off_heater_is_harmless)
{
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_1));
	zassert_ok(battery_heaters_disable(dual, BATTERY_HEATER_0));
	ASSERT_DUAL(1, 0, 1);
}

ZTEST(battery_heaters, test_enabled_requires_master_readback)
{
	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_0));
	/* Pad reads low although the driver drove it high. */
	zassert_ok(gpio_emul_input_set_dt(&en, 0));
	zassert_false(enabled(dual, BATTERY_HEATER_0));
}

ZTEST(battery_heaters, test_set_both)
{
	zassert_ok(battery_heaters_set(dual, true, false));
	ASSERT_DUAL(1, 1, 0);
	zassert_ok(battery_heaters_set(dual, true, true));
	ASSERT_DUAL(1, 1, 1);
	zassert_ok(battery_heaters_set(dual, false, true));
	ASSERT_DUAL(1, 0, 1);
	zassert_ok(battery_heaters_set(dual, false, false));
	ASSERT_DUAL(0, 0, 0);
}

ZTEST(battery_heaters, test_invalid_heater)
{
	bool value;

	zassert_equal(battery_heaters_enable(dual, 2), -EINVAL);
	zassert_equal(battery_heaters_disable(dual, 2), -EINVAL);
	zassert_equal(battery_heaters_enabled(dual, 2, &value), -EINVAL);
	ASSERT_DUAL(0, 0, 0);
}

ZTEST(battery_heaters, test_no_errors_when_readback_matches)
{
	struct battery_heaters_errors errors = {.count = 0xFF};

	zassert_ok(battery_heaters_device_errors(dual, &errors));
	zassert_equal(errors.count, 0);
	zassert_ok(battery_heaters_set(dual, true, true));
	zassert_ok(battery_heaters_device_errors(dual, &errors));
	zassert_equal(errors.count, 0);
}

ZTEST(battery_heaters, test_errors_report_mismatched_lines_in_python_order)
{
	struct battery_heaters_errors errors;

	zassert_ok(battery_heaters_enable(dual, BATTERY_HEATER_0));
	zassert_ok(gpio_emul_input_set_dt(&h0, 0));
	zassert_ok(battery_heaters_device_errors(dual, &errors));
	zassert_equal(errors.count, 1);
	zassert_equal(errors.codes[0], 0x27);

	/* Mismatch in both directions: EN stuck low, HEAT1 stuck high. */
	zassert_ok(gpio_emul_input_set_dt(&h1, 1));
	zassert_ok(gpio_emul_input_set_dt(&en, 0));
	zassert_ok(battery_heaters_device_errors(dual, &errors));
	zassert_equal(errors.count, 3);
	zassert_equal(errors.codes[0], BATT_HEATER_EN_GPIO_ERROR);
	zassert_equal(errors.codes[1], BATT_HEATER_HEAT0_GPIO_ERROR);
	zassert_equal(errors.codes[2], BATT_HEATER_HEAT1_GPIO_ERROR);
}

ZTEST(battery_heaters, test_single_heater_board)
{
	struct battery_heaters_errors errors;

	zassert_equal(battery_heaters_enable(single, BATTERY_HEATER_1), -ENOTSUP);
	zassert_equal(battery_heaters_disable(single, BATTERY_HEATER_1), -ENOTSUP);
	zassert_equal(battery_heaters_set(single, false, true), -ENOTSUP);
	zassert_false(enabled(single, BATTERY_HEATER_1));

	zassert_ok(battery_heaters_enable(single, BATTERY_HEATER_0));
	zassert_equal(gpio_emul_output_get_dt(&s_en), 1);
	zassert_equal(gpio_emul_output_get_dt(&s_h0), 1);
	zassert_true(enabled(single, BATTERY_HEATER_0));

	/* Python's disable_heater0 crashes here with no heater 1. */
	zassert_ok(battery_heaters_disable(single, BATTERY_HEATER_0));
	zassert_equal(gpio_emul_output_get_dt(&s_en), 0);
	zassert_equal(gpio_emul_output_get_dt(&s_h0), 0);

	zassert_ok(battery_heaters_set(single, true, false));
	zassert_ok(battery_heaters_device_errors(single, &errors));
	zassert_equal(errors.count, 0);
}

ZTEST_SUITE(battery_heaters, NULL, setup, before, NULL, NULL);
