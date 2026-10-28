/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include "adm1176_emul.h"
#include "argus/drivers/adm1176.h"

#define ADM1176_TEST_COMMAND_CONTINUOUS 0x05U
#define ADM1176_TEST_ALERT_HOT_SWAP 0x04U
#define ADM1176_TEST_ALERT_ADC_OC4 0x02U
#define ADM1176_TEST_ALERT_CLEAR 0x10U

struct adm1176_driver_fixture {
	const struct device *dev;
	const struct emul *emul;
};

static void *adm1176_driver_setup(void)
{
	static struct adm1176_driver_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(adm1176_test)),
		.emul = EMUL_DT_GET(DT_NODELABEL(adm1176_test)),
	};

	zassert_true(device_is_ready(fixture.dev));
	zassert_equal(adm1176_emul_get_command(fixture.emul),
		      ADM1176_TEST_COMMAND_CONTINUOUS);
	zassert_equal(adm1176_emul_get_alert_enable(fixture.emul),
		      ADM1176_TEST_ALERT_HOT_SWAP);
	zassert_equal(adm1176_emul_get_threshold(fixture.emul), UINT8_MAX);
	zassert_equal(adm1176_emul_get_control(fixture.emul), 0U);
	return &fixture;
}

static void adm1176_driver_before(void *fixture_ptr)
{
	struct adm1176_driver_fixture *fixture = fixture_ptr;

	adm1176_emul_reset(fixture->emul);
}

ZTEST_F(adm1176_driver, test_device_initializes_in_continuous_mode)
{
	zassert_true(device_is_ready(fixture->dev));
	zassert_equal(adm1176_emul_get_command(fixture->emul),
		      ADM1176_TEST_COMMAND_CONTINUOUS);
}

ZTEST_F(adm1176_driver, test_sensor_api_reads_voltage_current_and_power)
{
	const uint8_t measurement[3] = {0x80, 0x40, 0x00};
	struct sensor_value value;

	adm1176_emul_set_measurement(fixture->emul, measurement);
	zassert_ok(sensor_sample_fetch(fixture->dev));

	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 13175000LL);
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_CURRENT, &value));
	zassert_equal(sensor_value_to_micro(&value), 2646000LL);
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_POWER, &value));
	zassert_equal(sensor_value_to_micro(&value), 34861050LL);
}

ZTEST_F(adm1176_driver, test_sensor_api_rejects_unsupported_channel)
{
	struct sensor_value value;

	zassert_equal(sensor_sample_fetch_chan(fixture->dev,
					       SENSOR_CHAN_AMBIENT_TEMP),
		      -ENOTSUP);
	zassert_equal(sensor_channel_get(fixture->dev, SENSOR_CHAN_AMBIENT_TEMP,
					 &value), -ENOTSUP);
}

ZTEST_F(adm1176_driver, test_status_read_decodes_flags_and_restores_command)
{
	struct adm1176_status_flags status;

	adm1176_emul_set_status(fixture->emul,
				 ADM1176_STATUS_ADC_ALERT | ADM1176_STATUS_OFF);
	zassert_ok(adm1176_get_status_flags(fixture->dev, &status));
	zassert_true(status.adc_alert);
	zassert_true(status.output_off);
	zassert_false(status.hot_swap_alert);
	zassert_equal(adm1176_emul_get_command(fixture->emul),
		      ADM1176_TEST_COMMAND_CONTINUOUS);
}

ZTEST_F(adm1176_driver, test_hot_swap_output_can_be_disabled_and_enabled)
{
	zassert_ok(adm1176_set_enabled(fixture->dev, false));
	zassert_equal(adm1176_emul_get_control(fixture->emul), 1U);

	zassert_ok(adm1176_set_enabled(fixture->dev, true));
	zassert_equal(adm1176_emul_get_control(fixture->emul), 0U);
	zassert_equal(adm1176_emul_get_command(fixture->emul),
		      ADM1176_TEST_COMMAND_CONTINUOUS);
}

ZTEST_F(adm1176_driver, test_overcurrent_threshold_and_alert_clear)
{
	uint64_t current_ua;

	zassert_ok(adm1176_set_overcurrent_threshold(fixture->dev, 5292000ULL));
	zassert_equal(adm1176_emul_get_alert_enable(fixture->emul),
		      ADM1176_TEST_ALERT_HOT_SWAP | ADM1176_TEST_ALERT_ADC_OC4);
	zassert_equal(adm1176_emul_get_threshold(fixture->emul), 128U);
	zassert_ok(adm1176_get_overcurrent_threshold(fixture->dev, &current_ua));
	zassert_equal(current_ua, 5292000ULL);

	zassert_ok(adm1176_clear_alerts(fixture->dev));
	zassert_equal(adm1176_emul_get_alert_enable(fixture->emul),
		      ADM1176_TEST_ALERT_HOT_SWAP | ADM1176_TEST_ALERT_ADC_OC4 |
			      ADM1176_TEST_ALERT_CLEAR);
}

ZTEST_SUITE(adm1176_driver, NULL, adm1176_driver_setup,
		    adm1176_driver_before, NULL, NULL);
