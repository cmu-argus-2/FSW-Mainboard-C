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
#include <zephyr/init.h>
#include <zephyr/ztest.h>

#include "adm1176_emul.h"
#include "argus/drivers/adm1176.h"

#define ADM1176_TEST_COMMAND_CONTINUOUS 0x05U
#define ADM1176_TEST_ALERT_HOT_SWAP 0x04U
#define ADM1176_TEST_ALERT_ADC_OC4 0x02U
#define ADM1176_TEST_RETAINED_ALERT_ENABLE 0x0DU
#define ADM1176_TEST_RETAINED_THRESHOLD 0x37U

/* Seed write-only hardware state after I2C init but before the sensor driver,
 * modeling an MCU reboot while the power monitor remains powered.
 */
BUILD_ASSERT(CONFIG_I2C_INIT_PRIORITY < 89 && CONFIG_SENSOR_INIT_PRIORITY > 89);

static int adm1176_seed_warm_boot(void)
{
	adm1176_emul_seed_retained_config(
		EMUL_DT_GET(DT_NODELABEL(adm1176_test)),
		ADM1176_TEST_RETAINED_ALERT_ENABLE,
		ADM1176_TEST_RETAINED_THRESHOLD, 1U);
	return 0;
}

SYS_INIT(adm1176_seed_warm_boot, POST_KERNEL, 89);

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
	uint64_t threshold;
	struct sensor_value value;

	zassert_true(device_is_ready(fixture.dev));
	zassert_equal(adm1176_emul_get_command(fixture.emul),
		      ADM1176_TEST_COMMAND_CONTINUOUS);
	zassert_equal(adm1176_emul_get_alert_enable(fixture.emul),
		      ADM1176_TEST_RETAINED_ALERT_ENABLE);
	zassert_equal(adm1176_emul_get_threshold(fixture.emul),
		      ADM1176_TEST_RETAINED_THRESHOLD);
	zassert_equal(adm1176_emul_get_control(fixture.emul), 1U);
	zassert_equal(adm1176_get_overcurrent_threshold(fixture.dev, &threshold),
		      -ENODATA);
	zassert_equal(adm1176_clear_alerts(fixture.dev), -ENODATA);
	zassert_equal(adm1176_emul_get_alert_enable(fixture.emul),
		      ADM1176_TEST_RETAINED_ALERT_ENABLE);
	zassert_equal(sensor_channel_get(fixture.dev, SENSOR_CHAN_VOLTAGE, &value),
		      -ENODATA);
	return &fixture;
}

static void adm1176_driver_before(void *fixture_ptr)
{
	struct adm1176_driver_fixture *fixture = fixture_ptr;

	adm1176_emul_reset(fixture->emul);
	zassert_ok(adm1176_set_overcurrent_threshold_raw(fixture->dev, UINT8_MAX));
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
	bool enabled;

	zassert_ok(adm1176_is_enabled(fixture->dev, &enabled));
	zassert_false(enabled);

	zassert_ok(adm1176_set_enabled(fixture->dev, true));
	zassert_equal(adm1176_emul_get_control(fixture->emul), 0U);
	zassert_ok(adm1176_is_enabled(fixture->dev, &enabled));
	zassert_true(enabled);
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

	adm1176_emul_set_status(fixture->emul, ADM1176_STATUS_ADC_ALERT |
			       ADM1176_STATUS_HOT_SWAP_ALERT | ADM1176_STATUS_OFF_ALERT);
	zassert_ok(adm1176_clear_alerts(fixture->dev));
	zassert_equal(adm1176_emul_get_alert_enable(fixture->emul),
		      ADM1176_TEST_ALERT_HOT_SWAP | ADM1176_TEST_ALERT_ADC_OC4);
	uint8_t status;

	zassert_ok(adm1176_read_status(fixture->dev, &status));
	zassert_equal(status, 0U);
}

ZTEST_F(adm1176_driver, test_fetch_recovers_after_status_restore_failure)
{
	uint8_t status;
	const uint8_t measurement[3] = {0x80, 0x40, 0x00};
	struct sensor_value value;

	adm1176_emul_set_measurement(fixture->emul, measurement);
	adm1176_emul_fail_after(fixture->emul, 2U, -EIO);
	zassert_equal(adm1176_read_status(fixture->dev, &status), -EIO);
	zassert_equal(adm1176_emul_get_command(fixture->emul), 0x45U);
	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_equal(adm1176_emul_get_command(fixture->emul), 0x05U);
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 13175000LL);
}

ZTEST_F(adm1176_driver, test_status_read_failure_still_restores_command)
{
	uint8_t status;

	adm1176_emul_fail_after(fixture->emul, 1U, -EIO);
	zassert_equal(adm1176_read_status(fixture->dev, &status), -EIO);
	zassert_equal(adm1176_emul_get_command(fixture->emul), 0x05U);
	zassert_ok(adm1176_read_status(fixture->dev, &status));
}

ZTEST_F(adm1176_driver, test_failed_sample_read_invalidates_old_data)
{
	struct sensor_value value;

	zassert_ok(sensor_sample_fetch(fixture->dev));
	adm1176_emul_fail_after(fixture->emul, 1U, -EIO);
	zassert_equal(sensor_sample_fetch(fixture->dev), -EIO);
	zassert_equal(sensor_channel_get(fixture->dev, SENSOR_CHAN_CURRENT, &value),
		      -ENODATA);
	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_CURRENT, &value));
}

ZTEST_F(adm1176_driver, test_failed_measurement_command_invalidates_old_data)
{
	struct sensor_value value;

	zassert_ok(sensor_sample_fetch(fixture->dev));
	adm1176_emul_fail_after(fixture->emul, 0U, -EIO);
	zassert_equal(sensor_sample_fetch(fixture->dev), -EIO);
	zassert_equal(sensor_channel_get(fixture->dev, SENSOR_CHAN_POWER, &value),
		      -ENODATA);
}

ZTEST_F(adm1176_driver, test_failed_threshold_write_invalidates_policy)
{
	uint64_t threshold;

	adm1176_emul_fail_after(fixture->emul, 0U, -EIO);
	zassert_equal(adm1176_set_overcurrent_threshold_raw(fixture->dev, 128U), -EIO);
	zassert_equal(adm1176_get_overcurrent_threshold(fixture->dev, &threshold),
		      -ENODATA);
	zassert_equal(adm1176_clear_alerts(fixture->dev), -ENODATA);
	zassert_ok(adm1176_set_overcurrent_threshold_raw(fixture->dev, 128U));
	zassert_ok(adm1176_get_overcurrent_threshold(fixture->dev, &threshold));
	zassert_equal(threshold, 5292000ULL);
}

ZTEST_F(adm1176_driver, test_partial_configuration_invalidates_policy)
{
	uint64_t threshold;

	adm1176_emul_fail_after(fixture->emul, 1U, -EIO);
	zassert_equal(adm1176_set_overcurrent_threshold_raw(fixture->dev, 128U), -EIO);
	zassert_equal(adm1176_emul_get_threshold(fixture->emul), 128U);
	zassert_equal(adm1176_get_overcurrent_threshold(fixture->dev, &threshold),
		      -ENODATA);
	zassert_equal(adm1176_clear_alerts(fixture->dev), -ENODATA);
	zassert_ok(adm1176_set_overcurrent_threshold_raw(fixture->dev, 128U));
	zassert_ok(adm1176_clear_alerts(fixture->dev));
}

ZTEST_F(adm1176_driver, test_failed_clear_requires_explicit_configuration_retry)
{
	uint64_t threshold;

	adm1176_emul_fail_after(fixture->emul, 0U, -EIO);
	zassert_equal(adm1176_clear_alerts(fixture->dev), -EIO);
	zassert_equal(adm1176_get_overcurrent_threshold(fixture->dev, &threshold),
		      -ENODATA);
	zassert_equal(adm1176_clear_alerts(fixture->dev), -ENODATA);
	zassert_ok(adm1176_set_overcurrent_threshold_raw(fixture->dev, 128U));
	zassert_ok(adm1176_clear_alerts(fixture->dev));
}

ZTEST_F(adm1176_driver, test_failed_output_write_is_reported)
{
	adm1176_emul_fail_after(fixture->emul, 0U, -EIO);
	zassert_equal(adm1176_set_enabled(fixture->dev, false), -EIO);
	zassert_equal(adm1176_emul_get_control(fixture->emul), 0U);
}

ZTEST_SUITE(adm1176_driver, NULL, adm1176_driver_setup,
		    adm1176_driver_before, NULL, NULL);
