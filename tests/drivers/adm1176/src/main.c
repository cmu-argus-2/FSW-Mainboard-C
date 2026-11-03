/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>
#include "adm1176_emul.h"
#include "argus/drivers/adm1176.h"

static const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(adm1176_test));
static const struct emul *const emul = EMUL_DT_GET(DT_NODELABEL(adm1176_test));
static struct adm1176_emul_state *s;

static void *setup(void)
{
	s = adm1176_emul_state(emul);
	zassert_true(device_is_ready(dev));
	zassert_equal(s->command, 0x05);
	zassert_equal(s->alert_enable, 0x04);
	zassert_equal(s->control, 0);
	zassert_equal(adm1176_overcurrent_level(dev), 0xFF);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	adm1176_emul_reset(emul);
	zassert_ok(adm1176_config(dev, ADM1176_V_CONT | ADM1176_I_CONT));
	zassert_ok(adm1176_set_overcurrent_level(dev, 0xFF));
}

ZTEST(adm1176, test_voltage_current_and_12_bit_decode)
{
	struct sensor_value value;

	s->sample[0] = 0x80;
	s->sample[1] = 0x40; /* Regression: widen the high byte before shifting. */
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 13175000);
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &value));
	zassert_equal(sensor_value_to_micro(&value), 2646000);
	s->sample[0] = 0xFF;
	s->sample[1] = 0xFF;
	s->sample[2] = 0xFF;
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 26343566);
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &value));
	zassert_equal(sensor_value_to_micro(&value), 10581416);
}

ZTEST(adm1176, test_low_range_and_once_configuration)
{
	struct sensor_value value;

	zassert_ok(adm1176_config(dev, ADM1176_V_ONCE | ADM1176_I_ONCE | ADM1176_VRANGE));
	zassert_equal(s->command, 0x1A);
	s->sample[0] = 0x80;
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 3325000);
}

ZTEST(adm1176, test_single_channel_read_formats)
{
	struct sensor_value value;

	s->sample[0] = 0x80;
	s->sample[1] = 0x40;
	s->sample[2] = 0xAB;
	zassert_ok(adm1176_config(dev, ADM1176_V_CONT));
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value));
	zassert_equal(sensor_value_to_micro(&value), 13239331);
	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &value), -ENOTSUP);
	zassert_ok(adm1176_config(dev, ADM1176_I_CONT));
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &value));
	zassert_equal(sensor_value_to_micro(&value), 2674423);
}

ZTEST(adm1176, test_status_preserves_measurement_configuration)
{
	uint8_t status;

	zassert_ok(adm1176_config(dev, 0x15));
	s->status = 0x3F;
	zassert_ok(adm1176_status(dev, &status));
	zassert_equal(status, 0x3F);
	zassert_equal(s->command, 0x15);
}

ZTEST(adm1176, test_output_switching_and_i2c_error)
{
	bool on;

	zassert_ok(adm1176_set_device_on(dev, false));
	zassert_equal(s->control, 1);
	zassert_ok(adm1176_device_on(dev, &on));
	zassert_false(on);
	zassert_ok(adm1176_set_device_on(dev, true));
	zassert_equal(s->command, 0x05);
	zassert_ok(adm1176_device_on(dev, &on));
	zassert_true(on);
	s->error = -EIO;
	zassert_equal(adm1176_set_device_on(dev, false), -EIO);
	zassert_equal(s->control, 0);
}

ZTEST(adm1176, test_raw_threshold_and_clear)
{
	zassert_ok(adm1176_set_overcurrent_level(dev, 128));
	zassert_equal(s->threshold, 128);
	zassert_equal(adm1176_overcurrent_level(dev), 128);
	s->status = 0x2A;
	zassert_ok(adm1176_clear(dev));
	zassert_equal(s->status, 0);
	zassert_equal(s->alert_enable, 0x06);
	zassert_equal(adm1176_overcurrent_level(dev), 128);
}

ZTEST(adm1176, test_rejects_invalid_config_and_unsupported_channels)
{
	struct sensor_value value;

	zassert_equal(adm1176_config(dev, 0x40), -EINVAL);
	zassert_equal(s->command, 0x05);
	zassert_equal(sensor_sample_fetch_chan(dev, SENSOR_CHAN_POWER), -ENOTSUP);
	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_POWER, &value), -ENOTSUP);
	s->error = -EIO;
	zassert_equal(adm1176_config(dev, 0x15), -EIO);
	zassert_ok(sensor_sample_fetch(dev));
	zassert_equal(s->command, 0x05);
	zassert_ok(adm1176_config(dev, 0));
	zassert_equal(sensor_sample_fetch(dev), -ENOTSUP);
}

ZTEST(adm1176, test_failed_sample_invalidates_cache_and_recovers)
{
	struct sensor_value value;

	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value), -ENODATA);
	zassert_ok(sensor_sample_fetch(dev));
	s->error = -EIO;
	zassert_equal(sensor_sample_fetch(dev), -EIO);
	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value), -ENODATA);
	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &value));
}

ZTEST(adm1176, test_status_restore_failure_recovers_on_next_sample)
{
	uint8_t status;

	s->error = -EIO;
	s->skip = 2;
	zassert_equal(adm1176_status(dev, &status), -EIO);
	zassert_equal(s->command, 0x45);
	zassert_ok(sensor_sample_fetch(dev));
	zassert_equal(s->command, 0x05);
}

ZTEST(adm1176, test_status_read_failure_still_restores_command)
{
	uint8_t status;

	s->error = -ETIMEDOUT;
	s->skip = 1;
	zassert_equal(adm1176_status(dev, &status), -ETIMEDOUT);
	zassert_equal(s->command, 0x05);
}

ZTEST(adm1176, test_failed_threshold_writes_keep_last_successful_cache)
{
	zassert_ok(adm1176_set_overcurrent_level(dev, 128));
	for (unsigned int skip = 0; skip < 2; skip++) {
		s->error = -EIO;
		s->skip = skip;
		zassert_equal(adm1176_set_overcurrent_level(dev, 200), -EIO);
		zassert_equal(adm1176_overcurrent_level(dev), 128);
	}
	zassert_ok(adm1176_set_overcurrent_level(dev, 200));
	zassert_equal(adm1176_overcurrent_level(dev), 200);
}

ZTEST(adm1176, test_device_errors_reports_and_clears_adc_alert)
{
	struct adm1176_errors errors;

	s->status = ADM1176_STATUS_ADC_ALERT;
	zassert_ok(adm1176_device_errors(dev, &errors));
	zassert_equal(errors.count, 1);
	zassert_equal(errors.codes[0], 0x16); /* Original Python error code. */
	zassert_equal(s->status, 0);
	zassert_ok(adm1176_device_errors(dev, &errors));
	zassert_equal(errors.count, 0);
	s->status = ADM1176_STATUS_HOT_SWAP_ALERT;
	zassert_ok(adm1176_device_errors(dev, &errors));
	zassert_equal(errors.count, 0); /* Python reports only ADC_ALERT here. */
	zassert_equal(s->status, ADM1176_STATUS_HOT_SWAP_ALERT);
}

ZTEST(adm1176, test_device_errors_propagates_read_and_clear_failures)
{
	struct adm1176_errors errors = {.count = 1, .codes = {0x16}};

	s->status = ADM1176_STATUS_ADC_ALERT;
	s->error = -ETIMEDOUT;
	s->skip = 1;
	zassert_equal(adm1176_device_errors(dev, &errors), -ETIMEDOUT);
	zassert_equal(errors.count, 0);
	zassert_equal(s->status, ADM1176_STATUS_ADC_ALERT);
	s->status = ADM1176_STATUS_ADC_ALERT;
	s->error = -EIO;
	s->skip = 3; /* Status succeeds; the following CLEAR write fails. */
	zassert_equal(adm1176_device_errors(dev, &errors), -EIO);
	zassert_equal(errors.count, 1);
	zassert_equal(errors.codes[0], 0x16);
	zassert_equal(s->status, ADM1176_STATUS_ADC_ALERT);
}

ZTEST_SUITE(adm1176, NULL, setup, before, NULL, NULL);
