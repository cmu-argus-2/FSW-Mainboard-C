/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "argus/drivers/adm1176.h"

#define TEST_RSHUNT_UOHM 10000U
#define TEST_FULL_SCALE_CURRENT_UA 10584000ULL

ZTEST(adm1176, test_decode_rejects_invalid_arguments)
{
	const uint8_t buffer[3] = {0};
	struct adm1176_sample sample;

	zassert_equal(adm1176_decode_measurement(NULL, TEST_RSHUNT_UOHM,
						 ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
						 &sample), -EINVAL);
	zassert_equal(adm1176_decode_measurement(buffer, 0U,
						 ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
						 &sample), -EINVAL);
	zassert_equal(adm1176_decode_measurement(buffer, TEST_RSHUNT_UOHM, 1U,
						 &sample), -EINVAL);
	zassert_equal(adm1176_decode_measurement(buffer, TEST_RSHUNT_UOHM,
						 ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
						 NULL), -EINVAL);
}

ZTEST(adm1176, test_decode_zero_sample)
{
	const uint8_t buffer[3] = {0};
	struct adm1176_sample sample;

	zassert_ok(adm1176_decode_measurement(
		buffer, TEST_RSHUNT_UOHM, ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
		&sample));
	zassert_equal(sample.raw_voltage, 0U);
	zassert_equal(sample.raw_current, 0U);
	zassert_equal(sample.voltage_uv, 0U);
	zassert_equal(sample.current_ua, 0U);
	zassert_equal(sample.power_uw, 0U);
}

ZTEST(adm1176, test_decode_packs_nibbles_in_datasheet_order)
{
	const uint8_t buffer[3] = {0x12, 0x34, 0xAB};
	struct adm1176_sample sample;

	zassert_ok(adm1176_decode_measurement(
		buffer, TEST_RSHUNT_UOHM, ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
		&sample));
	zassert_equal(sample.raw_voltage, 0x12AU);
	zassert_equal(sample.raw_current, 0x34BU);
}

ZTEST(adm1176, test_decode_high_range_full_scale_code)
{
	const uint8_t buffer[3] = {0xFF, 0xFF, 0xFF};
	struct adm1176_sample sample;

	zassert_ok(adm1176_decode_measurement(
		buffer, TEST_RSHUNT_UOHM, ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
		&sample));
	zassert_equal(sample.raw_voltage, 4095U);
	zassert_equal(sample.raw_current, 4095U);
	zassert_within(sample.voltage_uv, 26343567ULL, 1ULL);
	zassert_within(sample.current_ua, 10581416ULL, 1ULL);
}

ZTEST(adm1176, test_decode_low_voltage_range)
{
	const uint8_t buffer[3] = {0x80, 0x00, 0x00};
	struct adm1176_sample sample;

	zassert_ok(adm1176_decode_measurement(
		buffer, TEST_RSHUNT_UOHM, ADM1176_VOLTAGE_FULL_SCALE_LOW_UV,
		&sample));
	zassert_equal(sample.raw_voltage, 2048U);
	zassert_equal(sample.voltage_uv, 3325000ULL);
}

ZTEST(adm1176, test_decode_calculates_power)
{
	const uint8_t buffer[3] = {0x80, 0x40, 0x00};
	struct adm1176_sample sample;

	zassert_ok(adm1176_decode_measurement(
		buffer, TEST_RSHUNT_UOHM, ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV,
		&sample));
	zassert_equal(sample.voltage_uv, 13175000ULL);
	zassert_equal(sample.current_ua, 2646000ULL);
	zassert_equal(sample.power_uw, 34861050ULL);
}

ZTEST(adm1176, test_status_rejects_null_output)
{
	zassert_equal(adm1176_decode_status(0U, NULL), -EINVAL);
}

ZTEST(adm1176, test_status_decodes_each_flag)
{
	struct adm1176_status_flags status;

	zassert_ok(adm1176_decode_status(0x3FU, &status));
	zassert_equal(status.raw, 0x3FU);
	zassert_true(status.adc_overcurrent);
	zassert_true(status.adc_alert);
	zassert_true(status.hot_swap_overcurrent);
	zassert_true(status.hot_swap_alert);
	zassert_true(status.output_off);
	zassert_true(status.output_off_alert);
}

ZTEST(adm1176, test_status_leaves_unset_flags_false)
{
	struct adm1176_status_flags status;

	zassert_ok(adm1176_decode_status(ADM1176_STATUS_ADC_ALERT, &status));
	zassert_false(status.adc_overcurrent);
	zassert_true(status.adc_alert);
	zassert_false(status.hot_swap_overcurrent);
	zassert_false(status.hot_swap_alert);
	zassert_false(status.output_off);
	zassert_false(status.output_off_alert);
}

ZTEST(adm1176, test_threshold_rejects_invalid_arguments)
{
	uint8_t threshold;
	uint64_t current_ua;

	zassert_equal(adm1176_threshold_from_microamps(0U, 0U, &threshold),
		      -EINVAL);
	zassert_equal(adm1176_threshold_from_microamps(0U, TEST_RSHUNT_UOHM,
						      NULL), -EINVAL);
	zassert_equal(adm1176_threshold_to_microamps(0U, 0U, &current_ua),
		      -EINVAL);
	zassert_equal(adm1176_threshold_to_microamps(0U, TEST_RSHUNT_UOHM,
						    NULL), -EINVAL);
}

ZTEST(adm1176, test_threshold_zero)
{
	uint8_t threshold = UINT8_MAX;
	uint64_t current_ua = UINT64_MAX;

	zassert_ok(adm1176_threshold_from_microamps(0U, TEST_RSHUNT_UOHM,
						    &threshold));
	zassert_equal(threshold, 0U);
	zassert_ok(adm1176_threshold_to_microamps(0U, TEST_RSHUNT_UOHM,
						  &current_ua));
	zassert_equal(current_ua, 0U);
}

ZTEST(adm1176, test_threshold_half_scale_round_trip)
{
	uint8_t threshold;
	uint64_t current_ua;

	zassert_ok(adm1176_threshold_from_microamps(
		TEST_FULL_SCALE_CURRENT_UA / 2U, TEST_RSHUNT_UOHM,
		&threshold));
	zassert_equal(threshold, 128U);
	zassert_ok(adm1176_threshold_to_microamps(
		threshold, TEST_RSHUNT_UOHM, &current_ua));
	zassert_equal(current_ua, TEST_FULL_SCALE_CURRENT_UA / 2U);
}

ZTEST(adm1176, test_threshold_full_scale_saturates)
{
	uint8_t threshold;

	zassert_ok(adm1176_threshold_from_microamps(
		TEST_FULL_SCALE_CURRENT_UA, TEST_RSHUNT_UOHM, &threshold));
	zassert_equal(threshold, UINT8_MAX);
}

ZTEST(adm1176, test_threshold_rejects_above_full_scale)
{
	uint8_t threshold;

	zassert_equal(adm1176_threshold_from_microamps(
		TEST_FULL_SCALE_CURRENT_UA + 1U, TEST_RSHUNT_UOHM,
		&threshold), -ERANGE);
}

ZTEST_SUITE(adm1176, NULL, NULL, NULL, NULL, NULL);
