/*
 * Copyright (c) 2026 Carnegie Mellon University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adm1176_common.c
 * @brief Hardware-independent ADM1176 conversions and status decoding.
 */

#include "argus/drivers/adm1176.h"

#include <errno.h>
#include <stddef.h>

#define ADM1176_ALERT_STEPS 256U
#define MICRO_UNITS_PER_UNIT 1000000ULL

static uint64_t rounded_divide(uint64_t numerator, uint64_t denominator)
{
	return (numerator + (denominator / 2U)) / denominator;
}

static bool voltage_range_is_valid(uint32_t voltage_range_microvolt)
{
	return voltage_range_microvolt == ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV ||
	       voltage_range_microvolt == ADM1176_VOLTAGE_FULL_SCALE_LOW_UV;
}

static uint64_t full_scale_current_ua(uint32_t rshunt_micro_ohms)
{
	return ((uint64_t)ADM1176_CURRENT_SENSE_FULL_SCALE_UV *
		MICRO_UNITS_PER_UNIT) /
	       rshunt_micro_ohms;
}

int adm1176_decode_measurement(const uint8_t buffer[3],
			       uint32_t rshunt_micro_ohms,
			       uint32_t voltage_range_microvolt,
			       struct adm1176_sample *sample)
{
	if (buffer == NULL || sample == NULL || rshunt_micro_ohms == 0U ||
	    !voltage_range_is_valid(voltage_range_microvolt)) {
		return -EINVAL;
	}

	sample->raw_voltage = ((uint16_t)buffer[0] << 4) |
			      ((uint16_t)buffer[2] >> 4);
	sample->raw_current = ((uint16_t)buffer[1] << 4) |
			      ((uint16_t)buffer[2] & 0x0FU);

	sample->voltage_uv = rounded_divide(
		(uint64_t)sample->raw_voltage * voltage_range_microvolt,
		ADM1176_ADC_STEPS);
	sample->current_ua = rounded_divide(
		(uint64_t)sample->raw_current * full_scale_current_ua(rshunt_micro_ohms),
		ADM1176_ADC_STEPS);
	sample->power_uw = rounded_divide(sample->voltage_uv * sample->current_ua,
					 MICRO_UNITS_PER_UNIT);

	return 0;
}

int adm1176_decode_status(uint8_t raw, struct adm1176_status_flags *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	status->raw = raw;
	status->adc_overcurrent = (raw & ADM1176_STATUS_ADC_OVERCURRENT) != 0U;
	status->adc_alert = (raw & ADM1176_STATUS_ADC_ALERT) != 0U;
	status->hot_swap_overcurrent =
		(raw & ADM1176_STATUS_HOT_SWAP_OVERCURRENT) != 0U;
	status->hot_swap_alert = (raw & ADM1176_STATUS_HOT_SWAP_ALERT) != 0U;
	status->output_off = (raw & ADM1176_STATUS_OFF) != 0U;
	status->output_off_alert = (raw & ADM1176_STATUS_OFF_ALERT) != 0U;

	return 0;
}

int adm1176_threshold_from_microamps(uint64_t current_ua,
				    uint32_t rshunt_micro_ohms,
				    uint8_t *threshold)
{
	if (rshunt_micro_ohms == 0U || threshold == NULL) {
		return -EINVAL;
	}

	const uint64_t full_scale_ua = full_scale_current_ua(rshunt_micro_ohms);

	if (current_ua > full_scale_ua) {
		return -ERANGE;
	}

	uint64_t raw = rounded_divide(current_ua * ADM1176_ALERT_STEPS,
				      full_scale_ua);

	if (raw > UINT8_MAX) {
		raw = UINT8_MAX;
	}
	*threshold = (uint8_t)raw;

	return 0;
}

int adm1176_threshold_to_microamps(uint8_t threshold,
				  uint32_t rshunt_micro_ohms,
				  uint64_t *current_ua)
{
	if (rshunt_micro_ohms == 0U || current_ua == NULL) {
		return -EINVAL;
	}

	*current_ua = rounded_divide(
		full_scale_current_ua(rshunt_micro_ohms) * threshold,
		ADM1176_ALERT_STEPS);

	return 0;
}
