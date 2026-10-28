/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file adm1176.h
 * @brief Zephyr interface for the ADM1176 power monitor and hot-swap controller.
 */

#ifndef ARGUS_DRIVERS_ADM1176_H_
#define ARGUS_DRIVERS_ADM1176_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADM1176_ADC_STEPS 4096U
#define ADM1176_VOLTAGE_FULL_SCALE_HIGH_UV 26350000U
#define ADM1176_VOLTAGE_FULL_SCALE_LOW_UV 6650000U
#define ADM1176_CURRENT_SENSE_FULL_SCALE_UV 105840U

enum adm1176_status_bit {
	ADM1176_STATUS_ADC_OVERCURRENT = 1U << 0,
	ADM1176_STATUS_ADC_ALERT = 1U << 1,
	ADM1176_STATUS_HOT_SWAP_OVERCURRENT = 1U << 2,
	ADM1176_STATUS_HOT_SWAP_ALERT = 1U << 3,
	ADM1176_STATUS_OFF = 1U << 4,
	ADM1176_STATUS_OFF_ALERT = 1U << 5,
};

struct adm1176_sample {
	uint16_t raw_voltage;
	uint16_t raw_current;
	uint64_t voltage_uv;
	uint64_t current_ua;
	uint64_t power_uw;
};

struct adm1176_status_flags {
	bool adc_overcurrent;
	bool adc_alert;
	bool hot_swap_overcurrent;
	bool hot_swap_alert;
	bool output_off;
	bool output_off_alert;
	uint8_t raw;
};

/** Decode one three-byte ADM1176 measurement without accessing hardware. */
int adm1176_decode_measurement(const uint8_t buffer[3],
			       uint32_t rshunt_micro_ohms,
			       uint32_t voltage_range_microvolt,
			       struct adm1176_sample *sample);

/* Fetch and decode one transaction under the device lock. Includes the raw
 * ADC codes for applications with a prescribed conversion/truncation policy.
 */
int adm1176_read_sample(const struct device *dev, struct adm1176_sample *sample);

/** Decode the ADM1176 status register into named flags. */
int adm1176_decode_status(uint8_t raw, struct adm1176_status_flags *status);

/** Convert an overcurrent limit in microamps to the ADM1176's 8-bit threshold. */
int adm1176_threshold_from_microamps(uint64_t current_ua,
				    uint32_t rshunt_micro_ohms,
				    uint8_t *threshold);

/** Convert an ADM1176 8-bit overcurrent threshold to microamps. */
int adm1176_threshold_to_microamps(uint8_t threshold,
				  uint32_t rshunt_micro_ohms,
				  uint64_t *current_ua);

int adm1176_read_status(const struct device *dev, uint8_t *status);
int adm1176_get_status_flags(const struct device *dev,
			     struct adm1176_status_flags *status);
int adm1176_is_enabled(const struct device *dev, bool *enabled);
int adm1176_set_enabled(const struct device *dev, bool enabled);
int adm1176_set_overcurrent_threshold_raw(const struct device *dev,
					 uint8_t threshold);
int adm1176_set_overcurrent_threshold(const struct device *dev,
				      uint64_t current_ua);
int adm1176_get_overcurrent_threshold(const struct device *dev,
				      uint64_t *current_ua);
int adm1176_clear_alerts(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_ADM1176_H_ */
