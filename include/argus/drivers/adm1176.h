/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ARGUS_DRIVERS_ADM1176_H_
#define ARGUS_DRIVERS_ADM1176_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

enum adm1176_command {
	ADM1176_V_CONT = 0x01,
	ADM1176_V_ONCE = 0x02,
	ADM1176_I_CONT = 0x04,
	ADM1176_I_ONCE = 0x08,
	ADM1176_VRANGE = 0x10,
};

enum adm1176_status_bit {
	ADM1176_STATUS_ADC_OVERCURRENT = 1U << 0,
	ADM1176_STATUS_ADC_ALERT = 1U << 1,
	ADM1176_STATUS_HOT_SWAP_OVERCURRENT = 1U << 2,
	ADM1176_STATUS_HOT_SWAP_ALERT = 1U << 3,
	ADM1176_STATUS_OFF = 1U << 4,
	ADM1176_STATUS_OFF_ALERT = 1U << 5,
};

/* Same numeric code as Python Errors.PWR_MON_ADC_ALERT_OVERCURRENT. */
enum adm1176_error_code {
	ADM1176_PWR_MON_ADC_ALERT_OVERCURRENT = 0x16,
};

/* Python device_errors returns [] or a list containing this one error. */
struct adm1176_errors {
	uint8_t count;
	uint8_t codes[1];
};

/* Voltage/current use sensor_sample_fetch() and sensor_channel_get(). */
int adm1176_config(const struct device *dev, uint8_t command);
int adm1176_status(const struct device *dev, uint8_t *status);
/* ON/SWOFF status, not a guarantee that the downstream rail is powered. */
int adm1176_device_on(const struct device *dev, bool *on);
int adm1176_set_device_on(const struct device *dev, bool on);
int adm1176_set_overcurrent_level(const struct device *dev, uint8_t level);
/* Python-compatible software cache: initially 0xFF, updated on success.
 * Not hardware readback: write-only registers may survive an MCU reboot.
 */
uint8_t adm1176_overcurrent_level(const struct device *dev);
/* Write this boot's alert mask (initially hot-swap only) with CLEAR set. */
int adm1176_clear(const struct device *dev);
/* Fill the error-code list and clear alerts when ADC_ALERT is present.
 * Return 0 on success or a negative I2C error. A detected code remains in
 * the list if clearing fails; callers must still check the return value.
 */
int adm1176_device_errors(const struct device *dev, struct adm1176_errors *errors);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_ADM1176_H_ */
