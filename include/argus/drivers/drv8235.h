/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file drv8235.h
 * @brief Safe Zephyr interface for Argus DRV8235 magnetorquer drivers.
 */

#ifndef ARGUS_DRIVERS_DRV8235_H_
#define ARGUS_DRIVERS_DRV8235_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WSET_VSET code 255 represents approximately 42.67 V. */
#define DRV8235_TARGET_FULL_SCALE_UV   42670000U
#define DRV8235_TARGET_FULL_SCALE_CODE UINT8_MAX

/* The data sheet defines IMTR code 0xC0 as the maximum current value. */
#define DRV8235_IMTR_FULL_SCALE_CODE 0xC0U
#define DRV8235_DUTY_FULL_SCALE_CODE 0x3FU

enum drv8235_bridge_mode {
	DRV8235_BRIDGE_COAST = 0,
	DRV8235_BRIDGE_REVERSE = 1,
	DRV8235_BRIDGE_FORWARD = 2,
	DRV8235_BRIDGE_BRAKE = 3,
};

struct drv8235_faults {
	bool any;
	bool stall;
	bool overcurrent;
	bool overvoltage;
	bool thermal_shutdown;
	bool undervoltage_lockout;
	uint8_t raw;
};

struct drv8235_telemetry {
	int32_t voltage_uv;
	int32_t current_ua;
	uint16_t duty_permille;
	uint8_t raw_voltage;
	uint8_t raw_current;
	uint8_t raw_duty;
	enum drv8235_bridge_mode bridge_mode;
	bool output_enabled;
};

struct drv8235_limits {
	uint32_t max_output_uv;
	uint32_t coil_resistance_milliohm;
	uint32_t imtr_full_scale_ua;
};

/** Convert a signed voltage command into a target code and bridge direction. */
int drv8235_voltage_to_code(int32_t voltage_uv, uint32_t max_output_uv, uint8_t *code,
			    enum drv8235_bridge_mode *mode);

/** Convert a VMTR code to signed microvolts using the bridge direction. */
int drv8235_voltage_from_code(uint8_t code, enum drv8235_bridge_mode mode, int32_t *voltage_uv);

/** Convert an IMTR code to signed microamps using the bridge direction. */
int drv8235_current_from_code(uint8_t code, uint32_t full_scale_ua, enum drv8235_bridge_mode mode,
			      int32_t *current_ua);

/** Convert DUTY_READ (0..63) to duty cycle in per-mille (0..1000). */
int drv8235_duty_from_code(uint8_t code, uint16_t *duty_permille);

/** Decode FAULT_STATUS without clearing it. */
int drv8235_decode_faults(uint8_t raw, struct drv8235_faults *faults);

/** Calculate the closest valid INV_R_SCALE and INV_R values. */
int drv8235_calculate_inv_r(uint32_t coil_resistance_milliohm, uint8_t *scale_code, uint8_t *inv_r);

/**
 * Restore the safe driver configuration after the COIL_EN rail was cycled.
 *
 * The rail must be stable before this call. On success the output is disabled,
 * the target is zero, the bridge is coasting, and latched power-up faults are
 * cleared.
 */
int drv8235_reconfigure(const struct device *dev);

/** Enable or disable the H-bridge output. Enabling never changes COIL_EN. */
int drv8235_set_enabled(const struct device *dev, bool enabled);

/** Return whether the H-bridge output is enabled. */
int drv8235_is_enabled(const struct device *dev, bool *enabled);

/**
 * Command signed coil voltage in microvolts.
 *
 * Zero selects coast. Values outside the devicetree safety limit are rejected
 * with -ERANGE. This call does not enable the output or the shared COIL_EN rail.
 */
int drv8235_set_output_uv(const struct device *dev, int32_t voltage_uv);

/** Set target voltage to zero and place the bridge in high-impedance coast. */
int drv8235_coast(const struct device *dev);

/** Set target voltage to zero and actively brake the attached coil. */
int drv8235_brake(const struct device *dev);

/** Read voltage, current, duty cycle, output enable, and bridge direction. */
int drv8235_get_telemetry(const struct device *dev, struct drv8235_telemetry *telemetry);

/** Read fault flags without clearing the latched status. */
int drv8235_get_faults(const struct device *dev, struct drv8235_faults *faults);

/** Clear all latched faults. */
int drv8235_clear_faults(const struct device *dev);

/** Read immutable safety/calibration limits from devicetree. */
int drv8235_get_limits(const struct device *dev, struct drv8235_limits *limits);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_DRV8235_H_ */
