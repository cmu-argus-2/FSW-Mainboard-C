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

/* VMTR code 0xE4 represents 38 V (distinct from the target-voltage scale). */
#define DRV8235_VMTR_REFERENCE_UV   38000000U
#define DRV8235_VMTR_REFERENCE_CODE 0xE4U

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
	/* Aggregate FAULT, latched event flags, or an uncleared power-on reset. */
	bool any;
	bool stall;
	bool overcurrent;
	bool overvoltage;
	bool thermal_shutdown;
	/* NPOR is active low; a reset requires reconfiguration before driving. */
	bool power_on_reset;
	uint8_t raw;
};

struct drv8235_telemetry {
	int32_t voltage_uv;
	/* Coil-current magnitude, never battery-input current or a signed direction. */
	uint32_t current_ua;
	uint16_t duty_permille;
	uint8_t raw_voltage;
	uint8_t raw_current;
	uint8_t raw_duty;
	enum drv8235_bridge_mode bridge_mode;
	bool output_enabled;
	/* Requires an enabled drive/brake path, valid configuration/status and IMTR. */
	bool current_valid;
};

struct drv8235_limits {
	uint32_t max_output_uv;
	uint32_t coil_resistance_milliohm;
	/* Provisional calibration; confirm against hardware before using for control. */
	uint32_t imtr_full_scale_ua;
};

/**
 * Convert a signed voltage command into a target code and bridge direction.
 * Rounding is capped so the nominal encoded voltage never exceeds max_output_uv.
 * Commands rounding to code zero select coast.
 */
int drv8235_voltage_to_code(int32_t voltage_uv, uint32_t max_output_uv, uint8_t *code,
			    enum drv8235_bridge_mode *mode);

/** Convert a VMTR code to signed microvolts using the bridge direction. */
int drv8235_voltage_from_code(uint8_t code, enum drv8235_bridge_mode mode, int32_t *voltage_uv);

/**
 * Convert an IMTR code to unsigned coil-current magnitude in microamps.
 * This only converts units; it cannot establish whether the sensing path is
 * active or determine the current direction. Use get_telemetry's current_valid.
 * The scale remains provisional pending hardware confirmation.
 */
int drv8235_current_from_code(uint8_t code, uint32_t full_scale_ua, uint32_t *current_ua);

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
 * cleared. Overcurrent/overtemperature use latch-off recovery; stall detection
 * and soft start/stop are disabled for the coil load. Failure requires another
 * successful reconfigure before enabling or commanding a nonzero output.
 */
int drv8235_reconfigure(const struct device *dev);

/**
 * Enable or disable the H-bridge output. Neither operation changes COIL_EN.
 * Disabling also clears the target and selects coast. Enabling checks NPOR,
 * fault status, and the hardware configuration; -EAGAIN requires reconfigure.
 */
int drv8235_set_enabled(const struct device *dev, bool enabled);

/** Return whether the H-bridge output is enabled. */
int drv8235_is_enabled(const struct device *dev, bool *enabled);

/**
 * Command signed coil voltage in microvolts.
 *
 * Zero selects coast. Values outside the devicetree safety limit are rejected
 * with -ERANGE. Same-direction updates do not coast; direction changes coast
 * before applying the new target and direction. This is not a guarantee of zero
 * coil current during reversal. This call does not enable the output or rail.
 */
int drv8235_set_output_uv(const struct device *dev, int32_t voltage_uv);

/** Set target voltage to zero and place the bridge in high-impedance coast. */
int drv8235_coast(const struct device *dev);

/** Set target voltage to zero and actively brake the attached coil. */
int drv8235_brake(const struct device *dev);

/**
 * Read voltage, current, duty cycle, output enable, and bridge direction.
 * current_ua is a coil-current magnitude, not a signed measurement and not
 * battery-input current. Enabled low-side brake can still carry sensed current.
 * Coast, disabled output, reset/fault/unconfigured state, or IMTR above 0xC0
 * report current_valid=false. Other readings and raw_current are retained;
 * current_ua=0 then is only a placeholder, not a zero-current measurement.
 * Sensing-path validity does not establish calibration or sample freshness.
 * The current scale remains provisional pending hardware calibration.
 */
int drv8235_get_telemetry(const struct device *dev, struct drv8235_telemetry *telemetry);

/** Read fault flags without clearing the latched status. */
int drv8235_get_faults(const struct device *dev, struct drv8235_faults *faults);

/**
 * Disable/coast/zero the output before clearing latched faults. Never re-enables
 * the bridge. Clearing faults does not substitute for reconfigure after reset.
 */
int drv8235_clear_faults(const struct device *dev);

/** Read immutable safety/calibration limits from devicetree. */
int drv8235_get_limits(const struct device *dev, struct drv8235_limits *limits);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_DRV8235_H_ */
