/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ARGUS_DRIVERS_DRV8235_H_
#define ARGUS_DRIVERS_DRV8235_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

enum drv8235_bridge_mode {
	DRV8235_BRIDGE_COAST = 0,
	DRV8235_BRIDGE_REVERSE = 1,
	DRV8235_BRIDGE_FORWARD = 2,
	DRV8235_BRIDGE_BRAKE = 3,
};

struct drv8235_faults {
	uint8_t raw;
	bool stall;
	bool overcurrent;
	bool overvoltage;
	bool thermal_shutdown;
	/* NPOR is active low: a reset indication, not a UVLO measurement. */
	bool power_on_reset;
};

/**
 * Configure I2C/PWM voltage control, internal VREF and the original 25-ohm
 * coil compensation (INV_R_SCALE=2, INV_R=41). Finish enabled at target zero
 * in coast, as in Python/Rust. Call again after cycling the device supply.
 * The application owns COIL_EN and must provide stable power before this call.
 */
int drv8235_init(const struct device *dev);

/** Clamp throttle to [-1, 1], scaled to 6 V, truncating the register code.
 * Zero (including values too small to encode) selects brake. */
int drv8235_set_throttle(const struct device *dev, float throttle);

/** Clamp volts to [-42.7, 42.7], rounding the register code; zero brakes.
 * Like Python/Rust, this low-level interface does not impose the 6 V coil limit. */
int drv8235_set_throttle_volts(const struct device *dev, float volts);

/** Clamp signed raw command to [-255, 255]; zero brakes. No 6 V coil limit. */
int drv8235_set_throttle_raw(const struct device *dev, int16_t raw);

/** Target zero and high-impedance coast: the C equivalent of Python's None.
 * Does not disable EN_OUT or switch the shared supply. */
int drv8235_coast(const struct device *dev);

/** Read target settings, not measured voltage. Coast returns -ENODATA without
 * changing the output argument (Python/Rust return None); brake returns zero.
 * The normalized getter retains Python's intermediate rounding to 3 decimals. */
int drv8235_get_throttle(const struct device *dev, float *throttle);
int drv8235_get_throttle_volts(const struct device *dev, float *volts);
int drv8235_get_throttle_raw(const struct device *dev, int16_t *raw);
int drv8235_get_bridge_control(const struct device *dev, enum drv8235_bridge_mode *mode);

/**
 * Voltage: VMTR * 0.16733 V. Current: IMTR * 3.7 / 192 A, following the team's
 * EPS conversion exception to Python. Both are negated in reverse; the sign
 * is inferred from the command, not measured.
 * Brake does not force current to zero. Coast may return stale/unsensed current;
 * these readings do not establish sensing validity or battery-input current.
 * Current calibration remains provisional pending hardware confirmation.
 * Output arguments are unchanged on an I2C error.
 */
int drv8235_read_voltage_current(const struct device *dev, float *volts, float *amps);

/** Read flags and, if FAULT is set, clear them afterwards as in Python/Rust.
 * Clearing first selects zero/coast to avoid restarting the previous command.
 * NPOR decoding is corrected; a detected reset requires init before driving. */
int drv8235_get_faults(const struct device *dev, struct drv8235_faults *faults);

/** Select zero/coast and clear faults. Does not replace init after a reset. */
int drv8235_clear_faults(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ARGUS_DRIVERS_DRV8235_H_ */
