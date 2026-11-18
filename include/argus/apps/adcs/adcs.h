/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_APPS_ADCS_ADCS_H_
#define ARGUS_APPS_ADCS_ADCS_H_

#include <stdbool.h>

/* Pure ADCS algorithms ported from Python flight/apps/adcs (acs.py, sun.py,
 * sensors.py, consts.py). No hardware access. Vectors are body frame.
 * Numeric enum values go on the wire and match the Python constants.
 */

/* StatusConst */
enum adcs_status {
	ADCS_STATUS_OK = 0,
	ADCS_STATUS_GYRO_FAIL = 21,
	ADCS_STATUS_MAG_FAIL = 31,
	ADCS_STATUS_SUN_NO_READINGS = 51,
	ADCS_STATUS_SUN_NOT_ENOUGH_READINGS = 52,
	ADCS_STATUS_SUN_ECLIPSE = 53,
	ADCS_STATUS_ZERO_NORM = 61,
};

/* Modes */
enum adcs_mode {
	ADCS_MODE_TUMBLING = 0,
	ADCS_MODE_STABLE = 1,
	ADCS_MODE_SUN_POINTING = 2,
	ADCS_MODE_ACS_OFF = 3,
	ADCS_MODE_VF_TUMBLING = 4,
};

/* ControllerModes */
enum adcs_controller {
	ADCS_CTRL_BDOT = 0,
	ADCS_CTRL_BCROSS = 1,
	ADCS_CTRL_SUN_POINTING = 2,
};

/* Light sensors in Python sun.py _FACES order (also the telemetry order). */
enum adcs_face {
	ADCS_FACE_XP, ADCS_FACE_XM, ADCS_FACE_YP, ADCS_FACE_YM,
	ADCS_FACE_ZP_XP, ADCS_FACE_ZP_YM, ADCS_FACE_ZP_XM, ADCS_FACE_ZP_YP,
	ADCS_FACE_ZM, ADCS_LIGHT_COUNT,
};

/* Torque coils in Python acs.py _MCM_FACES order. Axis n owns coils 2n, 2n+1. */
enum adcs_coil {
	ADCS_COIL_XP, ADCS_COIL_XM, ADCS_COIL_YP, ADCS_COIL_YM,
	ADCS_COIL_ZP, ADCS_COIL_ZM, ADCS_COIL_COUNT,
};

/* Lux placeholder for an unreadable sensor (Python _ERROR_LUX). */
#define ADCS_LUX_ERROR (-1.0f)
#define ADCS_BDOT_SAMPLES 6

/* Validate a gyro sample in rad/s. Rejects |w| > 2000 deg/s and NaN.
 * On failure out is zeroed and ADCS_STATUS_GYRO_FAIL is returned.
 */
enum adcs_status adcs_check_gyro(const float raw_rad_s[3], float out_rad_s[3]);

/* Convert a magnetometer sample from uT to T, subtract bias_t, multiply by
 * scale, then require 1 uT <= |B| <= 2500 uT. On failure out is zeroed and
 * ADCS_STATUS_MAG_FAIL is returned.
 */
enum adcs_status adcs_correct_mag(const float raw_ut[3], const float bias_t[3],
				  const float scale[3], float out_t[3]);

/* Weighted least-squares unit sun vector from lux in adcs_face order. Entries
 * equal to ADCS_LUX_ERROR count as missing. sun is zeroed unless status is OK.
 */
enum adcs_status adcs_sun_vector(const float lux[ADCS_LIGHT_COUNT], float sun[3]);

/* Control laws. Each writes a throttle vector u with |u| < 1 (sun pointing:
 * unit or zero). Zero is the Python FALLBACK_CONTROL.
 */
void adcs_spin_stabilizing(const float omega[3], const float mag[3], float u[3]);
void adcs_sun_pointing(const float sun[3], const float omega[3], const float mag[3],
		       float u[3]);
/* mag_buffer row 0 is the oldest sample; dt_s is the uniform sample spacing. */
void adcs_bdot(const float mag_buffer[ADCS_BDOT_SAMPLES][3], float dt_s, float u[3]);
void adcs_bcross(const float mag[3], const float omega[3], float u[3]);

/* MCM coil allocation (Python mcm_coil_allocator without the hardware calls).
 * Compensates a single fully failed axis by projecting along b, then splits
 * each axis between its available coils under the power limit. Coils that are
 * not available get 0 and must not be commanded.
 */
void adcs_allocate(const float u[3], const float b[3], const bool available[ADCS_COIL_COUNT],
		   float throttle[ADCS_COIL_COUNT]);

/* Mode transition logic (Python sensors.update_mode). */
enum adcs_mode adcs_next_mode(enum adcs_mode current, enum adcs_controller controller,
			      enum adcs_status gyro_status, const float omega[3],
			      enum adcs_status sun_status, const float sun[3]);

#endif
