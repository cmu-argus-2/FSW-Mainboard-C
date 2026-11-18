/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/apps/adcs/adcs.h"

#include <math.h>
#include <string.h>

/* ControllerConst */
static const float inertia[3][3] = {
	{3.544e-03f, -1.8729e-05f, -5.2467e-06f},
	{-1.8729e-05f, 3.590e-03f, 1.9134e-05f},
	{-5.2467e-06f, 1.9134e-05f, 4.120e-03f},
};
static const float inertia_major_dir[3] = {-0.01027212f, 0.03638753f, 0.99928496f};
#define OMEGA_MAG_TARGET 0.35f
#define SPIN_STABILIZING_GAIN 2.0e07f
#define DETUMB_GAIN 1.0e05f

/* Modes thresholds, rad/s or unitless vector-error norms. */
#define VF_TUMBLING_TOL_BDOT 2.62f
#define VF_TUMBLING_TOL 3.05f
#define TUMBLING_TOL 0.54f
#define DETUMBLED_TOL_LO 0.070f
#define DETUMBLED_TOL_HI 0.087f
#define STABLE_TOL_LO 0.26f
#define STABLE_TOL_HI 0.34f
#define SUN_POINTED_TOL_LO 0.176f
#define SUN_POINTED_TOL_HI 0.26f

/* sensors.py limits */
#define MIN_MAG_NORM_T 1.0e-6f
#define MAX_MAG_NORM_T 2.5e-3f
#define MAX_GYRO_NORM_RAD_S (2.0e3f * 3.14159265358979f / 180.0f)

/* sun.py */
#define THRESHOLD_ILLUMINATION_LUX 3000.0f
#define INV_SQRT2 0.70710678118f

static float norm3(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void cross3(const float a[3], const float b[3], float out[3])
{
	float x = a[1] * b[2] - a[2] * b[1];
	float y = a[2] * b[0] - a[0] * b[2];
	float z = a[0] * b[1] - a[1] * b[0];

	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static void inertia_times(const float v[3], float out[3])
{
	for (int i = 0; i < 3; ++i) {
		out[i] = inertia[i][0] * v[0] + inertia[i][1] * v[1] + inertia[i][2] * v[2];
	}
}

static void momentum_target(float out[3])
{
	const float omega[3] = {
		inertia_major_dir[0] * OMEGA_MAG_TARGET,
		inertia_major_dir[1] * OMEGA_MAG_TARGET,
		inertia_major_dir[2] * OMEGA_MAG_TARGET,
	};

	inertia_times(omega, out);
}

/* u / sqrt(|u|^2 + 1): smooth bound on the throttle norm. */
static void soft_clip(float u[3])
{
	float n = norm3(u);
	float d = sqrtf(n * n + 1.0f);

	for (int i = 0; i < 3; ++i) {
		u[i] /= d;
	}
}

static void zero3(float v[3])
{
	v[0] = v[1] = v[2] = 0.0f;
}

enum adcs_status adcs_check_gyro(const float raw_rad_s[3], float out_rad_s[3])
{
	/* Negated comparison also rejects NaN, as in Python. */
	if (!(norm3(raw_rad_s) <= MAX_GYRO_NORM_RAD_S)) {
		zero3(out_rad_s);
		return ADCS_STATUS_GYRO_FAIL;
	}
	memcpy(out_rad_s, raw_rad_s, 3 * sizeof(float));
	return ADCS_STATUS_OK;
}

enum adcs_status adcs_correct_mag(const float raw_ut[3], const float bias_t[3],
				  const float scale[3], float out_t[3])
{
	float mag[3];

	for (int i = 0; i < 3; ++i) {
		mag[i] = (raw_ut[i] * 1e-6f - bias_t[i]) * scale[i];
	}
	float n = norm3(mag);

	if (!(n >= MIN_MAG_NORM_T && n <= MAX_MAG_NORM_T)) {
		zero3(out_t);
		return ADCS_STATUS_MAG_FAIL;
	}
	memcpy(out_t, mag, sizeof(mag));
	return ADCS_STATUS_OK;
}

enum adcs_status adcs_sun_vector(const float lux[ADCS_LIGHT_COUNT], float sun[3])
{
	int valid = 0;

	zero3(sun);
	for (int i = 0; i < ADCS_LIGHT_COUNT; ++i) {
		valid += lux[i] != ADCS_LUX_ERROR;
	}
	if (valid == 0) {
		return ADCS_STATUS_SUN_NO_READINGS;
	}

	/* Solver order, as remapped in sun.py. */
	const float r0 = lux[ADCS_FACE_XP];
	const float r1 = lux[ADCS_FACE_XM];
	const float r2 = lux[ADCS_FACE_YP];
	const float r3 = lux[ADCS_FACE_YM];
	const float r4 = lux[ADCS_FACE_ZM];
	const float r5 = lux[ADCS_FACE_ZP_XP];
	const float r6 = lux[ADCS_FACE_ZP_XM];
	const float r7 = lux[ADCS_FACE_ZP_YP];
	const float r8 = lux[ADCS_FACE_ZP_YM];
	const float thr = THRESHOLD_ILLUMINATION_LUX;
	const float w0 = r0 > thr ? 1.0f : 0.0f;
	const float w1 = r1 > thr ? 1.0f : 0.0f;
	const float w2 = r2 > thr ? 1.0f : 0.0f;
	const float w3 = r3 > thr ? 1.0f : 0.0f;
	const float w4 = r4 > thr ? 1.0f : 0.0f;
	const float w5 = r5 > thr ? 1.0f : 0.0f;
	const float w6 = r6 > thr ? 1.0f : 0.0f;
	const float w7 = r7 > thr ? 1.0f : 0.0f;
	const float w8 = r8 > thr ? 1.0f : 0.0f;

	/* N^T W N; the X-Y cross term is zero by geometry. */
	const float a = w0 + w1 + 0.5f * (w5 + w6);
	const float b = w2 + w3 + 0.5f * (w7 + w8);
	const float e = w4 + 0.5f * (w5 + w6 + w7 + w8);
	const float c = 0.5f * (w5 - w6);
	const float d = 0.5f * (w7 - w8);

	if (a + b + e < 1e-8f) {
		return ADCS_STATUS_SUN_ECLIPSE;
	}
	if (a < 1e-8f || b < 1e-8f) {
		return ADCS_STATUS_SUN_NOT_ENOUGH_READINGS;
	}
	const float s = e - (c * c) / a - (d * d) / b;

	if (s < 1e-8f) {
		return ADCS_STATUS_SUN_NOT_ENOUGH_READINGS;
	}

	/* g = N^T W y */
	const float gx = w0 * r0 - w1 * r1 + INV_SQRT2 * (w5 * r5 - w6 * r6);
	const float gy = w2 * r2 - w3 * r3 + INV_SQRT2 * (w7 * r7 - w8 * r8);
	const float gz = -w4 * r4 + INV_SQRT2 * (w5 * r5 + w6 * r6 + w7 * r7 + w8 * r8);

	const float qz = (gz - c * gx / a - d * gy / b) / s;
	const float qx = (gx - c * qz) / a;
	const float qy = (gy - d * qz) / b;
	const float norm2 = qx * qx + qy * qy + qz * qz;

	if (norm2 < 1e-9f) {
		return ADCS_STATUS_ZERO_NORM;
	}
	const float inv = 1.0f / sqrtf(norm2);

	sun[0] = qx * inv;
	sun[1] = qy * inv;
	sun[2] = qz * inv;
	return ADCS_STATUS_OK;
}

void adcs_spin_stabilizing(const float omega[3], const float mag[3], float u[3])
{
	float target[3];
	float h[3];
	float error[3];

	momentum_target(target);
	inertia_times(omega, h);
	for (int i = 0; i < 3; ++i) {
		error[i] = target[i] - h[i];
	}
	cross3(mag, error, u);
	for (int i = 0; i < 3; ++i) {
		u[i] *= SPIN_STABILIZING_GAIN;
	}
	soft_clip(u);
}

void adcs_sun_pointing(const float sun[3], const float omega[3], const float mag[3],
		       float u[3])
{
	float h[3];
	float error[3];

	if (norm3(sun) == 0.0f || norm3(omega) == 0.0f) {
		zero3(u);
		return;
	}
	inertia_times(omega, h);
	float h_norm = norm3(h);

	/* Conical projection of the angular momentum onto the sun vector. */
	for (int i = 0; i < 3; ++i) {
		error[i] = sun[i] - h[i] / h_norm;
	}
	cross3(mag, error, u);
	float n = norm3(u);

	if (n < 1e-8f) {
		zero3(u);
		return;
	}
	for (int i = 0; i < 3; ++i) {
		u[i] /= n;
	}
}

void adcs_bdot(const float mag_buffer[ADCS_BDOT_SAMPLES][3], float dt_s, float u[3])
{
	/* 6-point derivative weights [-5, -3, -1, 1, 3, 5] / 35. */
	static const float weights[ADCS_BDOT_SAMPLES] = {
		-5.0f / 35.0f, -3.0f / 35.0f, -1.0f / 35.0f,
		1.0f / 35.0f, 3.0f / 35.0f, 5.0f / 35.0f,
	};

	if (!(dt_s > 0.0f)) {
		zero3(u);
		return;
	}
	for (int j = 0; j < 3; ++j) {
		float b_dot = 0.0f;

		for (int k = 0; k < ADCS_BDOT_SAMPLES; ++k) {
			b_dot += mag_buffer[k][j] * weights[k];
		}
		u[j] = -DETUMB_GAIN * (b_dot / dt_s);
	}
	soft_clip(u);
}

void adcs_bcross(const float mag[3], const float omega[3], float u[3])
{
	cross3(mag, omega, u);
	for (int i = 0; i < 3; ++i) {
		u[i] *= -DETUMB_GAIN;
	}
	soft_clip(u);
}

void adcs_allocate(const float u[3], const float b[3], const bool available[ADCS_COIL_COUNT],
		   float throttle[ADCS_COIL_COUNT])
{
	/* Only a single fully failed axis is compensated. */
	int axis_fail = -1;

	for (int axis = 0; axis < 3; ++axis) {
		if (!(available[2 * axis] || available[2 * axis + 1])) {
			if (axis_fail < 0) {
				axis_fail = axis;
			} else {
				axis_fail = -2;
				break;
			}
		}
	}

	/* u_eff = u - (u[a] / b[a]) * b zeroes the failed-axis dipole. */
	float u_eff[3] = {u[0], u[1], u[2]};

	if (axis_fail >= 0 && fabsf(b[axis_fail]) > 1e-9f) {
		float factor = u_eff[axis_fail] / b[axis_fail];

		for (int i = 0; i < 3; ++i) {
			u_eff[i] -= factor * b[i];
		}
	}

	/* Power limit on the effective coil vector: two coils share an axis at
	 * half throttle each; a lone coil carries the full axis value.
	 */
	float norm_sq = 0.0f;

	for (int axis = 0; axis < 3; ++axis) {
		float val = u_eff[axis];
		bool plus = available[2 * axis];
		bool minus = available[2 * axis + 1];

		if (plus && minus) {
			norm_sq += 0.5f * val * val;
		} else if (plus || minus) {
			norm_sq += val * val;
		}
	}
	float scale = fmaxf(1.0f, sqrtf(norm_sq) * 0.707f);

	for (int axis = 0; axis < 3; ++axis) {
		float val = u_eff[axis] / scale;
		bool plus = available[2 * axis];
		bool minus = available[2 * axis + 1];

		throttle[2 * axis] = 0.0f;
		throttle[2 * axis + 1] = 0.0f;
		if (plus && minus) {
			throttle[2 * axis] = 0.5f * val;
			throttle[2 * axis + 1] = 0.5f * val;
		} else if (plus) {
			throttle[2 * axis] = val;
		} else if (minus) {
			throttle[2 * axis + 1] = val;
		}
	}
}

enum adcs_mode adcs_next_mode(enum adcs_mode current, enum adcs_controller controller,
			      enum adcs_status gyro_status, const float omega[3],
			      enum adcs_status sun_status, const float sun[3])
{
	/* Fail-safe STABLE if the IMU fails. */
	if (gyro_status != ADCS_STATUS_OK) {
		return ADCS_MODE_STABLE;
	}
	const float omega_norm = norm3(omega);
	const float vf_tol = controller == ADCS_CTRL_BDOT ? VF_TUMBLING_TOL_BDOT : VF_TUMBLING_TOL;

	/* Tumbling logic common to all controllers. */
	if (current == ADCS_MODE_VF_TUMBLING) {
		return omega_norm < vf_tol ? ADCS_MODE_TUMBLING : ADCS_MODE_VF_TUMBLING;
	}
	if (current == ADCS_MODE_TUMBLING) {
		if (omega_norm <= TUMBLING_TOL) {
			return ADCS_MODE_STABLE;
		}
		return omega_norm >= vf_tol ? ADCS_MODE_VF_TUMBLING : ADCS_MODE_TUMBLING;
	}
	if (omega_norm >= TUMBLING_TOL) {
		return ADCS_MODE_TUMBLING;
	}

	/* Detumbling-only controllers (B-dot, B-cross). */
	if (controller != ADCS_CTRL_SUN_POINTING) {
		if (current == ADCS_MODE_STABLE) {
			return omega_norm < DETUMBLED_TOL_LO ? ADCS_MODE_ACS_OFF : ADCS_MODE_STABLE;
		}
		return omega_norm >= DETUMBLED_TOL_HI ? ADCS_MODE_STABLE : ADCS_MODE_ACS_OFF;
	}

	/* Sun pointing. */
	float h[3];
	float target[3];
	float diff[3];

	inertia_times(omega, h);
	momentum_target(target);
	const float target_mag = norm3(target);

	for (int i = 0; i < 3; ++i) {
		diff[i] = inertia_major_dir[i] - h[i] / target_mag;
	}
	const float momentum_error = norm3(diff);
	const bool sun_ok = sun_status == ADCS_STATUS_OK;

	if (current == ADCS_MODE_STABLE) {
		return momentum_error <= STABLE_TOL_LO && sun_ok ? ADCS_MODE_SUN_POINTING
								 : ADCS_MODE_STABLE;
	}
	if (!sun_ok) {
		return current;
	}
	const float h_norm = norm3(h);

	if (h_norm == 0.0f) {
		return ADCS_MODE_SUN_POINTING;
	}
	for (int i = 0; i < 3; ++i) {
		diff[i] = sun[i] - h[i] / h_norm;
	}
	const float sun_error = norm3(diff);

	if (current == ADCS_MODE_SUN_POINTING) {
		if (momentum_error >= STABLE_TOL_LO) {
			return ADCS_MODE_STABLE;
		}
		return sun_error <= SUN_POINTED_TOL_LO ? ADCS_MODE_ACS_OFF : ADCS_MODE_SUN_POINTING;
	}

	/* ACS_OFF */
	if (momentum_error >= STABLE_TOL_HI) {
		return ADCS_MODE_STABLE;
	}
	return sun_error >= SUN_POINTED_TOL_HI ? ADCS_MODE_SUN_POINTING : ADCS_MODE_ACS_OFF;
}
