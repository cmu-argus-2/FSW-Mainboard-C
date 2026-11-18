/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/adcs.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(ADCS_LIGHT_COUNT == 9 && ADCS_COIL_COUNT == 6, "ADCS telemetry layout changed");
_Static_assert(ADCS_VALID_ALL <= UINT16_MAX, "ADCS validity bits exceed the record mask");

void adcs_init(struct adcs_state *state, const struct adcs_config *config)
{
	static const struct adcs_config defaults = {
		.controller = ADCS_DEFAULT_CONTROLLER,
		.mag_bias_ut = {0.0f, 0.0f, 0.0f},
		.mag_scale = {1.0f, 1.0f, 1.0f},
	};

	memset(state, 0, sizeof(*state));
	state->config = config != NULL ? *config : defaults;
	if (adcs_set_controller(state, state->config.controller) != 0) {
		state->config.controller = ADCS_DEFAULT_CONTROLLER;
	}
	adcs_set_mag_bias_ut(state, state->config.mag_bias_ut);
	state->mode = ADCS_MODE_TUMBLING;
	state->coils_off = true;
}

int adcs_set_controller(struct adcs_state *state, enum adcs_controller controller)
{
	if (controller != ADCS_CTRL_BDOT && controller != ADCS_CTRL_BCROSS &&
	    controller != ADCS_CTRL_SUN_POINTING) {
		return -EINVAL;
	}
	state->config.controller = controller;
	return 0;
}

void adcs_set_mag_bias_ut(struct adcs_state *state, const float bias_ut[3])
{
	for (int i = 0; i < 3; ++i) {
		state->config.mag_bias_ut[i] = bias_ut[i];
		state->mag_bias_t[i] = bias_ut[i] * 1e-6f;
	}
}

/* --- Sensors (Python apps/adcs/sensors.py) --- */

static enum adcs_status read_gyro(const struct adcs_io *io, float out[3])
{
	float raw[3] = {0};

	if (io->read_gyro(io->context, raw) != 0) {
		memset(out, 0, 3 * sizeof(float));
		return ADCS_STATUS_GYRO_FAIL;
	}
	return adcs_check_gyro(raw, out);
}

static enum adcs_status read_mag(const struct adcs_state *state, const struct adcs_io *io,
				 float out[3])
{
	float raw[3] = {0};

	if (io->read_mag(io->context, raw) != 0) {
		memset(out, 0, 3 * sizeof(float));
		return ADCS_STATUS_MAG_FAIL;
	}
	return adcs_correct_mag(raw, state->mag_bias_t, state->config.mag_scale, out);
}

static uint16_t read_light(struct adcs_state *state, const struct adcs_io *io)
{
	uint16_t valid = 0;

	for (enum adcs_face i = 0; i < ADCS_LIGHT_COUNT; ++i) {
		float lux = 0.0f;

		if (io->read_lux(io->context, i, &lux) == 0) {
			state->lux[i] = lux;
			valid |= ADCS_VALID_LIGHT(i);
		} else {
			state->lux[i] = ADCS_LUX_ERROR;
		}
	}
	state->sun_status = adcs_sun_vector(state->lux, state->sun);
	return valid;
}

/* Python: int(lux * (1 / 3)) & 0xFFFF. Double matches the Python arithmetic. */
static uint16_t lux_log(float lux)
{
	double scaled = (double)lux * (1.0 / 3.0);

	if (!(scaled > -9.0e18 && scaled < 9.0e18)) {
		return 0;
	}
	return (uint16_t)(int64_t)scaled;
}

/* --- Coils (Python apps/adcs/acs.py) --- */

static void zero_all_coils(struct adcs_state *state, const struct adcs_io *io)
{
	bool all_zeroed = true;

	for (enum adcs_coil i = 0; i < ADCS_COIL_COUNT; ++i) {
		if (io->coil_available(io->context, i) && io->set_coil(io->context, i, 0.0f) != 0) {
			all_zeroed = false;
		}
	}
	/* A failed zero write is retried on the next ensure_coils_off. */
	state->coils_off = all_zeroed;
}

static void ensure_coils_off(struct adcs_state *state, const struct adcs_io *io,
			     bool hw_disable)
{
	bool cut_power = hw_disable && !state->coil_hw_disabled;

	if (!state->coils_off || cut_power) {
		zero_all_coils(state, io);
	}
	if (cut_power && io->set_coil_power(io->context, false) == 0) {
		state->coil_hw_disabled = true;
	}
}

static void drive_coils(struct adcs_state *state, const struct adcs_io *io, const float u[3])
{
	bool available[ADCS_COIL_COUNT];
	float throttle[ADCS_COIL_COUNT];

	for (enum adcs_coil i = 0; i < ADCS_COIL_COUNT; ++i) {
		available[i] = io->coil_available(io->context, i);
	}
	adcs_allocate(u, state->mag, available, throttle);
	for (enum adcs_coil i = 0; i < ADCS_COIL_COUNT; ++i) {
		/* Status reports a coil as active only if its command was accepted. */
		state->coil_status[i] =
			available[i] && io->set_coil(io->context, i, throttle[i]) == 0;
	}
	state->coils_off = false;
}

/* --- Control cycles (Python tasks/adcs.py) --- */

static void apply_control(struct adcs_state *state, const struct adcs_io *io)
{
	float u[3] = {0.0f, 0.0f, 0.0f};
	bool imu_ok = state->gyro_status == ADCS_STATUS_OK && state->mag_status == ADCS_STATUS_OK;

	if (state->config.controller == ADCS_CTRL_BCROSS) {
		if (imu_ok) {
			adcs_bcross(state->mag, state->gyro, u);
		}
	} else if (state->config.controller == ADCS_CTRL_SUN_POINTING) {
		if (state->mode == ADCS_MODE_TUMBLING || state->mode == ADCS_MODE_STABLE) {
			if (imu_ok) {
				adcs_spin_stabilizing(state->gyro, state->mag, u);
			}
		} else if (state->mode == ADCS_MODE_SUN_POINTING) {
			if (imu_ok && state->sun_status == ADCS_STATUS_OK) {
				adcs_sun_pointing(state->sun, state->gyro, state->mag, u);
			}
		}
	}
	drive_coils(state, io, u);
}

/* Six magnetometer samples 80 ms apart, coils on for 500 ms, then off. */
static void bdot_cycle(struct adcs_state *state, const struct adcs_io *io)
{
	float buffer[ADCS_BDOT_SAMPLES][3];
	int count = 0;

	for (int k = 0; k < ADCS_BDOT_SAMPLES; ++k) {
		if (read_mag(state, io, buffer[count]) == ADCS_STATUS_OK) {
			++count;
		}
		if (k < ADCS_BDOT_SAMPLES - 1) {
			io->sleep_ms(io->context, ADCS_MAG_SAMPLE_DT_MS);
		}
	}
	if (count > 0) {
		memcpy(state->mag, buffer[count - 1], sizeof(state->mag));
		state->mag_status = ADCS_STATUS_OK;
	} else {
		state->mag_status = ADCS_STATUS_MAG_FAIL;
	}

	if (count == ADCS_BDOT_SAMPLES) {
		float u[3];

		adcs_bdot(buffer, ADCS_MAG_SAMPLE_DT_MS / 1000.0f, u);
		drive_coils(state, io, u);
	} else {
		ensure_coils_off(state, io, false);
	}
	io->sleep_ms(io->context, ADCS_BDOT_COIL_ON_MS);
	ensure_coils_off(state, io, false);
}

/* Re-read the gyro and update the coils every 50 ms, then turn the coils off. */
static void bcross_sun_cycle(struct adcs_state *state, const struct adcs_io *io)
{
	int64_t start = io->uptime_ms(io->context);

	while (io->uptime_ms(io->context) - start < ADCS_BCROSS_DURATION_MS) {
		int64_t loop_start = io->uptime_ms(io->context);

		state->gyro_status = read_gyro(io, state->gyro);
		apply_control(state, io);

		int64_t remaining = ADCS_GYRO_INTERVAL_MS - (io->uptime_ms(io->context) - loop_start);

		if (remaining > 0) {
			io->sleep_ms(io->context, (int32_t)remaining);
		}
	}
	ensure_coils_off(state, io, false);
}

static void log_snapshot(struct adcs_state *state, const struct adcs_io *io, uint32_t time_s,
			 uint16_t valid_reads)
{
	struct adcs_record *r = &state->record;

	r->time = time_s;
	r->mode = (uint8_t)state->mode;
	r->controller = (uint8_t)state->config.controller;
	memcpy(r->gyro, state->gyro, sizeof(r->gyro));
	memcpy(r->mag, state->mag, sizeof(r->mag));
	r->sun_status = (uint8_t)state->sun_status;
	memcpy(r->sun, state->sun, sizeof(r->sun));
	for (int i = 0; i < ADCS_LIGHT_COUNT; ++i) {
		/* A failed sensor's -1 placeholder logs as int(-1 / 3) = 0. */
		r->lux[i] = lux_log(state->lux[i]);
	}
	for (int i = 0; i < ADCS_COIL_COUNT; ++i) {
		r->coil_status[i] = state->coil_status[i];
	}
	io->publish(io->context, r, valid_reads);
}

static void update_gyro_range(struct adcs_state *state, const struct adcs_io *io,
			      enum adcs_gyro_range range)
{
	/* Avoid an IMU write every second; retry if the last write failed. */
	if (state->gyro_range_set && state->gyro_range == range) {
		return;
	}
	state->gyro_range_set = io->set_gyro_range(io->context, range) == 0;
	state->gyro_range = range;
}

void adcs_step(struct adcs_state *state, const struct adcs_io *io, uint32_t time_s)
{
	enum argus_sat_state sat = io->get_sat_state(io->context);

	if (sat == ARGUS_STATE_STARTUP) {
		return;
	}
	if (sat == ARGUS_STATE_LOW_POWER) {
		/* Save power: no sensing, no control, coil drivers unpowered. */
		ensure_coils_off(state, io, true);
		return;
	}

	uint16_t valid_reads = 0;

	state->mag_status = read_mag(state, io, state->mag);
	state->gyro_status = read_gyro(io, state->gyro);
	valid_reads |= state->mag_status == ADCS_STATUS_OK ? ADCS_VALID_MAG : 0U;
	valid_reads |= state->gyro_status == ADCS_STATUS_OK ? ADCS_VALID_GYRO : 0U;
	valid_reads |= read_light(state, io);
	log_snapshot(state, io, time_s, valid_reads);

	if (sat == ARGUS_STATE_DETUMBLING) {
		update_gyro_range(state, io, ADCS_GYRO_RANGE_2000DPS);
	} else if (sat == ARGUS_STATE_NOMINAL) {
		update_gyro_range(state, io, ADCS_GYRO_RANGE_125DPS);
	}

	state->mode = adcs_next_mode(state->mode, state->config.controller, state->gyro_status,
				     state->gyro, state->sun_status, state->sun);

	if (state->mode == ADCS_MODE_ACS_OFF || state->mode == ADCS_MODE_VF_TUMBLING) {
		ensure_coils_off(state, io, true);
		return;
	}
	if (state->coil_hw_disabled && io->set_coil_power(io->context, true) == 0) {
		state->coil_hw_disabled = false;
	}
	if (state->config.controller == ADCS_CTRL_BDOT) {
		bdot_cycle(state, io);
	} else {
		bcross_sun_cycle(state, io);
	}
}

int adcs_format_record(char *buffer, size_t size, const struct adcs_record *r)
{
	if (buffer == NULL || r == NULL || size == 0) {
		return -EINVAL;
	}
	/* %.9g round-trips a float. */
	int used = snprintf(buffer, size,
			    "ADCS,%lu,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g,%.9g,%.9g",
			    (unsigned long)r->time, r->mode, r->controller,
			    (double)r->gyro[0], (double)r->gyro[1], (double)r->gyro[2],
			    (double)r->mag[0], (double)r->mag[1], (double)r->mag[2],
			    r->sun_status, (double)r->sun[0], (double)r->sun[1], (double)r->sun[2]);

	if (used < 0 || (size_t)used >= size) {
		return -ENOSPC;
	}
	for (int i = 0; i < ADCS_LIGHT_COUNT + ADCS_COIL_COUNT; ++i) {
		unsigned int v = i < ADCS_LIGHT_COUNT ? r->lux[i]
						      : r->coil_status[i - ADCS_LIGHT_COUNT];
		int written = snprintf(buffer + used, size - used, ",%u", v);

		if (written < 0 || (size_t)written >= size - used) {
			return -ENOSPC;
		}
		used += written;
	}
	return used;
}
