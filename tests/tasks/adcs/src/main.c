/* SPDX-License-Identifier: Apache-2.0 */
#include "argus/tasks/adcs.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "adcs_vectors.h"

#define TOL 1e-4f

struct fake_hw {
	enum argus_sat_state sat;
	int64_t now_ms;
	float gyro[3];
	int gyro_rc;
	unsigned int gyro_reads;
	float mag_ut[3];
	float mag_rate_ut_per_s[3];
	uint32_t mag_fail_mask; /* bit n: read n fails */
	unsigned int mag_reads;
	float lux[ADCS_LIGHT_COUNT];
	int lux_rc[ADCS_LIGHT_COUNT];
	int range_rc;
	unsigned int range_writes;
	enum adcs_gyro_range range;
	bool available[ADCS_COIL_COUNT];
	int coil_rc[ADCS_COIL_COUNT];
	unsigned int coil_writes;
	unsigned int drive_writes;
	float coil[ADCS_COIL_COUNT];
	float first_drive[ADCS_COIL_COUNT];
	bool driven;
	int power_rc;
	bool power;
	unsigned int power_writes;
	unsigned int publishes;
	struct adcs_record last;
	uint16_t last_valid;
};
static struct fake_hw hw;
static struct adcs_state state;

static enum argus_sat_state get_sat(void *ctx)
{
	return ((struct fake_hw *)ctx)->sat;
}
static int gyro(void *ctx, float out[3])
{
	struct fake_hw *h = ctx;

	++h->gyro_reads;
	memcpy(out, h->gyro, sizeof(h->gyro)); /* Even a failing callback may touch output. */
	return h->gyro_rc;
}
static int mag(void *ctx, float out[3])
{
	struct fake_hw *h = ctx;
	unsigned int n = h->mag_reads++;

	for (int i = 0; i < 3; ++i) {
		out[i] = h->mag_ut[i] + h->mag_rate_ut_per_s[i] * (float)h->now_ms / 1000.0f;
	}
	return n < 32 && (h->mag_fail_mask & BIT(n)) ? -EIO : 0;
}
static int range(void *ctx, enum adcs_gyro_range r)
{
	struct fake_hw *h = ctx;

	++h->range_writes;
	if (h->range_rc == 0) {
		h->range = r;
	}
	return h->range_rc;
}
static int lux(void *ctx, enum adcs_face face, float *out)
{
	struct fake_hw *h = ctx;

	*out = h->lux[face];
	return h->lux_rc[face];
}
static bool available(void *ctx, enum adcs_coil coil)
{
	return ((struct fake_hw *)ctx)->available[coil];
}
static int set_coil(void *ctx, enum adcs_coil coil, float throttle)
{
	struct fake_hw *h = ctx;

	zassert_true(h->available[coil], "commanded an unavailable coil");
	++h->coil_writes;
	if (h->coil_rc[coil] != 0) {
		return h->coil_rc[coil];
	}
	h->coil[coil] = throttle;
	if (throttle != 0.0f) {
		++h->drive_writes;
	}
	return 0;
}
static int coil_power(void *ctx, bool on)
{
	struct fake_hw *h = ctx;

	++h->power_writes;
	if (h->power_rc == 0) {
		h->power = on;
	}
	return h->power_rc;
}
static int64_t uptime(void *ctx)
{
	return ((struct fake_hw *)ctx)->now_ms;
}
static void sleep_ms(void *ctx, int32_t ms)
{
	struct fake_hw *h = ctx;

	zassert_true(ms > 0);
	h->now_ms += ms;
	/* Every coil of the first actuation has been commanded by its first sleep. */
	if (!h->driven && h->drive_writes > 0) {
		memcpy(h->first_drive, h->coil, sizeof(h->coil));
		h->driven = true;
	}
}
static void publish(void *ctx, const struct adcs_record *record, uint16_t valid)
{
	struct fake_hw *h = ctx;

	++h->publishes;
	h->last = *record;
	h->last_valid = valid;
}

static const struct adcs_io io = {
	.context = &hw, .get_sat_state = get_sat, .read_gyro = gyro, .read_mag = mag,
	.set_gyro_range = range, .read_lux = lux, .coil_available = available,
	.set_coil = set_coil, .set_coil_power = coil_power, .uptime_ms = uptime,
	.sleep_ms = sleep_ms, .publish = publish,
};

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&hw, 0, sizeof(hw));
	hw.sat = ARGUS_STATE_DETUMBLING;
	hw.power = true;
	/* 40 uT field, fast spin (TUMBLING), sunlit +X face. */
	hw.mag_ut[0] = 20.0f;
	hw.mag_ut[1] = -15.0f;
	hw.mag_ut[2] = 30.0f;
	hw.gyro[0] = 0.4f;
	hw.gyro[1] = -0.3f;
	hw.gyro[2] = 0.5f;
	for (int i = 0; i < ADCS_LIGHT_COUNT; ++i) {
		hw.lux[i] = 100.0f;
	}
	hw.lux[ADCS_FACE_XP] = 90000.0f;
	hw.lux[ADCS_FACE_ZP_XP] = 60000.0f;
	hw.lux[ADCS_FACE_YP] = 20000.0f;
	hw.lux[ADCS_FACE_ZM] = 30000.0f;
	for (int i = 0; i < ADCS_COIL_COUNT; ++i) {
		hw.available[i] = true;
	}
	adcs_init(&state, NULL);
}

static void assert_vec(const float *actual, const float *expected, int n, float tol)
{
	for (int i = 0; i < n; ++i) {
		zassert_within(actual[i], expected[i], tol, "index %d: %f != %f", i,
			       (double)actual[i], (double)expected[i]);
	}
}

static void assert_coils_zeroed(void)
{
	for (int i = 0; i < ADCS_COIL_COUNT; ++i) {
		zassert_equal(hw.coil[i], 0.0f, "coil %d left at %f", i, (double)hw.coil[i]);
	}
}

/* --- Parity with the Python flight software (adcs_vectors.h) --- */

ZTEST(adcs_python_parity, test_spin_stabilizing)
{
	for (size_t i = 0; i < ARRAY_SIZE(spin_vectors); ++i) {
		float u[3];

		adcs_spin_stabilizing(spin_vectors[i].a, spin_vectors[i].b, u);
		assert_vec(u, spin_vectors[i].u, 3, TOL);
	}
}

ZTEST(adcs_python_parity, test_bcross)
{
	for (size_t i = 0; i < ARRAY_SIZE(bcross_vectors); ++i) {
		float u[3];

		adcs_bcross(bcross_vectors[i].a, bcross_vectors[i].b, u);
		assert_vec(u, bcross_vectors[i].u, 3, TOL);
	}
}

ZTEST(adcs_python_parity, test_sun_pointing)
{
	for (size_t i = 0; i < ARRAY_SIZE(sun_pointing_vectors); ++i) {
		const struct ctrl_vector *v = &sun_pointing_vectors[i];
		float u[3];

		adcs_sun_pointing(v->a, v->b, v->c, u);
		assert_vec(u, v->u, 3, TOL);
	}
}

ZTEST(adcs_python_parity, test_bdot)
{
	for (size_t i = 0; i < ARRAY_SIZE(bdot_vectors); ++i) {
		float u[3];

		adcs_bdot(bdot_vectors[i].buffer, 0.08f, u);
		/* Differences of ~40 uT samples lose float32 precision. */
		assert_vec(u, bdot_vectors[i].u, 3, 5e-4f);
	}
}

ZTEST(adcs_python_parity, test_sun_vector)
{
	for (size_t i = 0; i < ARRAY_SIZE(sun_vectors); ++i) {
		float sun[3];

		zassert_equal(adcs_sun_vector(sun_vectors[i].lux, sun), sun_vectors[i].status,
			      "case %zu", i);
		assert_vec(sun, sun_vectors[i].sun, 3, TOL);
	}
}

ZTEST(adcs_python_parity, test_allocator_every_availability_mask)
{
	for (size_t i = 0; i < ARRAY_SIZE(alloc_vectors); ++i) {
		const struct alloc_vector *v = &alloc_vectors[i];
		bool avail[ADCS_COIL_COUNT];
		float throttle[ADCS_COIL_COUNT];

		for (int c = 0; c < ADCS_COIL_COUNT; ++c) {
			avail[c] = v->mask & BIT(c);
		}
		adcs_allocate(v->u, v->b, avail, throttle);
		assert_vec(throttle, v->throttle, ADCS_COIL_COUNT, TOL);
	}
}

ZTEST(adcs_python_parity, test_mode_transitions)
{
	for (size_t i = 0; i < ARRAY_SIZE(mode_vectors); ++i) {
		const struct mode_vector *v = &mode_vectors[i];

		zassert_equal(adcs_next_mode(v->current, v->controller, v->gyro_status, v->omega,
					     v->sun_status, v->sun),
			      v->next, "case %zu", i);
	}
}

/* --- Algorithm edge cases --- */

ZTEST(adcs_algorithms, test_gyro_limit_and_nan)
{
	float out[3] = {9, 9, 9};
	float limit = 2000.0f * 3.14159265f / 180.0f;

	zassert_equal(adcs_check_gyro((float[3]){limit * 0.999f, 0, 0}, out), ADCS_STATUS_OK);
	zassert_equal(out[0], limit * 0.999f);
	zassert_equal(adcs_check_gyro((float[3]){0, 0, limit * 1.001f}, out),
		      ADCS_STATUS_GYRO_FAIL);
	zassert_equal(out[2], 0.0f);
	zassert_equal(adcs_check_gyro((float[3]){NAN, 0, 0}, out), ADCS_STATUS_GYRO_FAIL);
}

ZTEST(adcs_algorithms, test_mag_units_bias_scale_and_bounds)
{
	const float bias[3] = {1e-6f, 0, 0};
	const float scale[3] = {2.0f, 1.0f, 1.0f};
	const float unit_scale[3] = {1, 1, 1};
	const float zero[3] = {0};
	float out[3];

	zassert_equal(adcs_correct_mag((float[3]){21.0f, -15.0f, 0}, bias, scale, out),
		      ADCS_STATUS_OK);
	assert_vec(out, (float[3]){40e-6f, -15e-6f, 0}, 3, 1e-10f);
	zassert_equal(adcs_correct_mag((float[3]){0.5f, 0, 0}, zero, unit_scale, out),
		      ADCS_STATUS_MAG_FAIL);
	zassert_equal(adcs_correct_mag((float[3]){2600.0f, 0, 0}, zero, unit_scale, out),
		      ADCS_STATUS_MAG_FAIL);
	zassert_equal(out[0], 0.0f);
	zassert_equal(adcs_correct_mag((float[3]){NAN, 0, 0}, zero, unit_scale, out),
		      ADCS_STATUS_MAG_FAIL);
}

ZTEST(adcs_algorithms, test_controller_fallbacks)
{
	float u[3] = {1, 1, 1};
	const float buffer[ADCS_BDOT_SAMPLES][3] = {{1e-5f, 0, 0}};

	adcs_sun_pointing((float[3]){0}, (float[3]){0.1f, 0, 0}, (float[3]){1e-5f, 0, 0}, u);
	assert_vec(u, (float[3]){0}, 3, 0);
	adcs_sun_pointing((float[3]){0, 0, 1}, (float[3]){0}, (float[3]){1e-5f, 0, 0}, u);
	assert_vec(u, (float[3]){0}, 3, 0);
	adcs_bdot(buffer, 0.0f, u);
	assert_vec(u, (float[3]){0}, 3, 0);
	adcs_bcross((float[3]){0}, (float[3]){1, 2, 3}, u);
	assert_vec(u, (float[3]){0}, 3, 0);
}

ZTEST(adcs_algorithms, test_mode_thresholds_are_inclusive_as_in_python)
{
	/* B-dot VF threshold is 2.62 rad/s, others 3.05 rad/s. */
	zassert_equal(adcs_next_mode(ADCS_MODE_TUMBLING, ADCS_CTRL_BDOT, ADCS_STATUS_OK,
				     (float[3]){2.63f, 0, 0}, 0, (float[3]){0}),
		      ADCS_MODE_VF_TUMBLING);
	zassert_equal(adcs_next_mode(ADCS_MODE_TUMBLING, ADCS_CTRL_BCROSS, ADCS_STATUS_OK,
				     (float[3]){2.63f, 0, 0}, 0, (float[3]){0}),
		      ADCS_MODE_TUMBLING);
	zassert_equal(adcs_next_mode(ADCS_MODE_TUMBLING, ADCS_CTRL_BCROSS, ADCS_STATUS_OK,
				     (float[3]){0.5f, 0, 0}, 0, (float[3]){0}),
		      ADCS_MODE_STABLE);
	zassert_equal(adcs_next_mode(ADCS_MODE_ACS_OFF, ADCS_CTRL_BCROSS, ADCS_STATUS_OK,
				     (float[3]){0.08f, 0, 0}, 0, (float[3]){0}),
		      ADCS_MODE_ACS_OFF);
	zassert_equal(adcs_next_mode(ADCS_MODE_ACS_OFF, ADCS_CTRL_BCROSS, ADCS_STATUS_OK,
				     (float[3]){0.09f, 0, 0}, 0, (float[3]){0}),
		      ADCS_MODE_STABLE);
	zassert_equal(adcs_next_mode(ADCS_MODE_ACS_OFF, ADCS_CTRL_BCROSS,
				     ADCS_STATUS_GYRO_FAIL, (float[3]){0}, 0, (float[3]){0}),
		      ADCS_MODE_STABLE);
}

/* --- Task --- */

ZTEST(adcs_task, test_startup_does_nothing)
{
	hw.sat = ARGUS_STATE_STARTUP;
	adcs_step(&state, &io, 1);
	zassert_equal(hw.publishes + hw.gyro_reads + hw.mag_reads + hw.coil_writes +
			      hw.power_writes + hw.range_writes, 0);
	zassert_equal(hw.now_ms, 0);
}

ZTEST(adcs_task, test_low_power_cuts_coil_power_once_without_sensing)
{
	hw.sat = ARGUS_STATE_LOW_POWER;
	adcs_step(&state, &io, 1);
	adcs_step(&state, &io, 2);
	zassert_equal(hw.publishes + hw.gyro_reads + hw.mag_reads, 0);
	zassert_false(hw.power);
	zassert_equal(hw.power_writes, 1);
	zassert_equal(hw.drive_writes, 0);
	assert_coils_zeroed();

	/* Leaving low power re-enables the drivers before actuating. */
	hw.sat = ARGUS_STATE_DETUMBLING;
	adcs_step(&state, &io, 3);
	zassert_true(hw.power);
	zassert_equal(hw.power_writes, 2);
}

ZTEST(adcs_task, test_failed_power_cut_is_retried)
{
	hw.sat = ARGUS_STATE_LOW_POWER;
	hw.power_rc = -EIO;
	adcs_step(&state, &io, 1);
	zassert_false(state.coil_hw_disabled);
	hw.power_rc = 0;
	adcs_step(&state, &io, 2);
	zassert_true(state.coil_hw_disabled);
	zassert_false(hw.power);
	zassert_equal(hw.power_writes, 2);
}

ZTEST(adcs_task, test_record_fields_validity_and_lux_scaling)
{
	hw.lux[ADCS_FACE_XM] = 200000.0f; /* 66666 wraps to 1130, as Python & 0xFFFF */
	hw.lux[ADCS_FACE_YM] = 3001.0f;
	hw.lux_rc[ADCS_FACE_ZP_YP] = -EIO;
	adcs_step(&state, &io, 42);
	zassert_equal(hw.publishes, 1);

	const struct adcs_record *r = &hw.last;

	zassert_equal(r->time, 42);
	zassert_equal(r->mode, ADCS_MODE_TUMBLING, "logged before the mode update");
	zassert_equal(r->controller, ADCS_CTRL_BDOT);
	assert_vec(r->gyro, hw.gyro, 3, 0);
	assert_vec(r->mag, (float[3]){20e-6f, -15e-6f, 30e-6f}, 3, 1e-10f);
	zassert_equal(r->sun_status, ADCS_STATUS_OK);
	zassert_equal(r->lux[ADCS_FACE_XP], 30000);
	zassert_equal(r->lux[ADCS_FACE_XM], 1130);
	zassert_equal(r->lux[ADCS_FACE_YM], 1000);
	zassert_equal(r->lux[ADCS_FACE_ZP_YP], 0, "failed sensor logs int(-1/3) = 0");
	zassert_equal(r->lux[ADCS_FACE_ZM], 10000);
	zassert_equal(hw.last_valid, ADCS_VALID_ALL & ~ADCS_VALID_LIGHT(ADCS_FACE_ZP_YP));

	float expected_sun[3];

	(void)adcs_sun_vector(state.lux, expected_sun);
	assert_vec(r->sun, expected_sun, 3, 0);
	zassert_within(sqrtf(r->sun[0] * r->sun[0] + r->sun[1] * r->sun[1] +
			     r->sun[2] * r->sun[2]), 1.0f, TOL);
}

ZTEST(adcs_task, test_failed_imu_reads_are_zero_placeholders)
{
	hw.gyro_rc = -EIO;
	hw.mag_fail_mask = BIT(0);
	adcs_step(&state, &io, 1);
	assert_vec(hw.last.gyro, (float[3]){0}, 3, 0);
	assert_vec(hw.last.mag, (float[3]){0}, 3, 0);
	zassert_equal(hw.last_valid & (ADCS_VALID_GYRO | ADCS_VALID_MAG), 0);
	zassert_equal(state.mode, ADCS_MODE_STABLE, "gyro failure falls back to STABLE");
}

ZTEST(adcs_task, test_gyro_range_follows_state_and_retries_failures)
{
	adcs_step(&state, &io, 1);
	adcs_step(&state, &io, 2);
	zassert_equal(hw.range, ADCS_GYRO_RANGE_2000DPS);
	zassert_equal(hw.range_writes, 1);
	hw.sat = ARGUS_STATE_NOMINAL;
	hw.range_rc = -EIO;
	adcs_step(&state, &io, 3);
	adcs_step(&state, &io, 4);
	zassert_equal(hw.range_writes, 3);
	hw.range_rc = 0;
	adcs_step(&state, &io, 5);
	adcs_step(&state, &io, 6);
	zassert_equal(hw.range, ADCS_GYRO_RANGE_125DPS);
	zassert_equal(hw.range_writes, 4);
	hw.sat = ARGUS_STATE_EXPERIMENT;
	adcs_step(&state, &io, 7);
	zassert_equal(hw.range_writes, 4, "experiment leaves the range alone");
}

ZTEST(adcs_task, test_bdot_cycle_timing_and_actuation)
{
	hw.mag_rate_ut_per_s[0] = 5.0f;
	hw.mag_rate_ut_per_s[2] = -3.0f;
	adcs_step(&state, &io, 1);

	/* Snapshot + 6 samples; 5 x 80 ms + 500 ms coil-on. */
	zassert_equal(hw.mag_reads, 7);
	zassert_equal(hw.now_ms, 900);
	zassert_true(hw.drive_writes > 0);
	assert_coils_zeroed();

	float buffer[ADCS_BDOT_SAMPLES][3];
	float corrected[3];
	float u[3];
	float throttle[ADCS_COIL_COUNT];
	const float zero[3] = {0};
	const float unit_scale[3] = {1, 1, 1};

	for (int k = 0; k < ADCS_BDOT_SAMPLES; ++k) {
		for (int i = 0; i < 3; ++i) {
			buffer[k][i] = hw.mag_ut[i] + hw.mag_rate_ut_per_s[i] * (0.08f * k);
		}
		(void)adcs_correct_mag(buffer[k], zero, unit_scale, corrected);
		memcpy(buffer[k], corrected, sizeof(corrected));
	}
	adcs_bdot(buffer, 0.08f, u);
	adcs_allocate(u, buffer[5], (bool[6]){1, 1, 1, 1, 1, 1}, throttle);
	assert_vec(hw.first_drive, throttle, ADCS_COIL_COUNT, TOL);
	for (int i = 0; i < ADCS_COIL_COUNT; ++i) {
		zassert_true(hw.last.coil_status[i] == 0, "first record precedes actuation");
		zassert_true(state.coil_status[i]);
	}
}

ZTEST(adcs_task, test_bdot_missing_sample_skips_actuation_but_keeps_timing)
{
	hw.mag_fail_mask = BIT(3); /* one of the six cycle samples */
	adcs_step(&state, &io, 1);
	zassert_equal(hw.drive_writes, 0);
	zassert_equal(hw.now_ms, 900);
	zassert_equal(state.mag_status, ADCS_STATUS_OK, "later samples still valid");

	hw.mag_fail_mask = GENMASK(31, 7); /* every cycle sample of the next step */
	adcs_step(&state, &io, 2);
	zassert_equal(state.mag_status, ADCS_STATUS_MAG_FAIL);
}

ZTEST(adcs_task, test_bcross_cycle_reads_gyro_at_20_hz)
{
	zassert_ok(adcs_set_controller(&state, ADCS_CTRL_BCROSS));
	adcs_step(&state, &io, 1);
	zassert_equal(hw.last.controller, ADCS_CTRL_BCROSS);
	zassert_equal(hw.gyro_reads, 1 + 20);
	zassert_equal(hw.mag_reads, 1, "B-cross uses the snapshot field");
	zassert_equal(hw.now_ms, 1000);
	assert_coils_zeroed();

	float u[3];
	float throttle[ADCS_COIL_COUNT];

	adcs_bcross(state.mag, hw.gyro, u);
	adcs_allocate(u, state.mag, (bool[6]){1, 1, 1, 1, 1, 1}, throttle);
	assert_vec(hw.first_drive, throttle, ADCS_COIL_COUNT, TOL);
}

ZTEST(adcs_task, test_sun_pointing_controller_spin_stabilizes_while_tumbling)
{
	zassert_ok(adcs_set_controller(&state, ADCS_CTRL_SUN_POINTING));
	adcs_step(&state, &io, 1);
	zassert_equal(state.mode, ADCS_MODE_TUMBLING);

	float u[3];
	float throttle[ADCS_COIL_COUNT];

	adcs_spin_stabilizing(hw.gyro, state.mag, u);
	adcs_allocate(u, state.mag, (bool[6]){1, 1, 1, 1, 1, 1}, throttle);
	assert_vec(hw.first_drive, throttle, ADCS_COIL_COUNT, TOL);
}

ZTEST(adcs_task, test_detumbled_turns_acs_off_and_recovers)
{
	zassert_ok(adcs_set_controller(&state, ADCS_CTRL_BCROSS));
	hw.gyro[0] = 0.01f;
	hw.gyro[1] = 0.02f;
	hw.gyro[2] = 0.0f;
	adcs_step(&state, &io, 1); /* TUMBLING -> STABLE, still actuating */
	zassert_equal(state.mode, ADCS_MODE_STABLE);
	zassert_true(hw.power);
	int64_t before_off = hw.now_ms;

	adcs_step(&state, &io, 2); /* STABLE -> ACS_OFF */
	zassert_equal(state.mode, ADCS_MODE_ACS_OFF);
	zassert_false(hw.power);
	zassert_equal(hw.now_ms, before_off, "no control cycle with ACS off");
	assert_coils_zeroed();

	hw.gyro[0] = 1.0f; /* spun up again */
	unsigned int drives = hw.drive_writes;

	adcs_step(&state, &io, 3);
	zassert_equal(state.mode, ADCS_MODE_TUMBLING);
	zassert_true(hw.power);
	zassert_true(hw.drive_writes > drives);
}

ZTEST(adcs_task, test_very_fast_tumbling_disables_coils)
{
	hw.gyro[0] = 3.0f; /* above the 2.62 rad/s B-dot limit */
	adcs_step(&state, &io, 1);
	zassert_equal(state.mode, ADCS_MODE_VF_TUMBLING);
	zassert_false(hw.power);
	zassert_equal(hw.drive_writes, 0);
}

ZTEST(adcs_task, test_only_available_coils_are_commanded)
{
	zassert_ok(adcs_set_controller(&state, ADCS_CTRL_BCROSS));
	hw.available[ADCS_COIL_ZP] = false;
	hw.available[ADCS_COIL_ZM] = false; /* Z axis failed: compensated along B */
	hw.available[ADCS_COIL_XM] = false;
	adcs_step(&state, &io, 1);
	zassert_true(hw.first_drive[ADCS_COIL_XP] != 0.0f);
	zassert_false(state.coil_status[ADCS_COIL_XM]);
	zassert_false(state.coil_status[ADCS_COIL_ZP]);
	zassert_true(state.coil_status[ADCS_COIL_YP]);
}

ZTEST(adcs_task, test_rejected_coil_command_is_not_reported_active)
{
	hw.mag_rate_ut_per_s[0] = 5.0f;
	hw.coil_rc[ADCS_COIL_YM] = -EIO;
	adcs_step(&state, &io, 1);
	zassert_false(state.coil_status[ADCS_COIL_YM]);
	zassert_true(state.coil_status[ADCS_COIL_YP]);
	zassert_false(state.coils_off, "failed zero write is retried");
	hw.coil_rc[ADCS_COIL_YM] = 0;
	hw.sat = ARGUS_STATE_LOW_POWER;
	adcs_step(&state, &io, 2);
	zassert_true(state.coils_off);
	adcs_step(&state, &io, 3);
	zassert_equal(hw.last.coil_status[ADCS_COIL_YM], 0);
}

ZTEST(adcs_task, test_commands_validate_and_apply_next_step)
{
	zassert_equal(adcs_set_controller(&state, 7), -EINVAL);
	zassert_equal(state.config.controller, ADCS_CTRL_BDOT);
	adcs_set_mag_bias_ut(&state, (float[3]){20.0f, 0, 0});
	adcs_step(&state, &io, 1);
	assert_vec(hw.last.mag, (float[3]){0, -15e-6f, 30e-6f}, 3, 1e-10f);

	struct adcs_config bad = {.controller = 9, .mag_scale = {1, 1, 1}};

	adcs_init(&state, &bad);
	zassert_equal(state.config.controller, ADCS_DEFAULT_CONTROLLER);
}

ZTEST(adcs_task, test_csv_has_28_fields_in_order)
{
	char line[ADCS_RECORD_BUFFER_SIZE];
	struct adcs_record r = {
		.time = 4000000000U, .mode = 2, .controller = 1,
		.gyro = {0.5f, -0.25f, 1e-3f}, .mag = {4e-5f, -1.5e-5f, 2.5e-3f},
		.sun_status = 53, .sun = {0, -1, 0.125f},
		.lux = {0, 1, 2, 3, 4, 5, 6, 7, 65535},
		.coil_status = {1, 0, 1, 0, 1, 1},
	};

	zassert_true(adcs_format_record(line, sizeof(line), &r) > 0);
	zassert_true(strncmp(line, "ADCS,", 5) == 0);

	const double expected[] = {
		4000000000.0, 2, 1, 0.5, -0.25, 1e-3, 4e-5, -1.5e-5, 2.5e-3, 53, 0, -1, 0.125,
		0, 1, 2, 3, 4, 5, 6, 7, 65535, 1, 0, 1, 0, 1, 1,
	};
	char *cursor = line + 4;

	BUILD_ASSERT(ARRAY_SIZE(expected) == ADCS_FIELD_COUNT);
	for (size_t i = 0; i < ARRAY_SIZE(expected); ++i) {
		char *end;

		zassert_equal(*cursor++, ',', "field %zu", i);
		double value = strtod(cursor, &end);

		zassert_true(end != cursor);
		zassert_within(value, expected[i], fabs(expected[i]) * 1e-6 + 1e-12, "field %zu",
			       i);
		cursor = end;
	}
	zassert_equal(*cursor, '\0');
	zassert_equal(adcs_format_record(line, 8, &r), -ENOSPC);
	zassert_equal(adcs_format_record(NULL, 8, &r), -EINVAL);
}

ZTEST(adcs_task, test_console_only_provider_never_actuates)
{
	const struct adcs_io *unconnected = adcs_console_only_io();
	float v[3] = {1, 2, 3};
	float l = 5.0f;

	zassert_equal(unconnected->get_sat_state(NULL), ARGUS_STATE_DETUMBLING);
	zassert_equal(unconnected->read_gyro(NULL, v), -ENODEV);
	zassert_equal(unconnected->read_mag(NULL, v), -ENODEV);
	zassert_equal(unconnected->read_lux(NULL, ADCS_FACE_XP, &l), -ENODEV);
	zassert_equal(unconnected->set_gyro_range(NULL, ADCS_GYRO_RANGE_125DPS), -ENODEV);
	zassert_equal(unconnected->set_coil(NULL, ADCS_COIL_XP, 0.5f), -ENODEV);
	zassert_equal(unconnected->set_coil_power(NULL, true), -ENODEV);
	for (int i = 0; i < ADCS_COIL_COUNT; ++i) {
		zassert_false(unconnected->coil_available(NULL, i));
	}
	zassert_equal(v[0], 1.0f);
	zassert_equal(l, 5.0f);
}

ZTEST_SUITE(adcs_python_parity, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(adcs_algorithms, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(adcs_task, NULL, NULL, before, NULL, NULL);
