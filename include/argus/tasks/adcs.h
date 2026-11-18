/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_TASKS_ADCS_H_
#define ARGUS_TASKS_ADCS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "argus/apps/adcs/adcs.h"
#include "argus/sat_state.h"

/* Gyro full-scale codes, as passed to the Python BMX160 gyro_range setter. */
enum adcs_gyro_range {
	ADCS_GYRO_RANGE_2000DPS = 0,
	ADCS_GYRO_RANGE_125DPS = 4,
};

/* Timing of the control cycles, from Python tasks/adcs.py. */
#define ADCS_PERIOD_MS 1000
#define ADCS_MAG_SAMPLE_DT_MS 80
#define ADCS_BDOT_COIL_ON_MS 500
#define ADCS_BCROSS_DURATION_MS 1000
#define ADCS_GYRO_INTERVAL_MS 50

/* Python satellite_config adcs_config defaults. */
#define ADCS_DEFAULT_CONTROLLER ADCS_CTRL_BDOT

/* Read-success bits. A set bit means the record field holds a measurement; a
 * clear bit means it is a zero placeholder (failed read, or a gyro/mag sample
 * that failed its plausibility check).
 */
#define ADCS_VALID_GYRO (1U << 0)
#define ADCS_VALID_MAG (1U << 1)
#define ADCS_VALID_LIGHT(face) (1U << (2 + (face)))
#define ADCS_VALID_ALL (ADCS_VALID_LIGHT(ADCS_LIGHT_COUNT) - 1U)

/* One telemetry record in Python ADCS_IDX order, with the data handler
 * format "LBBffffffBfffHHHHHHHHHBBBBBB" (28 fields).
 * Mode and coil status describe the previous control cycle: Python logs right
 * after the sensor snapshot, before the mode update and actuation.
 */
struct adcs_record {
	uint32_t time;
	uint8_t mode;
	uint8_t controller;
	float gyro[3];        /* rad/s */
	float mag[3];         /* T, bias and scale corrected */
	uint8_t sun_status;
	float sun[3];         /* unit vector */
	uint16_t lux[ADCS_LIGHT_COUNT]; /* lux / 3, truncated and wrapped to 16 bits */
	uint8_t coil_status[ADCS_COIL_COUNT];
};

#define ADCS_FIELD_COUNT 28U
#define ADCS_RECORD_BUFFER_SIZE 512U

struct adcs_config {
	enum adcs_controller controller;
	float mag_bias_ut[3];
	float mag_scale[3];
};

/* Hardware callbacks. int-returning callbacks give 0 on success or a negative
 * errno. Read callbacks use driver-decoded units: rad/s, uT and lux.
 */
struct adcs_io {
	void *context;
	enum argus_sat_state (*get_sat_state)(void *);
	int (*read_gyro)(void *, float rad_s[3]);
	int (*read_mag)(void *, float ut[3]);
	int (*set_gyro_range)(void *, enum adcs_gyro_range);
	int (*read_lux)(void *, enum adcs_face, float *lux);
	/* Python TORQUE_DRIVERS_AVAILABLE: the driver booted and is usable. */
	bool (*coil_available)(void *, enum adcs_coil);
	/* Normalized throttle in [-1, 1]; 0 stops the coil. */
	int (*set_coil)(void *, enum adcs_coil, float throttle);
	/* COIL_EN for all coil drivers. Turning on must also re-initialize the
	 * DRV8235s (drv8235_init) before returning, as Python enable_coils does.
	 */
	int (*set_coil_power)(void *, bool on);
	int64_t (*uptime_ms)(void *);
	void (*sleep_ms)(void *, int32_t ms);
	void (*publish)(void *, const struct adcs_record *, uint16_t valid_reads);
};

struct adcs_state {
	struct adcs_config config;
	float mag_bias_t[3];
	enum adcs_mode mode;
	enum adcs_status gyro_status;
	enum adcs_status mag_status;
	enum adcs_status sun_status;
	float gyro[3];
	float mag[3];
	float sun[3];
	float lux[ADCS_LIGHT_COUNT];
	bool coil_status[ADCS_COIL_COUNT];
	bool coils_off;
	bool coil_hw_disabled;
	bool gyro_range_set;
	enum adcs_gyro_range gyro_range;
	struct adcs_record record;
};

/* config may be NULL for the Python defaults (B-dot, zero bias, unit scale). */
void adcs_init(struct adcs_state *state, const struct adcs_config *config);

/* Command hooks (Python ControllerModes.update_mode, sensors.update_mag_bias).
 * They take effect on the next step. Persistence and cross-thread access are
 * the caller's responsibility: call them from the ADCS thread.
 */
int adcs_set_controller(struct adcs_state *state, enum adcs_controller controller);
void adcs_set_mag_bias_ut(struct adcs_state *state, const float bias_ut[3]);

/* One 1 Hz invocation, including the blocking control cycle (~0.9 s for
 * B-dot, ~1 s for B-cross and sun pointing).
 */
void adcs_step(struct adcs_state *state, const struct adcs_io *io, uint32_t time_s);

/* CSV with an "ADCS" label followed by the 28 fields. */
int adcs_format_record(char *buffer, size_t size, const struct adcs_record *record);

/* Run the task forever at 1 Hz with hardware callbacks from integration code. */
void adcs_run(const struct adcs_io *io);

/* Console-only bring-up: every hardware operation returns -ENODEV, no coil is
 * available and the satellite state is fixed at DETUMBLING so the read and
 * mode path runs. Replace with driver-backed callbacks for hardware.
 */
const struct adcs_io *adcs_console_only_io(void);

#endif
