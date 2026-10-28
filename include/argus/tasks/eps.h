/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARGUS_TASKS_EPS_H_
#define ARGUS_TASKS_EPS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum eps_power_flag { EPS_NONE, EPS_LOW_POWER, EPS_NOMINAL };
enum eps_monitor { EPS_MAIN, EPS_PERIPHERAL, EPS_RADIO, EPS_JETSON, EPS_MONITOR_COUNT };
enum eps_axis { EPS_XP, EPS_XM, EPS_YP, EPS_YM, EPS_ZP, EPS_ZM, EPS_COIL_COUNT };

/* Indices are the original Python core/dh_constants.py EPS_IDX wire order. */
enum eps_field {
	EPS_TIME, EPS_POWER_FLAG, EPS_CPU_TEMP_CC, EPS_MAIN_MV, EPS_MAIN_MA,
	EPS_PACK_TEMP_CC, EPS_AIN1_TEMP_CC, EPS_AIN2_TEMP_CC, EPS_DIE_TEMP_CC,
	EPS_SOC_PCT, EPS_CAPACITY_MAH, EPS_PACK_MA, EPS_PACK_MV, EPS_MIDPOINT_MV,
	EPS_TTE_S, EPS_TTF_S,
	EPS_XP_MV, EPS_XP_MA, EPS_XM_MV, EPS_XM_MA,
	EPS_YP_MV, EPS_YP_MA, EPS_YM_MV, EPS_YM_MA,
	EPS_ZP_MV, EPS_ZP_MA, EPS_ZM_MV, EPS_ZM_MA,
	EPS_JETSON_MV, EPS_JETSON_MA, EPS_RADIO_MV, EPS_RADIO_MA,
	EPS_PERIPHERAL_MV, EPS_PERIPHERAL_MA,
	EPS_SOLAR_XP_MV, EPS_SOLAR_XP_MA, EPS_SOLAR_XM_MV, EPS_SOLAR_XM_MA,
	EPS_SOLAR_YP_MV, EPS_SOLAR_YP_MA, EPS_SOLAR_YM_MV, EPS_SOLAR_YM_MA,
	EPS_HEATER1, EPS_HEATER2, EPS_FIELD_COUNT,
};

#define EPS_WARNING_COUNT 11U
#define EPS_SLOW_PERIOD 5U
#define EPS_RECORD_BUFFER_SIZE 640U

struct eps_vi {
	int32_t mv;
	int32_t ma;
};

struct eps_fuel {
	int32_t pack_temp_cc;
	int32_t ain1_temp_cc;
	int32_t ain2_temp_cc;
	int32_t die_temp_cc;
	int32_t soc_pct;
	int32_t capacity_mah;
	int32_t current_ma;
	int32_t voltage_mv;
	int32_t midpoint_mv;
	int32_t tte_s;
	int32_t ttf_s;
};

/* All callbacks return 0 on success, negative errno on an unavailable/failed read.
 * Implementations own device readiness and units. Each step commits only a whole
 * successful reading. Missing measurements remain zero, never aborting the cycle.
 */
struct eps_io {
	void *context;
	int (*read_monitor)(void *, enum eps_monitor, struct eps_vi *);
	int (*read_coil)(void *, enum eps_axis, struct eps_vi *);
	int (*read_fuel)(void *, struct eps_fuel *);
	int (*read_cpu_temp)(void *, int32_t *);
	int (*set_heaters)(void *, bool, bool);
	void (*publish)(void *, bool full, const int32_t *, size_t);
};

struct eps_state {
	int32_t fields[EPS_FIELD_COUNT];
	int32_t warnings[EPS_WARNING_COUNT];
	uint8_t phase;
	enum eps_power_flag power_flag;
	bool heater1;
	bool heater2;
};

void eps_init(struct eps_state *state);
/* One 1 Hz invocation. Slow work occurs on calls 5, 10, 15, ... . */
bool eps_step(struct eps_state *state, const struct eps_io *io, int32_t time_s);
enum eps_power_flag eps_next_power_flag(enum eps_power_flag previous, int32_t soc);
bool eps_next_heater(bool enabled, int32_t temp_cc);
bool eps_power_alert(struct eps_vi reading, int32_t threshold_mw);

/* Compatibility conversions from the EPS Task Spec, using integer truncation.
 * DRV8235 current alone differs from Python: 3.7 A / 192, not / 255.
 */
struct eps_vi eps_adm1176_convert(uint16_t raw_v, uint16_t raw_i);
struct eps_vi eps_drv8235_convert(uint8_t raw_v, uint8_t raw_i, bool reverse);
int32_t eps_pack_temperature_cc(uint16_t raw);
int32_t eps_shadow_temperature_cc(uint16_t raw);

/* CSV with a record label; label is not one of the 11/44 numeric fields. */
int eps_format_record(char *buffer, size_t size, bool full, const int32_t *fields,
		      size_t count);

/* Run the task at 1 Hz using hardware callbacks supplied by integration code.
 * Driver implementations and hardware initialization are outside this module.
 */
void eps_run(const struct eps_io *io);

/* Explicit console-only bring-up: all hardware operations return -ENODEV.
 * Does not fabricate measurements or actuate heaters. Replace this I/O provider
 * with real driver callbacks when the separate hardware integration is ready.
 */
const struct eps_io *eps_console_only_io(void);

#endif
