/**
 * @brief MAX17205 driver tests.
 *
 * The real driver runs unchanged against a fake gauge (max17205_emul.c) on an emulated
 * I2C bus. Each test sets the fake's registers, calls the fuel gauge API, and checks the result.
 *
 * Every register the test does not set reads back 0, so a driver that reads the wrong
 * register, or reads a shadow-RAM temperature from the main address, gets 0 and fails the
 * value check.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/ztest.h>

#include <argus/drivers/max17205.h>

#include "max17205_emul.h"

#define GAUGE_NODE DT_NODELABEL(fuel_gauge)
#define MAIN       MAX17205_MAIN_I2C_ADDR
#define SHADOW     MAX17205_SHADOW_I2C_ADDR

static const struct device *const gauge = DEVICE_DT_GET(GAUGE_NODE);
static const struct emul *const gauge_emul = EMUL_DT_GET(GAUGE_NODE);

static void max17205_before(void *fixture)
{
	ARG_UNUSED(fixture);

	max17205_emul_reset(gauge_emul);
}

ZTEST_SUITE(max17205, NULL, NULL, max17205_before, NULL, NULL);

/* Set one register, read one property, and fail the test if the read fails */
static union fuel_gauge_prop_val read_prop(uint16_t addr, uint8_t reg, uint16_t raw,
					   fuel_gauge_prop_t prop)
{
	union fuel_gauge_prop_val val = {0};

	max17205_emul_set_reg(gauge_emul, addr, reg, raw);
	zassert_ok(fuel_gauge_get_prop(gauge, prop, &val));
	return val;
}

ZTEST(max17205, test_init)
{
	zassert_true(device_is_ready(gauge));
}

/* 1/256 % per LSB */
ZTEST(max17205, test_state_of_charge)
{
	union fuel_gauge_prop_val val = read_prop(MAIN, MAX17205_EMUL_REG_REPSOC, 0x3200,
						  FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT);

	zassert_equal(val.relative_state_of_charge_pct, 50);
}

/* 5 uVh / 10 mOhm = 500 uAh per LSB */
ZTEST(max17205, test_remaining_capacity)
{
	union fuel_gauge_prop_val val = read_prop(MAIN, MAX17205_EMUL_REG_REPCAP, 3000,
						  FUEL_GAUGE_REMAINING_CAPACITY_UAH);

	zassert_equal(val.remaining_capacity_uah, 1500000, "got %u", val.remaining_capacity_uah);
}

/* 1.5625 uV / 10 mOhm = 156.25 uA per LSB, signed */
ZTEST(max17205, test_current)
{
	union fuel_gauge_prop_val val;

	val = read_prop(MAIN, MAX17205_EMUL_REG_CURRENT, 256, FUEL_GAUGE_CURRENT_UA);
	zassert_equal(val.current_ua, 40000, "got %d", val.current_ua);

	/* -256: discharging */
	val = read_prop(MAIN, MAX17205_EMUL_REG_CURRENT, 0xFF00, FUEL_GAUGE_CURRENT_UA);
	zassert_equal(val.current_ua, -40000, "got %d", val.current_ua);
}

/* 1.25 mV per LSB */
ZTEST(max17205, test_pack_voltage)
{
	union fuel_gauge_prop_val val =
		read_prop(MAIN, MAX17205_EMUL_REG_VBAT, 6400, FUEL_GAUGE_VOLTAGE_UV);

	zassert_equal(val.voltage_uv, 8000000, "got %d", val.voltage_uv);
}

/* 78.125 uV per LSB */
ZTEST(max17205, test_mid_voltage)
{
	union fuel_gauge_prop_val val =
		read_prop(MAIN, MAX17205_EMUL_REG_VCELL, 40960, MAX17205_PROP_MID_VOLTAGE_UV);

	zassert_equal(val.custom_uint, 3200000, "got %u", val.custom_uint);
}

/* 16% of a full cycle per LSB */
ZTEST(max17205, test_cycle_count)
{
	union fuel_gauge_prop_val val;

	val = read_prop(MAIN, MAX17205_EMUL_REG_CYCLES, 100, FUEL_GAUGE_CYCLE_COUNT);
	zassert_equal(val.cycle_count, 16, "got %u", val.cycle_count);

	/* Full scale */
	val = read_prop(MAIN, MAX17205_EMUL_REG_CYCLES, 0xFFFF, FUEL_GAUGE_CYCLE_COUNT);
	zassert_equal(val.cycle_count, 10485, "got %u", val.cycle_count);
}

/* 5.625 s per LSB */
ZTEST(max17205, test_runtime_to_empty)
{
	union fuel_gauge_prop_val val =
		read_prop(MAIN, MAX17205_EMUL_REG_TTE, 640, FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS);

	zassert_equal(val.runtime_to_empty_mins, 60, "got %u", val.runtime_to_empty_mins);
}

ZTEST(max17205, test_runtime_to_full)
{
	union fuel_gauge_prop_val val =
		read_prop(MAIN, MAX17205_EMUL_REG_TTF, 320, FUEL_GAUGE_RUNTIME_TO_FULL_MINS);

	zassert_equal(val.runtime_to_full_mins, 30, "got %u", val.runtime_to_full_mins);
}

/* TimerH: 3.2 h per LSB */
ZTEST(max17205, test_time_since_powerup)
{
	union fuel_gauge_prop_val val;

	val = read_prop(MAIN, MAX17205_EMUL_REG_TIMERH, 1, MAX17205_PROP_TIME_PWRUP_S);
	zassert_equal(val.custom_uint, 11520, "got %u", val.custom_uint);

	/* Full scale, about 24 years, still fits in 32 bits */
	val = read_prop(MAIN, MAX17205_EMUL_REG_TIMERH, 0xFFFF, MAX17205_PROP_TIME_PWRUP_S);
	zassert_equal(val.custom_uint, 754963200, "got %u", val.custom_uint);
}

/* 1/256 degC per LSB, signed, reported in 0.1 K */
ZTEST(max17205, test_temperature)
{
	union fuel_gauge_prop_val val;

	/* 25 degC */
	val = read_prop(MAIN, MAX17205_EMUL_REG_TEMP, 0x1900, FUEL_GAUGE_TEMPERATURE_DK);
	zassert_equal(val.temperature_dk, 2982, "got %u", val.temperature_dk);

	/* -10 degC */
	val = read_prop(MAIN, MAX17205_EMUL_REG_TEMP, 0xF600, FUEL_GAUGE_TEMPERATURE_DK);
	zassert_equal(val.temperature_dk, 2632, "got %u", val.temperature_dk);
}

/* Shadow RAM temperatures are already 0.1 K */
ZTEST(max17205, test_temperature_ain1)
{
	union fuel_gauge_prop_val val =
		read_prop(SHADOW, MAX17205_EMUL_REG_TEMP1, 2982, MAX17205_PROP_TEMP_AIN1_DK);

	zassert_equal(val.custom_int, 2982, "got %d", val.custom_int);
}

ZTEST(max17205, test_temperature_ain2)
{
	union fuel_gauge_prop_val val =
		read_prop(SHADOW, MAX17205_EMUL_REG_TEMP2, 2932, MAX17205_PROP_TEMP_AIN2_DK);

	zassert_equal(val.custom_int, 2932, "got %d", val.custom_int);
}

ZTEST(max17205, test_temperature_die)
{
	union fuel_gauge_prop_val val =
		read_prop(SHADOW, MAX17205_EMUL_REG_INTTEMP, 3032, MAX17205_PROP_TEMP_DIE_DK);

	zassert_equal(val.custom_int, 3032, "got %d", val.custom_int);
}

ZTEST(max17205, test_unsupported_property)
{
	union fuel_gauge_prop_val val;

	zassert_equal(fuel_gauge_get_prop(gauge, FUEL_GAUGE_DESIGN_CAPACITY, &val), -ENOTSUP);
}

ZTEST(max17205, test_bus_error)
{
	union fuel_gauge_prop_val val;

	max17205_emul_set_nack(gauge_emul, true);
	zassert_equal(fuel_gauge_get_prop(gauge, FUEL_GAUGE_VOLTAGE_UV, &val), -EIO);
}

/* Reset sets POR_CMD (bit 0) in Config2 */
ZTEST(max17205, test_reset)
{
	zassert_ok(max17205_reset(gauge));

	uint16_t config2 = max17205_emul_get_reg(gauge_emul, MAIN, MAX17205_EMUL_REG_CONFIG2);

	zassert_equal(config2, 0x0001, "Config2 is 0x%04x", config2);
}
