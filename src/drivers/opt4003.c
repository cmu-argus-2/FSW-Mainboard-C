/**
 * @author John Buttles
 *
 * @brief Driver for Texas Instruments' Automotive-rated (AECQ-100) digital light sensor
 *
 * Port of flight/hal/drivers/opt4003.py. Exposes lux through the Zephyr sensor API:
 *
 *     sensor_sample_fetch(dev);
 *     sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &val);
 */

#define DT_DRV_COMPAT ti_opt4003

#include "argus/drivers/opt4003.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(opt4003, CONFIG_SENSOR_LOG_LEVEL);

/* Registers as described on page 25 of the datasheet */
#define OPT4003_REG_RESULT_MSB_CH0 0x00
#define OPT4003_REG_RESULT_LSB_CH0 0x01
#define OPT4003_REG_CONFIGURATION  0x0A
#define OPT4003_REG_FLAGS          0x0C
#define OPT4003_REG_DEVICE_ID      0x11

/* CONFIGURATION register fields, pages 30 and 31 of the datasheet. Bit 14 is reserved. */
#define OPT4003_CFG_QUICK_WAKEUP    BIT(15)
#define OPT4003_CFG_LUX_RANGE       GENMASK(13, 10)
#define OPT4003_CFG_CONVERSION_TIME GENMASK(9, 6)
/* TODO: OPT4003_CFG_OPERATING_MODE (5:4), _LATCH (3), _INT_POL (2), _FAULT_COUNT (1:0) */

/* DEVICE_ID register fields */
#define OPT4003_DIDL          GENMASK(13, 12)
#define OPT4003_DIDH          GENMASK(11, 0)
#define OPT4003_DIDH_EXPECTED 0x221

/* Longest conversion time is 800 ms; same timeout as the Python driver */
#define OPT4003_READY_TIMEOUT_MS 1100

/* lux = ADC codes * 535e-6, pages 17 and 18 of the datasheet */
#define OPT4003_MICROLUX_PER_CODE 535

/* Read-only, one per sensor, built at compile time from devicetree */
struct opt4003_config {
	struct i2c_dt_spec i2c;
	uint16_t config; /* value written to OPT4003_REG_CONFIGURATION */
};

/* Mutable, one per sensor, updated by sample_fetch */
struct opt4003_data {
	uint32_t adc_codes; /* mantissa << exponent from the last fetch */
	uint16_t flags;     /* FLAGS register from the last fetch */
};

/* Python: read_u16() */
static int opt4003_read_reg(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct opt4003_config *cfg = dev->config;
	uint8_t buf[2];

	int ret = i2c_burst_read_dt(&cfg->i2c, reg, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		LOG_ERR("%s: read 0x%02x failed (%d)", dev->name, reg, ret);
		return ret;
	}

	uint16_t value = sys_get_be16(buf);

	*val = value;
	return 0;
}

/* Python: the RWBit/RWBits setters, but writing the whole register at once */
static int opt4003_write_reg(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct opt4003_config *cfg = dev->config;
	uint8_t buf[2] = {0};

	sys_put_be16(val, buf);

	int ret = i2c_burst_write_dt(&cfg->i2c, reg, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		LOG_ERR("%s: write 0x%02x failed (%d)", dev->name, reg, ret);
		return ret;
	}

	return 0;
}

/* Python: check_id(). Returns 0 if the ID matches, -ENODEV if not, or an I2C error. */
static int opt4003_check_id(const struct device *dev)
{
	uint16_t id;
	int ret;

	ret = opt4003_read_reg(dev, OPT4003_REG_DEVICE_ID, &id);
	if (ret < 0) {
		return ret;
	}

	if (FIELD_GET(OPT4003_DIDL, id) != 0 || FIELD_GET(OPT4003_DIDH, id) != OPT4003_DIDH_EXPECTED) {
		LOG_ERR("%s: unexpected device id 0x%04x", dev->name, id);
		return -ENODEV;
	}

	return 0;
}

/*
 * Python: the conversion_ready_flag polling loop at the top of result_of_addr().
 * Leaves the FLAGS register in data->flags so opt4003_get_flags() can report it.
 */
static int opt4003_wait_ready(const struct device *dev)
{
	struct opt4003_data *data = dev->data;
	int64_t deadline = k_uptime_get() + OPT4003_READY_TIMEOUT_MS;
	int ret;

	do {
		ret = opt4003_read_reg(dev, OPT4003_REG_FLAGS, &data->flags);
		if (ret < 0) {
			return ret;
		}

		/* Check conversion ready bit (b2) in OPT4003_REG_FLAGS (0x0C) */
		if (((data->flags & 0x04) >> 2) == 1) {
			return 0;
		}

		k_msleep(1);
	} while (k_uptime_get() < deadline);

	return -ETIMEDOUT;
}

/*
 * CRC from the RESULT_LSB_CH0 comment in the Python driver.
 * E = exponent (4 bits), R = mantissa (20 bits), C = counter (4 bits):
 *   X[0] = XOR of every bit of E, R, C
 *   X[1] = XOR(C[1],C[3], R[1],R[3],...,R[19], E[1],E[3])  -> masks 0xA, 0xAAAAA, 0xA
 *   X[2] = XOR(C[3], R[3],R[7],R[11],R[15],R[19], E[3])     -> masks 0x8, 0x88888, 0x8
 *   X[3] = XOR(R[3],R[11],R[19])                             -> mask 0x80808 on R only
 * Returns X[3:0] with X[0] in bit 0. __builtin_parity(x) gives the XOR of all bits of x.
 */
static uint8_t opt4003_crc(uint8_t exponent, uint32_t mantissa, uint8_t counter)
{
	uint8_t x[4] = {0};

	x[0] = __builtin_parity(exponent ^ mantissa ^ counter);
	x[1] = __builtin_parity((exponent & 0xAA) ^ (mantissa & 0xAAAAAAAA) ^ (counter & 0xAA));
	x[2] = __builtin_parity((exponent & 0x88) ^ (mantissa & 0x88888888) ^ (counter & 0x88));
	x[3] = __builtin_parity(mantissa & 0x80808);

	return x[0] | (x[1] << 1) | (x[2] << 2) | (x[3] << 3);
}

/* Python: result_of_addr() */
static int opt4003_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct opt4003_config *cfg = dev->config;
	struct opt4003_data *data = dev->data;
	uint8_t buf[4];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	ret = opt4003_wait_ready(dev);
	if (ret < 0) {
		LOG_ERR("%s: conversion not ready (%d)", dev->name, ret);
		return ret;
	}

	/*
	 * Read RESULT_MSB_CH0 and RESULT_LSB_CH0 in one 4-byte burst so both halves come from
	 * the same conversion (the Python driver does two separate reads).
	 *
	 *   buf[0] = RESULT_MSB_CH0[15:8]  EXPONENT[3:0] | RESULT_MSB[11:8]
	 *   buf[1] = RESULT_MSB_CH0[7:0]   RESULT_MSB[7:0]
	 *   buf[2] = RESULT_LSB_CH0[15:8]  RESULT_LSB[7:0]
	 *   buf[3] = RESULT_LSB_CH0[7:0]   COUNTER[3:0] | CRC[3:0]
	 */
	ret = i2c_burst_read_dt(&cfg->i2c, OPT4003_REG_RESULT_MSB_CH0, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	uint8_t exponent = (buf[0] & 0xF0) >> 4;
	uint32_t mantissa = ((uint32_t)(buf[0] & 0x0F) << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	uint8_t counter = (buf[3] & 0xF0) >> 4;
	uint8_t crc = (buf[3] & 0x0F);
	
	uint8_t calc_crc = opt4003_crc(exponent, mantissa, counter);
	if (calc_crc != crc) {
		LOG_ERR("%s: crc read 0x%02x, but does not match 0x%02x", dev->name, crc, calc_crc);
		return -EIO;
	}

	data->adc_codes = mantissa << exponent;

	return 0;
}

/* Python: lux() */
static int opt4003_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct opt4003_data *data = dev->data;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	int ret = sensor_value_from_micro(val, (int64_t)data->adc_codes * OPT4003_MICROLUX_PER_CODE);
	if (ret < 0) {
		LOG_ERR("%s: channel get %d failed (%d)", dev->name, chan, ret);
		return ret;
	}

	return 0;
}

/* Python: device_errors */
uint16_t opt4003_get_flags(const struct device *dev)
{
	struct opt4003_data *data = dev->data;

	return data->flags;
}

static DEVICE_API(sensor, opt4003_api) = {
	.sample_fetch = opt4003_sample_fetch,
	.channel_get = opt4003_channel_get,
};

/* Python: __init__(). Runs once per sensor at boot; a non-zero return marks it not ready. */
static int opt4003_init(const struct device *dev)
{
	const struct opt4003_config *cfg = dev->config;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	ret = opt4003_check_id(dev);
	if (ret < 0) {
		LOG_ERR("%s: device id check failed (%d)", dev->name, ret);
		return ret;
	}

	return opt4003_write_reg(dev, OPT4003_REG_CONFIGURATION, cfg->config);
}

/*
 * Builds the CONFIGURATION register from the devicetree properties of instance `inst`.
 * DT property `lux-range` is read as DT_INST_PROP(inst, lux_range) (dashes become underscores).
 * Boolean properties read as 0 or 1.
 */
#define OPT4003_CONFIG_REG(inst)                                                             \
	((DT_INST_PROP(inst, quick_wakeup) ? OPT4003_CFG_QUICK_WAKEUP : 0) |                  \
	 FIELD_PREP(OPT4003_CFG_LUX_RANGE, DT_INST_PROP(inst, lux_range)) |                  \
	 FIELD_PREP(OPT4003_CFG_CONVERSION_TIME, DT_INST_PROP(inst, conversion_time))         \
	 /* TODO: operating mode, latch, int polarity, fault count */)

/* Instantiates config, data and the device for one devicetree node */
#define OPT4003_DEFINE(inst)                                                                 \
	static struct opt4003_data opt4003_data_##inst;                                      \
	static const struct opt4003_config opt4003_config_##inst = {                        \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                          \
		.config = OPT4003_CONFIG_REG(inst),                                         \
	};                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, opt4003_init, NULL, &opt4003_data_##inst,         \
				     &opt4003_config_##inst, POST_KERNEL,                    \
				     CONFIG_SENSOR_INIT_PRIORITY, &opt4003_api);

/* Runs OPT4003_DEFINE once for every `compatible = "ti,opt4003"` node with status okay */
DT_INST_FOREACH_STATUS_OKAY(OPT4003_DEFINE)
