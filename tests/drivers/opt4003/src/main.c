/**
 * @brief OPT4003 driver tests.
 *
 * The real driver runs unchanged against a fake sensor (opt4003_emul.c) on an emulated
 * I2C bus. Each test sets the fake's registers, calls the sensor API, and checks the result.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include <argus/drivers/opt4003.h>

#include "opt4003_emul.h"

#define LIGHT_NODE          DT_NODELABEL(light)
#define LIGHT_DEFERRED_NODE DT_NODELABEL(light_deferred)

static const struct device *const light = DEVICE_DT_GET(LIGHT_NODE);
static const struct emul *const light_emul = EMUL_DT_GET(LIGHT_NODE);

/* Clear what tests change. CONFIGURATION and DEVICE_ID keep the values boot left them with. */
static void opt4003_before(void *fixture)
{
	ARG_UNUSED(fixture);

	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_MSB_CH0, 0);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_LSB_CH0, 0);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_FLAGS, 0);
}

ZTEST_SUITE(opt4003, NULL, NULL, opt4003_before, NULL, NULL);

/* Boot ran opt4003_init(): the ID check passed and CONFIGURATION was built from devicetree */
ZTEST(opt4003, test_init_writes_config)
{
	uint16_t config = opt4003_emul_get_reg(light_emul, OPT4003_EMUL_REG_CONFIGURATION);

	zassert_true(device_is_ready(light));

	/* auto range, 600 us conversion, continuous mode, latched window */
	zassert_equal(config, 0x3038, "CONFIGURATION is 0x%04x", config);
}

/* exponent 3, mantissa 0x12345, counter 0, CRC 0x3 -> 596520 ADC codes -> 319.1382 lux */
ZTEST(opt4003, test_lux)
{
	struct sensor_value val;

	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_MSB_CH0, 0x3123);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_LSB_CH0, 0x4503);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_FLAGS, BIT(OPT4003_FLAG_CONVERSION_READY));

	zassert_ok(sensor_sample_fetch(light));
	zassert_ok(sensor_channel_get(light, SENSOR_CHAN_LIGHT, &val));
	zassert_equal(val.val1, 319, "val1 is %d", val.val1);
	zassert_equal(val.val2, 138200, "val2 is %d", val.val2);
}

ZTEST(opt4003, test_bad_crc)
{
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_MSB_CH0, 0x3123);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_LSB_CH0, 0x4504);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_FLAGS, BIT(OPT4003_FLAG_CONVERSION_READY));

	int ret = sensor_sample_fetch(light);

	zassert_equal(ret, -EIO);
}

ZTEST(opt4003, test_not_ready)
{
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_MSB_CH0, 0x3123);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_LSB_CH0, 0x4504);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_FLAGS, 0);

	int ret = sensor_sample_fetch(light);

	zassert_equal(ret, -ETIMEDOUT);
}

ZTEST(opt4003, test_wrong_channel)
{
	int ret = sensor_sample_fetch_chan(light, SENSOR_CHAN_ACCEL_XYZ);
	zassert_equal(ret, -ENOTSUP);
	
	struct sensor_value val;
	ret = sensor_channel_get(light, SENSOR_CHAN_ACCEL_X, &val);
	zassert_equal(ret, -ENOTSUP);
}

ZTEST(opt4003, test_flags)
{
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_MSB_CH0, 0x3123);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_RESULT_LSB_CH0, 0x4503);
	opt4003_emul_set_reg(light_emul, OPT4003_EMUL_REG_FLAGS, BIT(OPT4003_FLAG_CONVERSION_READY) | BIT(OPT4003_FLAG_OVERLOAD));
	zassert_ok(sensor_sample_fetch(light));

	uint16_t flags = opt4003_get_flags(light);
	zassert_true(flags & BIT(OPT4003_FLAG_OVERLOAD));
}

ZTEST(opt4003, test_bad_device_id)
{
	const struct device *dev = DEVICE_DT_GET(LIGHT_DEFERRED_NODE);
	const struct emul *emul = EMUL_DT_GET(LIGHT_DEFERRED_NODE);

	opt4003_emul_set_reg(emul, OPT4003_EMUL_REG_DEVICE_ID, 0x0121);
	int ret = device_init(dev);
	zassert_equal(ret, -ENODEV);

	bool ret1 = device_is_ready(dev);
	zassert_equal(ret1, false);
}
