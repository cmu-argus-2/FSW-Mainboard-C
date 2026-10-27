/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/regulator/fake.h>
#include <zephyr/ztest.h>

#include "drv8235_emul.h"
#include "../../../../samples/drv8235_diag/src/safety.h"

DEFINE_FFF_GLOBALS;

static const struct device *const axis = DEVICE_DT_GET(DT_NODELABEL(drv8235_test));
static const struct device *const supply = DEVICE_DT_GET(DT_NODELABEL(test_coil_power));
static const struct emul *const emulator = EMUL_DT_GET(DT_NODELABEL(drv8235_test));

static void diagnostic_before(void *fixture)
{
	ARG_UNUSED(fixture);
	if (!regulator_is_enabled(supply)) {
		zassert_ok(regulator_enable(supply));
	}
	drv8235_emul_power_cycle(emulator);
	zassert_ok(drv8235_reconfigure(axis));
	zassert_ok(drv8235_set_output_uv(axis, 1000000));
	zassert_ok(drv8235_set_enabled(axis, true));
}

ZTEST(drv8235_diagnostic, test_normal_stop_keeps_rail_armed_but_disables_bridge)
{
	bool armed = true;
	bool enabled;

	zassert_ok(drv8235_diag_stop_axis(axis, supply, &armed));
	zassert_true(armed);
	zassert_true(regulator_is_enabled(supply));
	zassert_ok(drv8235_is_enabled(axis, &enabled));
	zassert_false(enabled);
}

ZTEST(drv8235_diagnostic, test_i2c_shutdown_failure_removes_shared_supply)
{
	bool armed = true;

	drv8235_emul_set_error(emulator, -ETIMEDOUT);
	zassert_equal(drv8235_diag_stop_axis(axis, supply, &armed), -ETIMEDOUT);
	zassert_false(armed);
	zassert_false(regulator_is_enabled(supply));
}

ZTEST(drv8235_diagnostic, test_failed_rail_shutdown_remains_observable)
{
	bool armed = true;

	drv8235_emul_set_error(emulator, -ETIMEDOUT);
	regulator_fake_disable_fake.return_val = -EIO;
	zassert_equal(drv8235_diag_stop_axis(axis, supply, &armed), -ETIMEDOUT);
	zassert_false(armed, "prevent another pulse even if the rail cannot be removed");
	zassert_true(regulator_is_enabled(supply), "do not report a failed shutdown as OFF");
	zassert_equal(drv8235_diag_enable_rail(supply), -EBUSY);
	regulator_fake_disable_fake.return_val = 0;
	zassert_ok(regulator_disable(supply));
	zassert_false(regulator_is_enabled(supply), "rejected arm must not add a rail reference");
}

ZTEST(drv8235_diagnostic, test_arm_from_off_takes_only_one_rail_reference)
{
	zassert_ok(regulator_disable(supply));
	zassert_ok(drv8235_diag_enable_rail(supply));
	zassert_true(regulator_is_enabled(supply));
	zassert_equal(drv8235_diag_enable_rail(supply), -EBUSY);
	zassert_ok(regulator_disable(supply));
	zassert_false(regulator_is_enabled(supply));
}

ZTEST_SUITE(drv8235_diagnostic, NULL, NULL, diagnostic_before, NULL, NULL);
