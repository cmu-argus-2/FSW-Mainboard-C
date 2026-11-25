/* SPDX-License-Identifier: Apache-2.0 */

/*
 * C port of FSW-mainboard scripts/mag_hyst.py. For each coil in turn (XP, XM, YP, YM,
 * ZP, ZM) it steps the throttle from -1.0 to +1.0 in 61 steps with every other coil at
 * 0 (brake), waits 200 ms, then prints the BMX160 field and every coil's voltage and
 * current. The CSV columns match mag_hyst.py, so python/compare_mag_hyst.py can match
 * the two logs row by row:
 *
 *     coil,step,time_ns,mag_x,mag_y,mag_z,voltage_xp..voltage_zm,current_xp..current_zm
 *
 * Field in uT, as Python IMU.mag(). Voltage and current are the C driver's readings
 * (its current scale is the EPS team's 3.7 / 192 A per count, not Python's 0.01451).
 * A failed read prints nan. A coil whose driver doesn't answer at start-up (panel not
 * connected) isn't swept and reads nan, as in python/mag_hyst.py.
 *
 * Differences from the script, none of which change the rows:
 * - It doesn't boot the satellite: only the peripheral rail, COIL_EN, the six coil
 *   drivers and the IMU are used, so the external watchdog is never armed.
 * - It starts 1 s after the capture opens the USB port, and stops if the port closes.
 *   Each new open runs a new sweep, so repeat runs don't need a reflash.
 * - Opening the port at 1200 baud reboots into BOOTSEL (the Arduino convention), so a
 *   new build can be flashed without holding BOOT.
 * - At the end it also coasts every coil and turns COIL_EN off.
 */

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <pico/bootrom.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "argus/drivers/drv8235.h"

/* As in mag_hyst.py */
#define N_STEPS   60
#define SETTLE_MS 200

#define COIL_COUNT 6

/* mag_hyst.py COILS order */
static const char *const names[COIL_COUNT] = {"XP", "XM", "YP", "YM", "ZP", "ZM"};
static const struct device *const coils[COIL_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(torque_xp)), DEVICE_DT_GET(DT_NODELABEL(torque_xm)),
	DEVICE_DT_GET(DT_NODELABEL(torque_yp)), DEVICE_DT_GET(DT_NODELABEL(torque_ym)),
	DEVICE_DT_GET(DT_NODELABEL(torque_zp)), DEVICE_DT_GET(DT_NODELABEL(torque_zm)),
};
/* For reading the configuration back directly, bypassing the driver */
static const struct i2c_dt_spec buses[COIL_COUNT] = {
	I2C_DT_SPEC_GET(DT_NODELABEL(torque_xp)), I2C_DT_SPEC_GET(DT_NODELABEL(torque_xm)),
	I2C_DT_SPEC_GET(DT_NODELABEL(torque_yp)), I2C_DT_SPEC_GET(DT_NODELABEL(torque_ym)),
	I2C_DT_SPEC_GET(DT_NODELABEL(torque_zp)), I2C_DT_SPEC_GET(DT_NODELABEL(torque_zm)),
};
static bool ready[COIL_COUNT];
static bool reported[COIL_COUNT]; /* first command failure already printed */

static const struct device *const coil_power = DEVICE_DT_GET(DT_NODELABEL(coil_power));
static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(imu));
static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Python: round(-1.0 + i * STEP_SIZE, 2). No step lands on a rounding tie. */
static float step_value(int i)
{
	double v = round((-1.0 + i * (2.0 / N_STEPS)) * 100.0) / 100.0;

	return (float)(v + 0.0); /* + 0.0 turns -0.0 into 0.0 */
}

/* 1200 baud is the request to reboot into BOOTSEL. */
static void bootsel_if_requested(void)
{
	uint32_t baud = 0;

	if (uart_line_ctrl_get(console, UART_LINE_CTRL_BAUD_RATE, &baud) == 0 && baud == 1200) {
		printk("1200 baud: rebooting into BOOTSEL\n");
		k_msleep(100);
		reset_usb_boot(0, 0);
	}
}

/* True while a terminal has the USB serial port open (DTR set). */
static bool terminal_open(void)
{
	uint32_t dtr = 0;

	if (uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr) != 0) {
		return true; /* not a USB console: nothing to wait for */
	}
	return dtr != 0;
}

static int read_mag_ut(double ut[3])
{
	struct sensor_value v[3];
	int ret = device_is_ready(imu) ? sensor_sample_fetch_chan(imu, SENSOR_CHAN_MAGN_XYZ)
				       : -ENODEV;

	if (ret == 0) {
		ret = sensor_channel_get(imu, SENSOR_CHAN_MAGN_XYZ, v);
	}
	for (int i = 0; i < 3; ++i) {
		ut[i] = ret == 0 ? sensor_value_to_double(&v[i]) * 100.0 : (double)NAN; /* G -> uT */
	}
	return ret;
}

/* Every coil to 0 (brake), as the script does before and after the sweep. */
static void all_zero(void)
{
	for (int c = 0; c < COIL_COUNT; ++c) {
		if (ready[c]) {
			(void)drv8235_set_throttle(coils[c], 0.0f);
		}
	}
}

static void shut_down(void)
{
	for (int c = 0; c < COIL_COUNT; ++c) {
		int ret = drv8235_coast(coils[c]);

		if (ret != 0 && ready[c]) {
			printk("%s: coast failed (%d)\n", names[c], ret);
		}
	}
	int ret = regulator_disable(coil_power);

	if (ret != 0 || regulator_is_enabled(coil_power)) {
		printk("COIL_EN off failed (%d): REMOVE EXTERNAL POWER\n", ret);
	}
}

/* FAULT_STATUS (reg 0x00): bit 7 FAULT, 5 STALL, 4 OCP, 3 OVP, 2 TSD, 1 NPOR. */
static void print_faults(int c, const char *when)
{
	struct drv8235_faults f;
	int ret = drv8235_get_faults(coils[c], &f);

	if (ret != 0) {
		printk("%s: FAULT_STATUS read failed (%d) %s\n", names[c], ret, when);
		return;
	}
	/* A chip that has really reset is back at its defaults: EN_OUT (CONFIG0 bit 7) and
	 * I2C_BC/PMODE (CONFIG4 bits 3:2) clear. Still set means it kept its configuration.
	 */
	uint8_t config0 = 0, config4 = 0;
	int r0 = i2c_reg_read_byte_dt(&buses[c], 0x09, &config0);
	int r4 = i2c_reg_read_byte_dt(&buses[c], 0x0D, &config4);

	printk("%s: FAULT_STATUS 0x%02x %s (NPOR bit %d; COIL_EN %s)", names[c], f.raw, when,
	       (f.raw >> 1) & 1, regulator_is_enabled(coil_power) ? "on" : "off");
	if (r0 == 0 && r4 == 0) {
		printk(" CONFIG0 0x%02x (EN_OUT %d) CONFIG4 0x%02x (I2C_BC/PMODE %d%d)\n", config0,
		       (config0 >> 7) & 1, config4, (config4 >> 3) & 1, (config4 >> 2) & 1);
	} else {
		printk(" config read failed (%d, %d)\n", r0, r4);
	}
}

static bool bring_up(void)
{
	/* The Python HAL turns COIL_EN on at import; regulator-fixed waits 100 ms. */
	int ret = regulator_enable(coil_power);
	bool any = false;

	if (ret != 0) {
		printk("COIL_EN on failed (%d)\n", ret);
		return false;
	}
	for (int c = 0; c < COIL_COUNT; ++c) {
		uint8_t status;

		/* Just powered by COIL_EN: NPOR (bit 1) should be set until init clears it. */
		if (i2c_reg_read_byte_dt(&buses[c], 0x00, &status) == 0) {
			printk("%s: FAULT_STATUS 0x%02x before init (NPOR bit %d)\n", names[c],
			       status, (status >> 1) & 1);
		}
		ret = drv8235_init(coils[c]);
		ready[c] = ret == 0;
		any |= ready[c];
		printk("%s: init %s (%d)\n", names[c], ready[c] ? "ok" : "FAILED", ret);
		if (ready[c]) {
			print_faults(c, "after init");
		}
	}
	if (!any) {
		printk("no coil driver answered; is VBATT connected?\n");
	}
	return any;
}

/* One row: the coil being swept, its step, then the field and all twelve readings. */
static void print_row(int coil, float step, int64_t time_ns, uint32_t *mag_cycles)
{
	double mag[3];
	double volts[COIL_COUNT], amps[COIL_COUNT];
	uint32_t start = k_cycle_get_32();

	(void)read_mag_ut(mag);
	*mag_cycles = k_cycle_get_32() - start;

	for (int c = 0; c < COIL_COUNT; ++c) {
		float v, a;

		if (ready[c] && drv8235_read_voltage_current(coils[c], &v, &a) == 0) {
			volts[c] = v;
			amps[c] = a;
		} else {
			volts[c] = amps[c] = (double)NAN;
		}
	}
	printk("%s,%.2f,%lld,%.6f,%.6f,%.6f", names[coil], (double)step, (long long)time_ns,
	       mag[0], mag[1], mag[2]);
	for (int c = 0; c < COIL_COUNT; ++c) {
		printk(",%.5f", volts[c]);
	}
	for (int c = 0; c < COIL_COUNT; ++c) {
		printk(",%.5f", amps[c]);
	}
	printk("\n");
}

static void sweep(void)
{
	memset(reported, 0, sizeof(reported));
	printk("drv8235_mag_hyst (C): %d coils x %d steps, %d ms settle\n", COIL_COUNT,
	       N_STEPS + 1, SETTLE_MS);
	printk("IMU: %s\n", device_is_ready(imu) ? "ready" : "NOT READY (mag columns will be nan)");
	if (!bring_up()) {
		shut_down();
		printk("ABORT\n");
		return;
	}

	printk("coil,step,time_ns,mag_x,mag_y,mag_z,voltage_xp,voltage_xm,voltage_yp,"
	       "voltage_ym,voltage_zp,voltage_zm,current_xp,current_xm,current_yp,"
	       "current_ym,current_zp,current_zm\n");

	all_zero();

	uint64_t mag_cycles_total = 0;
	uint32_t mag_count = 0;
	bool stopped = false;

	for (int coil = 0; coil < COIL_COUNT && !stopped; ++coil) {
		if (!ready[coil]) {
			printk("%s: not connected; skipped\n", names[coil]);
			continue;
		}
		for (int i = 0; i <= N_STEPS; ++i) {
			if (!terminal_open()) {
				printk("terminal closed; stopping\n");
				stopped = true;
				break;
			}
			float step = step_value(i);

			for (int c = 0; c < COIL_COUNT; ++c) {
				if (!ready[c]) {
					continue;
				}
				int ret = drv8235_set_throttle(coils[c], c == coil ? step : 0.0f);

				if (ret != 0 && !reported[c]) {
					/* Once per coil, with the chip's view, not on every step. */
					reported[c] = true;
					printk("%s: set throttle failed (%d); further failures not "
					       "printed\n", names[c], ret);
					print_faults(c, "at first failure");
				}
			}
			k_msleep(SETTLE_MS);

			int64_t time_ns = (int64_t)k_ticks_to_ns_floor64(k_uptime_ticks());
			uint32_t cycles;

			print_row(coil, step, time_ns, &cycles);
			mag_cycles_total += cycles;
			mag_count++;
		}
	}

	all_zero();
	if (mag_count > 0) {
		printk("Average mag measurement time (ns): %llu\n",
		       (unsigned long long)k_cyc_to_ns_floor64(mag_cycles_total / mag_count));
	}
	shut_down();
	printk("DONE\n");
}

int main(void)
{
	for (;;) {
		/* Start when the capture opens the port, so no rows are lost. The second
		 * lets the host finish opening it before the first lines go out.
		 */
		while (!terminal_open()) {
			k_msleep(100);
		}
		k_msleep(1000);
		bootsel_if_requested();
		sweep();
		/* Next sweep on the next open */
		while (terminal_open()) {
			bootsel_if_requested();
			k_msleep(100);
		}
	}
	return 0;
}
