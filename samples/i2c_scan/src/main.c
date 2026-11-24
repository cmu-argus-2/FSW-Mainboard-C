/*
 * Read-only scan of every I2C bus. Every few seconds it probes each 7-bit address and
 * compares what answers with the devices the devicetree puts on that bus.
 *
 * Each probe is a 1-byte read, so nothing is written to any device. Zephyr's "i2c scan"
 * shell command asks for a zero-length write instead. The DesignWare controller can't
 * send one, so its driver sends one byte from an uninitialized buffer, which can move a
 * device's register pointer.
 *
 * No device drivers are built into this app, so nothing else uses the buses during a scan.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/* Skip the reserved addresses at both ends, as i2cdetect does */
#define FIRST_ADDR 0x08
#define LAST_ADDR  0x77

#define SCAN_PERIOD_MS 5000

struct expected_dev {
	uint16_t addr;
	const char *label;  /* devicetree node label, e.g. "light_zp_xp" */
	const char *compat; /* first compatible, e.g. "ti,opt4003" */
};

struct i2c_bus {
	const char *label;
	const struct device *dev;
	uint32_t bitrate;
	const struct expected_dev *expected;
	size_t num_expected;
};

/* A node's label as a string, or "" if it has none (several would run together) */
#define LABEL_STR(nodelabel) STRINGIFY(nodelabel)
#define NODE_LABELS(node_id) "" DT_FOREACH_NODELABEL(node_id, LABEL_STR)

#define EXPECTED_DEV(node_id)                                                                \
	{DT_REG_ADDR(node_id), NODE_LABELS(node_id), DT_PROP_BY_IDX(node_id, compatible, 0)},

/* One table of enabled child devices per enabled I2C controller */
#define BUS_EXPECTED_TABLE(bus_id)                                                           \
	static const struct expected_dev DT_CAT(bus_id, _expected)[] = {                       \
		DT_FOREACH_CHILD_STATUS_OKAY(bus_id, EXPECTED_DEV)};

DT_FOREACH_STATUS_OKAY(snps_designware_i2c, BUS_EXPECTED_TABLE)

#define BUS(bus_id)                                                                          \
	{                                                                                    \
		.label = NODE_LABELS(bus_id),                                                \
		.dev = DEVICE_DT_GET(bus_id),                                                \
		.bitrate = DT_PROP(bus_id, clock_frequency),                                 \
		.expected = DT_CAT(bus_id, _expected),                                       \
		.num_expected = ARRAY_SIZE(DT_CAT(bus_id, _expected)),                       \
	},

/* Every I2C controller on the RP2350 is a DesignWare block */
static const struct i2c_bus buses[] = {DT_FOREACH_STATUS_OKAY(snps_designware_i2c, BUS)};

static const char *error_meaning(int err)
{
	switch (err) {
	case -ETIME:
		return "SDA or SCL stuck low";
	case -ETIMEDOUT:
		return "transfer timed out";
	default:
		return "unexpected error";
	}
}

static void scan_bus(const struct i2c_bus *bus)
{
	int result[LAST_ADDR + 1];
	unsigned int answered = 0;
	unsigned int missing = 0;
	unsigned int other_errors = 0;
	int first_other_error = 0;

	if (!device_is_ready(bus->dev)) {
		printk("%s: controller not ready\n", bus->label);
		return;
	}

	/* Probe everything first so printing doesn't stretch out the scan */
	for (uint16_t addr = FIRST_ADDR; addr <= LAST_ADDR; addr++) {
		uint8_t byte;

		result[addr] = i2c_read(bus->dev, &byte, sizeof(byte), addr);
		if (result[addr] == 0) {
			answered++;
		} else if (result[addr] != -EIO) {
			/* -EIO is a NACK (or lost arbitration); anything else is a bus problem */
			if (other_errors++ == 0) {
				first_other_error = result[addr];
			}
		}
	}

	for (size_t i = 0; i < bus->num_expected; i++) {
		uint16_t addr = bus->expected[i].addr;

		if (addr < FIRST_ADDR || addr > LAST_ADDR || result[addr] != 0) {
			missing++;
		}
	}

	printk("%s at %u kHz: %u answered, %u of %zu expected missing\n", bus->label,
	       bus->bitrate / 1000, answered, missing, bus->num_expected);

	/* Devicetree addresses the scan skips, which should never happen */
	for (size_t i = 0; i < bus->num_expected; i++) {
		const struct expected_dev *e = &bus->expected[i];

		if (e->addr < FIRST_ADDR || e->addr > LAST_ADDR) {
			printk("  0x%02x  %-22s%-18snot scanned (reserved address)\n", e->addr,
			       e->label, e->compat);
		}
	}

	for (uint16_t addr = FIRST_ADDR; addr <= LAST_ADDR; addr++) {
		bool is_expected = false;

		for (size_t i = 0; i < bus->num_expected; i++) {
			const struct expected_dev *e = &bus->expected[i];

			if (e->addr != addr) {
				continue;
			}
			is_expected = true;

			if (result[addr] == 0) {
				printk("  0x%02x  %-22s%-18sok\n", addr, e->label, e->compat);
			} else if (result[addr] == -EIO) {
				printk("  0x%02x  %-22s%-18sMISSING (no ACK)\n", addr, e->label,
				       e->compat);
			} else {
				printk("  0x%02x  %-22s%-18sMISSING (error %d)\n", addr, e->label,
				       e->compat, result[addr]);
			}
		}

		if (!is_expected && result[addr] == 0) {
			printk("  0x%02x  %-40sanswers, not in devicetree\n", addr, "");
		}
	}

	if (other_errors > 0) {
		printk("  %u probes failed with error %d (%s)\n", other_errors, first_other_error,
		       error_meaning(first_other_error));
	}
}

int main(void)
{
	for (unsigned int pass = 0;; pass++) {
		printk("--- I2C scan %u (0x%02x-0x%02x)\n", pass, FIRST_ADDR, LAST_ADDR);
		for (size_t i = 0; i < ARRAY_SIZE(buses); i++) {
			scan_bus(&buses[i]);
		}
		k_msleep(SCAN_PERIOD_MS);
	}
}
