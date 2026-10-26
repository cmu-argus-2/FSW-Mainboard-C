/**
 * @author Ailisi Bao, John Buttles, Jacob Rohozen
 *
 * @brief Driver for Texas Instruments' Automotive-rated (AECQ-100) digital light sensor
 *
 * Read lux through the standard Zephyr sensor API (SENSOR_CHAN_LIGHT).
 * Per-sensor settings come from devicetree, see dts/bindings/sensor/ti,opt4003.yaml.
 */

#ifndef ARGUS_DRIVERS_OPT4003_H_
#define ARGUS_DRIVERS_OPT4003_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* Bit positions in the FLAGS register (0x0C), page 31 of the datasheet */
enum opt4003_flag {
	OPT4003_FLAG_LOWER_THAN_THRESHOLD = 0,
	OPT4003_FLAG_HIGHER_THAN_THRESHOLD,
	OPT4003_FLAG_CONVERSION_READY,
	OPT4003_FLAG_OVERLOAD,
};

/**
 * @brief FLAGS register as read by the last sensor_sample_fetch().
 *
 * Test a flag with: opt4003_get_flags(dev) & BIT(OPT4003_FLAG_OVERLOAD)
 */
uint16_t opt4003_get_flags(const struct device *dev);

#endif /* ARGUS_DRIVERS_OPT4003_H_ */
