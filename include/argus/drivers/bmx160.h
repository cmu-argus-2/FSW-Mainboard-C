/**
 * @brief Driver for the Bosch BMX160 9-axis IMU (BMI160 accel/gyro + BMM150 magnetometer)
 *
 * Read it through the standard Zephyr sensor API:
 *
 *     sensor_sample_fetch(dev);                   // everything in one burst
 *     sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);  // or one sensor
 *     sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, vals);
 *
 * Units follow Zephyr, not the Python driver, for the magnetometer:
 *
 * | Channel                | Unit  | Python driver |
 * |------------------------|-------|---------------|
 * | SENSOR_CHAN_GYRO_*     | rad/s | rad/s         |
 * | SENSOR_CHAN_ACCEL_*    | m/s^2 | m/s^2         |
 * | SENSOR_CHAN_MAGN_*     | gauss | uT (1 G = 100 uT) |
 * | SENSOR_CHAN_DIE_TEMP   | degC  | degC          |
 *
 * Per-device settings come from devicetree, see dts/bindings/sensor/bosch,bmx160.yaml.
 * Not thread-safe: one thread (the ADCS task) should own the device.
 */

#ifndef ARGUS_DRIVERS_BMX160_H_
#define ARGUS_DRIVERS_BMX160_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* GYR_RANGE register codes, the same values the Python gyro_range setter takes */
enum bmx160_gyro_range {
	BMX160_GYRO_RANGE_2000DPS = 0,
	BMX160_GYRO_RANGE_1000DPS = 1,
	BMX160_GYRO_RANGE_500DPS = 2,
	BMX160_GYRO_RANGE_250DPS = 3,
	BMX160_GYRO_RANGE_125DPS = 4,
};

/* ERR_REG (0x02) fields */
#define BMX160_ERR_FATAL    BIT(0)
#define BMX160_ERR_CODE     GENMASK(4, 1)
#define BMX160_ERR_I2C_FAIL BIT(5)
#define BMX160_ERR_DROP_CMD BIT(6)
#define BMX160_ERR_MAG_DRDY BIT(7)

/**
 * @brief Set the gyro full scale. Python: the gyro_range setter.
 *
 * Writes GYR_RANGE and reads it back. Later gyro readings are scaled with the range the
 * chip reports, so a failed write never leaves readings scaled for the wrong range.
 *
 * When the range changes, this sleeps 40 ms (two gyro samples) before returning, so the
 * next read is a sample taken in the new range. Python doesn't wait, and its first
 * reading after a change is off by the ratio of the two ranges (16x for 2000 <-> 125).
 * Setting the range the chip already has doesn't wait.
 *
 * @return 0, -EINVAL for an unknown range, -EIO if the read-back differs, or an I2C error.
 */
int bmx160_set_gyro_range(const struct device *dev, enum bmx160_gyro_range range);

/** @brief The gyro range readings are currently scaled with. Python: the gyro_range getter. */
enum bmx160_gyro_range bmx160_get_gyro_range(const struct device *dev);

/**
 * @brief Read ERR_REG. Python: device_errors (fatal_err, drop_cmd_err) and error_code.
 *
 * Test fields with e.g. err & BMX160_ERR_FATAL.
 */
int bmx160_read_errors(const struct device *dev, uint8_t *err);

#endif /* ARGUS_DRIVERS_BMX160_H_ */
