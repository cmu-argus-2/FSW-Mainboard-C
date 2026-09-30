/**
 * @file zephyr/drivers/i2c.h
 * @brief Host-side stand-in for Zephyr's <zephyr/drivers/i2c.h>.
 *
 * Declares just the part of the API the MAX17205 driver uses. In Zephyr these
 * are static inlines over the bus driver's i2c_transfer(); here they are plain
 * functions, implemented by the register-map fake in i2c_fake.c.
 *
 * Return values follow Zephyr: 0 on success, a negative errno on failure
 * (-EIO for a NACK, as the real controller drivers report it).
 */

#ifndef FAKE_ZEPHYR_DRIVERS_I2C_H
#define FAKE_ZEPHYR_DRIVERS_I2C_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct device {
    const char *name;
};

struct i2c_dt_spec {
    const struct device *bus;
    uint16_t addr;
};

bool i2c_is_ready_dt(const struct i2c_dt_spec *spec);

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, uint32_t num_bytes);
int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, uint32_t num_bytes);
int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *write_buf, size_t num_write,
                      void *read_buf, size_t num_read);

int i2c_burst_read_dt(const struct i2c_dt_spec *spec, uint8_t start_addr, uint8_t *buf,
                      uint32_t num_bytes);
int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t start_addr, const uint8_t *buf,
                       uint32_t num_bytes);

#endif /* FAKE_ZEPHYR_DRIVERS_I2C_H */
