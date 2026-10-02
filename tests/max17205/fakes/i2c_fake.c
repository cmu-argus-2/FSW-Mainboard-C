/**
 * @file i2c_fake.c
 * @brief Implementation of the fake Zephyr I2C API. See i2c_fake.h.
 */

#include "i2c_fake.h"

#include <string.h>

i2c_fake_t i2c_fake;

static const struct device fake_bus = {.name = "i2c_fake"};

/* ---------------------------------------------------------------- setup -- */

void i2c_fake_reset(void) {
    memset(&i2c_fake, 0, sizeof(i2c_fake));
    i2c_fake.bus_ready = true;
    i2c_fake.read_fill = 0xA5;
}

const struct device *i2c_fake_bus(void) {
    return &fake_bus;
}

struct i2c_dt_spec i2c_fake_spec(uint8_t addr) {
    struct i2c_dt_spec spec = {.bus = &fake_bus, .addr = addr};
    return spec;
}

/** The device at `addr`, or NULL. */
static i2c_fake_device_t *find_device(uint8_t addr) {
    for (unsigned i = 0; i < I2C_FAKE_MAX_DEVICES; i++) {
        if (i2c_fake.dev[i].used && i2c_fake.dev[i].addr == addr) {
            return &i2c_fake.dev[i];
        }
    }
    return NULL;
}

/** The device at `addr`, creating it if there is a free slot. */
static i2c_fake_device_t *get_device(uint8_t addr) {
    i2c_fake_device_t *d = find_device(addr);
    if (d != NULL) {
        return d;
    }
    for (unsigned i = 0; i < I2C_FAKE_MAX_DEVICES; i++) {
        if (!i2c_fake.dev[i].used) {
            d = &i2c_fake.dev[i];
            memset(d, 0, sizeof(*d));
            d->used = true;
            d->addr = addr;
            return d;
        }
    }
    /* Out of slots. Raise I2C_FAKE_MAX_DEVICES if a test needs more. */
    return NULL;
}

void i2c_fake_set_present(uint8_t addr, bool present) {
    if (present) {
        (void)get_device(addr);
        return;
    }
    i2c_fake_device_t *d = find_device(addr);
    if (d != NULL) {
        memset(d, 0, sizeof(*d));
    }
}

void i2c_fake_set_reg(uint8_t addr, uint8_t reg, uint16_t value) {
    i2c_fake_device_t *d = get_device(addr);
    if (d != NULL) {
        d->reg[reg] = value;
    }
}

uint16_t i2c_fake_get_reg(uint8_t addr, uint8_t reg) {
    i2c_fake_device_t *d = find_device(addr);
    return d != NULL ? d->reg[reg] : 0;
}

void i2c_fake_fail_all(int err) {
    i2c_fake.forced = err;
}

void i2c_fake_fail_after(unsigned n_ok, int err) {
    i2c_fake.fail_after_armed = true;
    i2c_fake.fail_after_remaining = n_ok;
    i2c_fake.fail_after_status = err;
}

void i2c_fake_fail_reg(uint8_t addr, uint8_t reg, int err) {
    i2c_fake_device_t *d = get_device(addr);
    if (d != NULL) {
        d->reg_status[reg] = err;
    }
}

/* ------------------------------------------------------------ inspection -- */

const i2c_fake_xfer_t *i2c_fake_last_xfer(void) {
    if (i2c_fake.xfer_calls == 0) {
        return NULL;
    }
    unsigned n = i2c_fake.xfer_calls;
    if (n > I2C_FAKE_MAX_EVENTS) {
        n = I2C_FAKE_MAX_EVENTS;
    }
    return &i2c_fake.xfer[n - 1];
}

/** How many transfers are actually in the log. */
static unsigned logged(void) {
    return i2c_fake.xfer_calls < I2C_FAKE_MAX_EVENTS ? i2c_fake.xfer_calls : I2C_FAKE_MAX_EVENTS;
}

int i2c_fake_find_reg_read(uint8_t addr, uint8_t reg) {
    for (unsigned i = 0; i < logged(); i++) {
        const i2c_fake_xfer_t *x = &i2c_fake.xfer[i];
        if (x->op == i2c_fake_op_write || x->addr != addr || !x->reg_valid || x->r_len == 0) {
            continue;
        }
        /* A read of r_len bytes covers ceil(r_len/2) registers from x->reg. */
        unsigned span = (unsigned)((x->r_len + 1) / 2);
        for (unsigned k = 0; k < span; k++) {
            if ((uint8_t)(x->reg + k) == reg) {
                return (int)i;
            }
        }
    }
    return -1;
}

int i2c_fake_find_reg_write(uint8_t addr, uint8_t reg, uint16_t value) {
    for (unsigned i = 0; i < logged(); i++) {
        const i2c_fake_xfer_t *x = &i2c_fake.xfer[i];
        if (x->op == i2c_fake_op_read || x->addr != addr || x->w_len < 3) {
            continue;
        }
        unsigned pairs = (unsigned)((x->w_len - 1) / 2);
        for (unsigned k = 0; k < pairs && (1 + 2 * k + 1) < I2C_FAKE_MAX_BYTES; k++) {
            uint16_t v = (uint16_t)(x->w[1 + 2 * k] | ((uint16_t)x->w[1 + 2 * k + 1] << 8));
            if ((uint8_t)(x->w[0] + k) == reg && v == value) {
                return (int)i;
            }
        }
    }
    return -1;
}

/* -------------------------------------------------------------- transfers -- */

/** Status for one transfer, honouring the injected failures in precedence order. */
static int xfer_status(uint8_t addr, uint8_t reg, bool reg_valid) {
    if (i2c_fake.forced != 0) {
        return i2c_fake.forced;
    }
    if (i2c_fake.fail_after_armed) {
        if (i2c_fake.fail_after_remaining > 0) {
            i2c_fake.fail_after_remaining--;
        } else {
            return i2c_fake.fail_after_status;
        }
    }
    i2c_fake_device_t *d = find_device(addr);
    if (d == NULL) {
        return -EIO; /* nobody ACKed the address */
    }
    if (reg_valid && d->reg_status[reg] != 0) {
        return d->reg_status[reg];
    }
    return 0;
}

static void record(i2c_fake_op_t op, const struct i2c_dt_spec *spec, uint8_t reg, bool reg_valid,
                   const uint8_t *src, size_t w_len, const uint8_t *dst, size_t r_len, int ret) {
    if (i2c_fake.xfer_calls < I2C_FAKE_MAX_EVENTS) {
        i2c_fake_xfer_t *x = &i2c_fake.xfer[i2c_fake.xfer_calls];
        memset(x, 0, sizeof(*x));
        x->op = op;
        x->bus = spec->bus;
        x->addr = (uint8_t)spec->addr;
        x->reg = reg;
        x->reg_valid = reg_valid;
        x->w_len = w_len;
        x->r_len = r_len;
        x->ret = ret;
        if (src != NULL) {
            memcpy(x->w, src, w_len < I2C_FAKE_MAX_BYTES ? w_len : I2C_FAKE_MAX_BYTES);
        }
        if (dst != NULL) {
            memcpy(x->r, dst, r_len < I2C_FAKE_MAX_BYTES ? r_len : I2C_FAKE_MAX_BYTES);
        }
    }
    i2c_fake.xfer_calls++;
}

/**
 * Apply a write payload to the register map: src[0] is the register pointer,
 * any bytes after it are little-endian 16-bit values written to successive
 * registers. A dangling odd byte is recorded but not applied -- the MAX17205
 * has no half-register write.
 */
static void apply_write(i2c_fake_device_t *d, const uint8_t *src, size_t len) {
    if (d == NULL || len == 0) {
        return;
    }
    uint8_t reg = src[0];
    d->ptr = reg;
    d->ptr_valid = true;
    for (size_t i = 1; i + 1 < len; i += 2) {
        d->reg[reg] = (uint16_t)(src[i] | ((uint16_t)src[i + 1] << 8));
        reg++;
        d->ptr = reg; /* the pointer advances past what was written, as on hardware */
    }
}

/** Fill `dst` from the register map starting at `reg`, little-endian. */
static void apply_read(i2c_fake_device_t *d, uint8_t reg, uint8_t *dst, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint16_t v = d != NULL ? d->reg[(uint8_t)(reg + i / 2)] : i2c_fake.read_fill;
        dst[i] = (i % 2 == 0) ? (uint8_t)(v & 0xFF) : (uint8_t)(v >> 8);
    }
    if (d != NULL) {
        d->ptr = (uint8_t)(reg + (len + 1) / 2);
        d->ptr_valid = true;
    }
}

/** A spec that could not have come from I2C_DT_SPEC_GET() is a test bug, not a bus error. */
static int check_spec(const struct i2c_dt_spec *spec) {
    if (spec == NULL || spec->bus == NULL) {
        return -EINVAL;
    }
    return 0;
}

/* ======================================================================= */
/* zephyr/drivers/i2c.h                                                     */
/* ======================================================================= */

bool i2c_is_ready_dt(const struct i2c_dt_spec *spec) {
    i2c_fake.is_ready_calls++;
    return spec != NULL && spec->bus == &fake_bus && i2c_fake.bus_ready;
}

int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, uint32_t num_bytes) {
    int status = check_spec(spec);
    if (status != 0 || buf == NULL || num_bytes == 0) {
        return status != 0 ? status : -EINVAL;
    }

    i2c_fake_device_t *d = find_device((uint8_t)spec->addr);
    /* A bare read takes whatever the pointer was left at by the last write. */
    uint8_t reg = (d != NULL) ? d->ptr : 0;
    bool reg_valid = (d != NULL) && d->ptr_valid;

    status = xfer_status((uint8_t)spec->addr, reg, reg_valid);
    if (status == 0) {
        apply_read(d, reg, buf, num_bytes);
    }

    i2c_fake.read_calls++;
    record(i2c_fake_op_read, spec, reg, reg_valid, NULL, 0, status == 0 ? buf : NULL, num_bytes,
           status);
    return status;
}

int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf, uint32_t num_bytes) {
    int status = check_spec(spec);
    if (status != 0 || buf == NULL || num_bytes == 0) {
        return status != 0 ? status : -EINVAL;
    }

    status = xfer_status((uint8_t)spec->addr, buf[0], true);
    if (status == 0) {
        apply_write(find_device((uint8_t)spec->addr), buf, num_bytes);
    }

    i2c_fake.write_calls++;
    record(i2c_fake_op_write, spec, buf[0], true, buf, num_bytes, NULL, 0, status);
    return status;
}

int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *write_buf, size_t num_write,
                      void *read_buf, size_t num_read) {
    const uint8_t *src = write_buf;
    uint8_t *dst = read_buf;

    int status = check_spec(spec);
    if (status != 0 || src == NULL || dst == NULL || num_write == 0 || num_read == 0) {
        return status != 0 ? status : -EINVAL;
    }

    uint8_t reg = src[0];
    status = xfer_status((uint8_t)spec->addr, reg, true);
    if (status == 0) {
        i2c_fake_device_t *d = find_device((uint8_t)spec->addr);
        apply_write(d, src, num_write);
        /* A repeated START reads back from the register the write named, not
         * from wherever the write left the pointer. */
        apply_read(d, reg, dst, num_read);
    }

    i2c_fake.write_read_calls++;
    record(i2c_fake_op_write_read, spec, reg, true, src, num_write, status == 0 ? dst : NULL,
           num_read, status);
    return status;
}

int i2c_burst_read_dt(const struct i2c_dt_spec *spec, uint8_t start_addr, uint8_t *buf,
                      uint32_t num_bytes) {
    return i2c_write_read_dt(spec, &start_addr, 1, buf, num_bytes);
}

int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t start_addr, const uint8_t *buf,
                       uint32_t num_bytes) {
    /* On the wire this is the register byte followed by the data, which is
     * exactly what a single write of [reg, data...] models. */
    uint8_t tmp[1 + 2 * I2C_FAKE_REGS];
    if (buf == NULL || num_bytes == 0 || num_bytes > sizeof(tmp) - 1) {
        return -EINVAL;
    }
    tmp[0] = start_addr;
    memcpy(&tmp[1], buf, num_bytes);
    return i2c_write_dt(spec, tmp, num_bytes + 1);
}
