/**
 * @file i2c_fake.h
 * @brief Fake of Zephyr's I2C API (fakes/zephyr/drivers/i2c.h), for testing drivers on top of it.
 *
 * max17205.c's dependency is <zephyr/drivers/i2c.h>, so that is where these
 * tests cut: the driver is compiled for the host against this fake instead of
 * a real Zephyr bus driver.
 *
 * The fake does two things at once:
 *
 *   1. Records every call (i2c_fake.xfer[], plus per-entry-point counters), so
 *      a test can assert on the exact bytes the driver put on the wire.
 *   2. Models a register-map device: 256 16-bit registers per I2C address,
 *      with the usual "write the register pointer, then read" protocol and
 *      little-endian byte order, which is what the MAX17205 speaks. So a test
 *      can just say "register 0x06 reads back 0x8000" and call the driver.
 *
 * Typical use:
 *
 *     void setUp(void) {
 *         i2c_fake_reset();
 *         i2c_fake_set_present(0x36, true);
 *     }
 *
 *     void test_read_soc(void) {
 *         i2c_fake_set_reg(0x36, 0x06, 0x8000);    // 50.0% in 1/256 % units
 *         ...
 *         TEST_ASSERT_EQUAL_INT(0, i2c_fake_find_reg_read(0x36, 0x06) < 0);
 *     }
 *
 * Status codes are Zephyr's: 0, or a negative errno. A transfer to an address
 * nobody answers at returns -EIO, as the real controller drivers do on a NACK.
 */

#ifndef I2C_FAKE_H
#define I2C_FAKE_H

#include <zephyr/drivers/i2c.h>

/** Recorded transfers kept before the log stops growing (counters keep going). */
#define I2C_FAKE_MAX_EVENTS 64
/** Bytes retained per direction per transfer. Longer transfers are truncated
 *  in the log only -- the register model still sees all of them. */
#define I2C_FAKE_MAX_BYTES 16
/** Distinct I2C addresses that can hold a register map at once. The MAX17205
 *  needs two (main + shadow RAM). */
#define I2C_FAKE_MAX_DEVICES 4
/** Registers per device. The MAX17205 register space is 8-bit addressed. */
#define I2C_FAKE_REGS 256

typedef enum {
    i2c_fake_op_write = 0,
    i2c_fake_op_read,
    i2c_fake_op_write_read,
} i2c_fake_op_t;

/** One transfer (any of the i2c_*_dt() calls). */
typedef struct {
    i2c_fake_op_t        op;
    const struct device *bus;
    uint8_t              addr;      /* 7-bit address of the target device */
    uint8_t              reg;       /* register pointer this transfer acted on */
    bool                 reg_valid; /* false for a bare read with no pointer ever set */
    uint8_t              w[I2C_FAKE_MAX_BYTES];
    size_t               w_len;     /* length the caller asked for, even if truncated above */
    uint8_t              r[I2C_FAKE_MAX_BYTES];
    size_t               r_len;
    int                  ret;       /* what the fake returned */
} i2c_fake_xfer_t;

/** A register-map device sitting at one address. */
typedef struct {
    bool     used;
    uint8_t  addr;
    uint16_t reg[I2C_FAKE_REGS];
    int      reg_status[I2C_FAKE_REGS]; /* 0 unless a failure was injected */
    uint8_t  ptr;                       /* register pointer, as last written */
    bool     ptr_valid;
} i2c_fake_device_t;

typedef struct {
    /** What i2c_is_ready_dt() reports for the fake bus. True after reset. */
    bool     bus_ready;
    unsigned is_ready_calls;

    /* --- transfers --- */
    unsigned        xfer_calls;    /* total, including any beyond the log */
    unsigned        read_calls;
    unsigned        write_calls;
    unsigned        write_read_calls;
    i2c_fake_xfer_t xfer[I2C_FAKE_MAX_EVENTS];

    /* --- devices --- */
    i2c_fake_device_t dev[I2C_FAKE_MAX_DEVICES];

    /* --- failure injection (see the helpers below) --- */
    int      forced;          /* 0 = off; anything else fails every transfer */
    bool     fail_after_armed;
    unsigned fail_after_remaining;
    int      fail_after_status;

    /** Byte returned for a read of a register on a device with no map entry.
     *  Registers all start at 0, so this only shows up past the 256-register
     *  space; it exists to make such a read visibly wrong rather than zero. */
    uint8_t read_fill;
} i2c_fake_t;

extern i2c_fake_t i2c_fake;

/** Clear every recorded call, device and injected failure. Call from setUp(). */
void i2c_fake_reset(void);

/** The fake I2C controller, i.e. what DEVICE_DT_GET(DT_NODELABEL(i2c1)) is on target. */
const struct device *i2c_fake_bus(void);

/** An i2c_dt_spec on the fake bus at `addr`, as I2C_DT_SPEC_GET() would give. */
struct i2c_dt_spec i2c_fake_spec(uint8_t addr);

/** Put a register-map device on the bus at `addr` (or take it off again).
 *  A transfer to an absent address returns -EIO. */
void i2c_fake_set_present(uint8_t addr, bool present);

/** Set/get one 16-bit register. Setting implies the device is present. */
void     i2c_fake_set_reg(uint8_t addr, uint8_t reg, uint16_t value);
uint16_t i2c_fake_get_reg(uint8_t addr, uint8_t reg);

/** Every transfer returns `err` (a negative errno). Pass 0 to turn it back off. */
void i2c_fake_fail_all(int err);

/** The next `n_ok` transfers succeed; every one after that returns `err`.
 *  Useful for "read_all gives up partway through" cases. */
void i2c_fake_fail_after(unsigned n_ok, int err);

/** Only accesses to this one register fail, with `err`. 0 clears it. */
void i2c_fake_fail_reg(uint8_t addr, uint8_t reg, int err);

/** Most recent recorded transfer, or NULL if there were none. */
const i2c_fake_xfer_t *i2c_fake_last_xfer(void);

/** Index in i2c_fake.xfer[] of the first transfer that read `reg` from `addr`,
 *  or -1. A read spanning several registers matches on any it covered. */
int i2c_fake_find_reg_read(uint8_t addr, uint8_t reg);

/** Index of the first transfer that wrote `value` to `reg` on `addr`, or -1. */
int i2c_fake_find_reg_write(uint8_t addr, uint8_t reg, uint16_t value);

#endif /* I2C_FAKE_H */
