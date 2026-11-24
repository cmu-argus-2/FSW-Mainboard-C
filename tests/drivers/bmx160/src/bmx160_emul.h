/**
 * @brief Fake BMX160 for tests: register files for the BMX160 and the BMM150 behind it.
 */

#ifndef BMX160_EMUL_H_
#define BMX160_EMUL_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/emul.h>

/* Register map (the driver keeps its copy private) */
#define BMX160_EMUL_REG_CHIP_ID    0x00
#define BMX160_EMUL_REG_ERR        0x02
#define BMX160_EMUL_REG_PMU_STATUS 0x03
#define BMX160_EMUL_REG_DATA_MAG   0x04
#define BMX160_EMUL_REG_DATA_GYR   0x0C
#define BMX160_EMUL_REG_DATA_ACC   0x12
#define BMX160_EMUL_REG_TEMP       0x20
#define BMX160_EMUL_REG_ACC_CONF   0x40
#define BMX160_EMUL_REG_ACC_RANGE  0x41
#define BMX160_EMUL_REG_GYR_CONF   0x42
#define BMX160_EMUL_REG_GYR_RANGE  0x43
#define BMX160_EMUL_REG_MAG_CONF   0x44
#define BMX160_EMUL_REG_MAG_IF1    0x4C
#define BMX160_EMUL_REG_MAG_IF2    0x4D
#define BMX160_EMUL_REG_CMD        0x7E
#define BMX160_EMUL_NUM_REGS       0x80

#define BMX160_EMUL_CHIP_ID 0xD8
#define BMX160_EMUL_NO_REG  0xFF

/* BMM150 trim the fake reports, as the driver should decode it */
#define BMX160_EMUL_TRIM_X1   (-3)
#define BMX160_EMUL_TRIM_Y1   5
#define BMX160_EMUL_TRIM_Z4   (-200)
#define BMX160_EMUL_TRIM_X2   (-4)
#define BMX160_EMUL_TRIM_Y2   26
#define BMX160_EMUL_TRIM_Z2   763
#define BMX160_EMUL_TRIM_Z1   24747
#define BMX160_EMUL_TRIM_XYZ1 6696 /* stored with bit 15 set, which the driver must mask */
#define BMX160_EMUL_TRIM_Z3   (-100)
#define BMX160_EMUL_TRIM_XY2  (-3)
#define BMX160_EMUL_TRIM_XY1  29

void bmx160_emul_set_reg(const struct emul *target, uint8_t reg, uint8_t val);
uint8_t bmx160_emul_get_reg(const struct emul *target, uint8_t reg);
/* Little-endian 16-bit value at reg, reg + 1 */
void bmx160_emul_set_reg16(const struct emul *target, uint8_t reg, int16_t val);

void bmx160_emul_set_bmm150_reg(const struct emul *target, uint8_t reg, uint8_t val);
uint8_t bmx160_emul_get_bmm150_reg(const struct emul *target, uint8_t reg);

/* CHIP_ID after the next soft reset */
void bmx160_emul_set_chip_id(const struct emul *target, uint8_t id);
/* ERR_REG reads this instead of the register, when non-zero */
void bmx160_emul_force_err(const struct emul *target, uint8_t err);
/* Writes to reg are dropped, so its read-back differs; BMX160_EMUL_NO_REG for none */
void bmx160_emul_ignore_writes(const struct emul *target, uint8_t reg);
/* Every transfer fails with -EIO */
void bmx160_emul_set_fail(const struct emul *target, bool fail);

/* CMD writes since boot, oldest first, up to 32 */
size_t bmx160_emul_get_cmds(const struct emul *target, const uint8_t **cmds);

#endif /* BMX160_EMUL_H_ */
