/**
 * @brief Fake MAX17205 for tests: two register files the driver reaches over the emulated I2C bus.
 */

#ifndef MAX17205_EMUL_H_
#define MAX17205_EMUL_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/emul.h>

/* Main register block, I2C address 0x36 (the driver keeps its copy private) */
#define MAX17205_EMUL_REG_REPCAP  0x05
#define MAX17205_EMUL_REG_REPSOC  0x06
#define MAX17205_EMUL_REG_TEMP    0x08
#define MAX17205_EMUL_REG_VCELL   0x09
#define MAX17205_EMUL_REG_CURRENT 0x0A
#define MAX17205_EMUL_REG_TTE     0x11
#define MAX17205_EMUL_REG_CYCLES  0x17
#define MAX17205_EMUL_REG_TTF     0x20
#define MAX17205_EMUL_REG_CONFIG2 0xBB
#define MAX17205_EMUL_REG_TIMERH  0xBE
#define MAX17205_EMUL_REG_VBAT    0xDA

/* Shadow RAM, I2C address 0x0B */
#define MAX17205_EMUL_REG_TEMP1   0x34
#define MAX17205_EMUL_REG_INTTEMP 0x35
#define MAX17205_EMUL_REG_TEMP2   0x3B

/* Which register file: MAX17205_MAIN_I2C_ADDR or MAX17205_SHADOW_I2C_ADDR */
void max17205_emul_set_reg(const struct emul *target, uint16_t addr, uint8_t reg, uint16_t val);
uint16_t max17205_emul_get_reg(const struct emul *target, uint16_t addr, uint8_t reg);

/* Fail every transfer with -EIO, as if the gauge stopped answering */
void max17205_emul_set_nack(const struct emul *target, bool nack);

/* Zero both register files and clear the NACK */
void max17205_emul_reset(const struct emul *target);

#endif /* MAX17205_EMUL_H_ */
