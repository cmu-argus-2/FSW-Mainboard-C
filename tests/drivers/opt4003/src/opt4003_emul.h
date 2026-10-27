/**
 * @brief Fake OPT4003 for tests: a register file the driver reaches over the emulated I2C bus.
 */

#ifndef OPT4003_EMUL_H_
#define OPT4003_EMUL_H_

#include <stdint.h>
#include <zephyr/drivers/emul.h>

/* Register map, as described on page 25 of the datasheet (the driver keeps its copy private) */
#define OPT4003_EMUL_REG_RESULT_MSB_CH0 0x00
#define OPT4003_EMUL_REG_RESULT_LSB_CH0 0x01
#define OPT4003_EMUL_REG_CONFIGURATION  0x0A
#define OPT4003_EMUL_REG_FLAGS          0x0C
#define OPT4003_EMUL_REG_DEVICE_ID      0x11
#define OPT4003_EMUL_NUM_REGS           0x12

/* What DEVICE_ID holds after reset: DIDL = 0, DIDH = 0x221 */
#define OPT4003_EMUL_DEVICE_ID 0x0221

void opt4003_emul_set_reg(const struct emul *target, uint8_t reg, uint16_t val);
uint16_t opt4003_emul_get_reg(const struct emul *target, uint8_t reg);

#endif /* OPT4003_EMUL_H_ */
