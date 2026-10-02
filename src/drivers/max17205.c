/**
 * @authors Ailisi Bao, John Buttles, Jacob Rohozen
 * @file max17205.c
 * 
 * @brief This file defines the implementation for reading certain registers of the MAX17205 fuel gauge 
 * on the Argus mainboard, over Zephyr's I2C API (see include/argus/drivers/max17205.h).
 * 
 */

#define DT_DRV_COMPAT maxim_max17205

#include "argus/drivers/max17205.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(max17205, CONFIG_FUEL_GAUGE_LOG_LEVEL); 

/* Defining Internal Addresses for Reads on the MAX17205 */
/* Main block (0x36) */
#define MAX17205_VCELL_ADDR 0x09  // Lowest cell voltage of a pack
#define MAX17205_REPSOC_ADDR 0x06  // Reported state of charge
#define MAX17205_REPCAP_ADDR 0x05  // Reported remaining capacity
#define MAX17205_CURRENT_ADDR 0x0A  // Battery current
#define MAX17205_TTE_ADDR 0x11  // Time to empty
#define MAX17205_TTF_ADDR 0x20  // Time to full
#define MAX17205_CAPACITY_ADDR 0x10  // Full capacity estimation
#define MAX17205_VBAT_ADDR 0xDA  // Battery pack voltage
#define MAX17205_CYCLES_ADDR 0x17  // Battery cycles
#define MAX17205_TIMERH_ADDR 0xBE  // Time since power up
#define MAX17205_TEMP_ADDR 0x08  // Temp register

#define MAX17205_COMMAND_ADDR 0x60  // Command register
#define MAX17205_CONFIG2_ADDR 0xBB  // Config2 register (bit 0 = POR_CMD, fuel gauge reset)

/* Addresses in shadow RAM (I2C address 0x0B). These are SBS registers: 0.1 K per LSB. */
#define MAX17205_TEMP1_ADDR 0x34  // AIN1 thermistor temperature
#define MAX17205_TEMP2_ADDR 0x3B  // AIN2 thermistor temperature
#define MAX17205_INTTEMP_ADDR 0x35  // Internal die temperature

struct max17205_config {
    struct i2c_dt_spec main;
    struct i2c_dt_spec shadow;
    uint32_t sense_ohm;
};

/* Init/Deinit */
static int max17205_init(const struct device *dev) {
      const struct max17205_config *cfg = dev->config;

      if (!i2c_is_ready_dt(&cfg->main)) {
              LOG_ERR("I2C bus not ready");
              return -ENODEV;
      }
      /* Keep this tolerant: don't fail boot just because the gauge didn't ACK */
      return 0;
}

/* Helper fxn for reading values from I2C bus */
static int max17205_read(const struct i2c_dt_spec *spec, uint8_t reg, uint16_t *out) {
      uint8_t buf[2];
      int rc = i2c_write_read_dt(spec, &reg, 1, buf, sizeof(buf));

      if (rc) {
        return rc;
      }
      *out = sys_get_le16(buf);  /* MAX17205 registers are little-endian */
      return 0;
}

/* Callback fxn for Fuel gauge driver */
static int max17205_get_prop(const struct device *dev, fuel_gauge_prop_t prop, union fuel_gauge_prop_val *val) {
    const struct max17205_config *cfg = dev->config;
      uint16_t raw;
      int rc;

      switch (prop) {
        case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT:
            rc = max17205_read(&cfg->main, MAX17205_REPSOC_ADDR, &raw);
            if (rc == 0) {
                val->relative_state_of_charge_pct = (uint8_t)(raw / 256);
            }
            return rc;
        case FUEL_GAUGE_REMAINING_CAPACITY_UAH:
            rc = max17205_read(&cfg->main, MAX17205_REPCAP_ADDR, &raw);
            if (rc == 0) {
                val->remaining_capacity_uah = (uint32_t)((uint64_t)raw * 5000000 / cfg->sense_ohm);  /* 5 uVh / Rsense */
            }
            return rc;
        case FUEL_GAUGE_CURRENT_UA:
            rc = max17205_read(&cfg->main, MAX17205_CURRENT_ADDR, &raw);
            if (rc == 0) {
                val->current_ua = (int32_t)((int64_t)(int16_t)raw * 1562500 / cfg->sense_ohm);
            }
            return rc;
        case FUEL_GAUGE_VOLTAGE_UV:
            rc = max17205_read(&cfg->main, MAX17205_VBAT_ADDR, &raw);
            if (rc == 0) {
                val->voltage_uv = (int32_t)raw * 1250;
            }
            return rc;
        case MAX17205_PROP_MID_VOLTAGE_UV:
            rc = max17205_read(&cfg->main, MAX17205_VCELL_ADDR, &raw);
            if (rc == 0) {
                val->custom_uint = (uint32_t)raw * 625 / 8;  /* 78.125 uV */
            }
            return rc;
        case FUEL_GAUGE_CYCLE_COUNT:
            rc = max17205_read(&cfg->main, MAX17205_CYCLES_ADDR, &raw);
            if (rc == 0) {
                val->cycle_count = (uint32_t)raw;
            }
            return rc;
        case FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS:
            rc = max17205_read(&cfg->main, MAX17205_TTE_ADDR, &raw);
            if (rc == 0) {
                val->runtime_to_empty_mins = (uint32_t)raw * 3 / 32;  /* 5.625 s */
            }
            return rc;
        case FUEL_GAUGE_RUNTIME_TO_FULL_MINS:
            rc = max17205_read(&cfg->main, MAX17205_TTF_ADDR, &raw);
            if (rc == 0) {
                val->runtime_to_full_mins = (uint32_t)raw * 3 / 32;  /* 5.625 s */
            }
            return rc;
        case MAX17205_PROP_TIME_PWRUP_S:
            rc = max17205_read(&cfg->main, MAX17205_TIMERH_ADDR, &raw);
            if (rc == 0) {
                val->custom_uint = (uint32_t)raw;
            }
            return rc;
        case FUEL_GAUGE_TEMPERATURE_DK:
            rc = max17205_read(&cfg->main, MAX17205_TEMP_ADDR, &raw);
            if (rc == 0) {
                val->temperature_dk = (uint16_t)((int16_t)raw * 10 / 256 + 2732);
            }
            return rc;
        case MAX17205_PROP_TEMP_AIN1_DK:
            rc = max17205_read(&cfg->shadow, MAX17205_TEMP1_ADDR, &raw);
            if (rc == 0) {
                val->custom_int = raw;  /* already 0.1 K */
            }
            return rc;
        case MAX17205_PROP_TEMP_AIN2_DK:
            rc = max17205_read(&cfg->shadow, MAX17205_TEMP2_ADDR, &raw);
            if (rc == 0) {
                val->custom_int = raw;  /* already 0.1 K */
            }
            return rc;
        case MAX17205_PROP_TEMP_DIE_DK:
            rc = max17205_read(&cfg->shadow, MAX17205_INTTEMP_ADDR, &raw);
            if (rc == 0) {
                val->custom_int = raw;  /* already 0.1 K */
            }
            return rc;
        default:
              return -ENOTSUP;
      }
}

/* Reset */
int max17205_reset(const struct device *dev) {
    const struct max17205_config *cfg = dev->config;
    uint8_t buf[3] = {MAX17205_CONFIG2_ADDR, 0x01, 0x00};

    return i2c_write_dt(&cfg->main, buf, sizeof(buf));
}

static DEVICE_API(fuel_gauge, max17205_api) = {
      .get_property = max17205_get_prop,
};

#define MAX17205_DEFINE(inst)                                                   \
      static const struct max17205_config max17205_config_##inst = {         \
              .main = I2C_DT_SPEC_INST_GET(inst),                             \
              .shadow = { .bus = DEVICE_DT_GET(DT_INST_BUS(inst)),            \
                          .addr = MAX17205_SHADOW_I2C_ADDR },                 \
              .sense_ohm = DT_INST_PROP(inst, rsense_micro_ohms),             \
      };                                                                      \
      DEVICE_DT_INST_DEFINE(inst, max17205_init, NULL, NULL,                  \
                            &max17205_config_##inst, POST_KERNEL,             \
                            CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17205_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_DEFINE)
