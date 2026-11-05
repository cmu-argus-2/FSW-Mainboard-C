/**
 * @authors Ailisi Bao, John Buttles, Jacob Rohozen
 * @file max17205.h
 * 
 * @brief This file defines the interface for the MAX17205 fuel gauge on the Argus mainboard.
 * This talks to the gauge through Zephyr's I2C API; the bus and address come from the
 * `fuel_gauge` devicetree node (boards/cmu/argus/argus.dtsi).
 *
 */

#ifndef MAX17205
#define MAX17205

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/fuel_gauge.h>

/* Shadow RAM (thermistor/die temperatures) answers at a second, fixed address
 * on the same bus as the main register block. */
#define MAX17205_MAIN_I2C_ADDR 0x36
#define MAX17205_SHADOW_I2C_ADDR 0x0B

enum max17205_fuel_gauge_prop {
      MAX17205_PROP_TEMP_AIN1_DK = FUEL_GAUGE_CUSTOM_BEGIN,
      MAX17205_PROP_TEMP_AIN2_DK,
      MAX17205_PROP_TEMP_DIE_DK,
      MAX17205_PROP_MID_VOLTAGE_UV,
      MAX17205_PROP_TIME_PWRUP_S,
};

/* Reset */
int max17205_reset(const struct device *dev);

#endif
