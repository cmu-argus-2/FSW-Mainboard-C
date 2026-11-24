cmake_minimum_required(VERSION 3.20.0)

# This sample is its own Zephyr app two levels below the repo root, so point it at the
# repo's board (boards/) and devicetree bindings (dts/bindings/). No drivers are built:
# the scan talks to the I2C controllers directly.
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND BOARD_ROOT ${ARGUS_ROOT})
list(APPEND DTS_ROOT ${ARGUS_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_scan)

target_sources(app PRIVATE src/main.c)
