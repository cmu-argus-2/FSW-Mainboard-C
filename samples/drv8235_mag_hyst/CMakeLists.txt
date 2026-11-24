# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Same board as samples/drv8235_diag: the Board Library's argus, which has the six
# DRV8235 nodes and coil_power.
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${ARGUS_ROOT}/cmake/drv8235_board_lib.cmake)
set(BOARD_DIRECTORIES "${ARGUS_BOARD_LIB}/boards/argus")
list(APPEND ZEPHYR_EXTRA_MODULES "${ARGUS_BOARD_LIB}")

# Add only the bosch,bmx160 binding: the repo root would define maxim,max17205 a second
# time next to the Board Library's copy, which devicetree rejects.
set(BMX160_DTS_ROOT ${CMAKE_CURRENT_BINARY_DIR}/bmx160_dts_root)
file(COPY ${ARGUS_ROOT}/dts/bindings/sensor/bosch,bmx160.yaml
     DESTINATION ${BMX160_DTS_ROOT}/dts/bindings/sensor)
list(APPEND DTS_ROOT ${BMX160_DTS_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drv8235_mag_hyst)

target_sources(app PRIVATE
        src/main.c
        ${ARGUS_ROOT}/src/drivers/drv8235.c
        ${ARGUS_ROOT}/src/drivers/bmx160.c
)

target_include_directories(app PRIVATE
        ${ARGUS_ROOT}/include
)
