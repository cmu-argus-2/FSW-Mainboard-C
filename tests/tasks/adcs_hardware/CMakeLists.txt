# SPDX-License-Identifier: Apache-2.0
# The ADCS task's BMX160 callbacks (src/tasks/adcs_hardware.c), with the real driver
# against the fake IMU from tests/drivers/bmx160.
cmake_minimum_required(VERSION 3.20.0)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
list(APPEND DTS_ROOT "${ARGUS_ROOT}")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adcs_hardware_test)
target_sources(app PRIVATE
    src/main.c
    ${ARGUS_ROOT}/src/tasks/adcs_hardware.c
    ${ARGUS_ROOT}/src/tasks/adcs_runner.c
    ${ARGUS_ROOT}/src/tasks/adcs.c
    ${ARGUS_ROOT}/src/apps/adcs/adcs.c
    ${ARGUS_ROOT}/src/drivers/bmx160.c
    ${ARGUS_ROOT}/tests/drivers/bmx160/src/bmx160_emul.c
)
target_include_directories(app PRIVATE
    ${ARGUS_ROOT}/include
    ${ARGUS_ROOT}/tests/drivers/bmx160/src
)
