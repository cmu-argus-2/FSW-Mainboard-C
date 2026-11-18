# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
list(APPEND DTS_ROOT ${ARGUS_ROOT})
include(${ARGUS_ROOT}/cmake/drv8235_board_lib.cmake)
# Import only the coil binding. The board library also has a MAX17205
# binding, which would conflict with the application's copy.
set(EPS_TEST_DTS_ROOT "${CMAKE_CURRENT_BINARY_DIR}/board-lib-binding")
configure_file(
    "${ARGUS_BOARD_LIB}/dts/bindings/motor/ti,drv8235.yaml"
    "${EPS_TEST_DTS_ROOT}/dts/bindings/motor/ti,drv8235.yaml"
    COPYONLY
)
list(APPEND DTS_ROOT ${EPS_TEST_DTS_ROOT})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(eps_hardware_test)
target_sources(app PRIVATE
    src/main.c
    ${ARGUS_ROOT}/src/tasks/eps_hardware.c
    ${ARGUS_ROOT}/src/apps/eps/eps.c
    ${ARGUS_ROOT}/src/tasks/eps.c
    ${ARGUS_ROOT}/src/tasks/eps_runner.c
    ${ARGUS_ROOT}/src/drivers/adm1176.c
    ${ARGUS_ROOT}/src/drivers/drv8235.c
    ${ARGUS_ROOT}/src/drivers/max17205.c
    ${ARGUS_ROOT}/tests/drivers/adm1176/src/adm1176_emul.c
    ${ARGUS_ROOT}/tests/drivers/drv8235/src/drv8235_emul.c
    ${ARGUS_ROOT}/tests/drivers/max17205/src/max17205_emul.c
)
target_include_directories(app PRIVATE
    ${ARGUS_ROOT}/include
    ${ARGUS_ROOT}/tests/drivers/adm1176/src
    ${ARGUS_ROOT}/tests/drivers/drv8235/src
    ${ARGUS_ROOT}/tests/drivers/max17205/src
)
