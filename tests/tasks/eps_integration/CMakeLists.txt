# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
list(APPEND DTS_ROOT ${ROOT})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(eps_integration)
target_sources(app PRIVATE
    src/main.c
    ${ROOT}/src/tasks/eps.c ${ROOT}/src/tasks/eps_zephyr.c
    ${ROOT}/src/drivers/adm1176.c ${ROOT}/src/drivers/adm1176_common.c
    ${ROOT}/src/drivers/drv8235.c ${ROOT}/src/drivers/drv8235_common.c
    ${ROOT}/src/drivers/max17205.c
    ${ROOT}/tests/drivers/adm1176/src/adm1176_emul.c
    ${ROOT}/tests/drivers/drv8235/src/drv8235_emul.c
    ${ROOT}/tests/drivers/max17205/src/max17205_emul.c)
target_include_directories(app PRIVATE ${ROOT}/include
    ${ROOT}/tests/drivers/adm1176/src ${ROOT}/tests/drivers/drv8235/src
    ${ROOT}/tests/drivers/max17205/src)
