# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drv8235_diag)

target_sources(app PRIVATE
        src/main.c
        ../../src/drivers/drv8235.c
        ../../src/drivers/drv8235_common.c
)

target_include_directories(app PRIVATE
        ${ARGUS_ROOT}/include
)
