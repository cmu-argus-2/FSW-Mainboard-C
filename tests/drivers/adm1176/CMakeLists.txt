# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
list(APPEND DTS_ROOT ${ARGUS_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adm1176_tests)

target_sources(app PRIVATE
        src/main.c
        src/adm1176_emul.c
        src/driver_api.c
        ../../../src/drivers/adm1176.c
        ../../../src/drivers/adm1176_common.c
)

target_include_directories(app PRIVATE
        ${ARGUS_ROOT}/include
)
