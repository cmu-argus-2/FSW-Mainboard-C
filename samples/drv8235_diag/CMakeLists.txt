# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${ARGUS_ROOT}/cmake/drv8235_board_lib.cmake)
set(BOARD_DIRECTORIES "${ARGUS_BOARD_LIB}/boards/argus")
list(APPEND ZEPHYR_EXTRA_MODULES "${ARGUS_BOARD_LIB}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drv8235_diag)

target_sources(app PRIVATE
        src/main.c
        ../../src/drivers/drv8235.c
)

target_include_directories(app PRIVATE
        ${ARGUS_ROOT}/include
)
