# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adcs_tests)
target_sources(app PRIVATE
    src/main.c
    ../../../src/apps/adcs/adcs.c
    ../../../src/tasks/adcs.c
    ../../../src/tasks/adcs_runner.c
)
target_include_directories(app PRIVATE ../../../include)
