cmake_minimum_required(VERSION 3.20.0)

# This sample is its own Zephyr app two levels below the repo root, so point it
# at the repo's devicetree bindings (dts/bindings/) and the driver.
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND DTS_ROOT ${ARGUS_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(max17205_demo)

target_sources(app PRIVATE
        src/main.c
        ${ARGUS_ROOT}/src/drivers/max17205.c
)

target_include_directories(app PRIVATE
        ${ARGUS_ROOT}/include
)
