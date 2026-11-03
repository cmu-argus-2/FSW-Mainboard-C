cmake_minimum_required(VERSION 3.20.0)

# This test is its own Zephyr app three levels below the repo root, so point it
# at the repo's devicetree bindings (dts/bindings/) and the driver under test.
set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
list(APPEND DTS_ROOT ${REPO_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(opt4003_test)

target_sources(app PRIVATE
        src/main.c
        src/opt4003_emul.c
        ${REPO_ROOT}/src/drivers/opt4003.c
)

target_include_directories(app PRIVATE
        ${REPO_ROOT}/include
)
