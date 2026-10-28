cmake_minimum_required(VERSION 3.20.0)

# Default target (current default hardware revision is set in boards/cmu/argus/board.yml).
# Pick a revision explicitly with e.g. `west build -b argus@4/rp2350b/m33_0`.
set(BOARD argus/rp2350b/m33_0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(FSW-Mainboard-C)

target_sources(app PRIVATE
        src/main.c
        src/drivers/max17205.c
)

target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Generate argus_version.h (git hash + dirty flag) on every build, so the
# version printed at boot is never stale.
set(ARGUS_VERSION_DIR ${CMAKE_BINARY_DIR}/include/generated/argus)
add_custom_target(argus_version_h
        COMMAND ${CMAKE_COMMAND}
                -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -DOUT_FILE=${ARGUS_VERSION_DIR}/argus_version.h
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_version.cmake
        BYPRODUCTS ${ARGUS_VERSION_DIR}/argus_version.h
)
add_dependencies(app argus_version_h)
target_include_directories(app PRIVATE ${ARGUS_VERSION_DIR})
