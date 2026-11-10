# SPDX-License-Identifier: Apache-2.0
# Share the real hardware binding without globally registering another argus
# board in west/Twister. Only the DRV8235 test and diagnostic include this file.
# A local checkout can be supplied for offline builds.
set(ARGUS_BOARD_LIB "" CACHE PATH "Local Board Library checkout (optional)")
if(NOT ARGUS_BOARD_LIB)
    include(FetchContent)
    FetchContent_Declare(argus_drv8235_board_lib
        GIT_REPOSITORY https://github.com/cmu-argus-2/FSW-Zephyr-Board-Lib.git
        GIT_TAG 4483fe67a5bf866f25d9d3c5868d3a2e10f0aeaa
        GIT_PROGRESS FALSE
    )
    FetchContent_MakeAvailable(argus_drv8235_board_lib)
    set(ARGUS_BOARD_LIB "${argus_drv8235_board_lib_SOURCE_DIR}")
endif()
if(NOT EXISTS "${ARGUS_BOARD_LIB}/dts/bindings/motor/ti,drv8235.yaml")
    message(FATAL_ERROR "ARGUS_BOARD_LIB must contain the DRV8235 binding from Board Lib PR #2")
endif()
