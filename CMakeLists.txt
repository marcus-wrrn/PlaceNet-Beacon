# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "t_beam")
endif()

message(STATUS "Building for board: ${BOARD_TYPE}")

# Use the variable to select different configs
if(BOARD_TYPE STREQUAL "t_beam")
    # set(SDKCONFIG_DEFAULTS "sdkconfig.defaults.t_beam")
    add_compile_definitions(BOARD_T_BEAM=1)
elseif(BOARD_TYPE STREQUAL "t_deck")
    # set(SDKCONFIG_DEFAULTS "sdkconfig.defaults.t_deck")
    add_compile_definitions(BOARD_T_DECK=1)
else()
    message(FATAL_ERROR "Unknown BOARD_TYPE: ${BOARD_TYPE}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PlaceNet-Beacon)