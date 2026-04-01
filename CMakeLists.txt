cmake_minimum_required(VERSION 3.28)

project(MiniNav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)

add_executable(sim_v0)

target_sources(sim_v0
    PRIVATE
        src/core/robot_model.cpp
        src/core/csv_writer.cpp
        src/core/staged_command_source.cpp
        src/apps/sim_v0_main.cpp
    PRIVATE
        FILE_SET CXX_MODULES FILES
            src/core/types.ixx
            src/core/math.ixx
            src/core/robot_model.ixx
            src/core/trajectory.ixx
            src/core/csv_writer.ixx
            src/core/logger.ixx
            src/core/command_source.ixx
)

set_property(TARGET sim_v0 PROPERTY CXX_SCAN_FOR_MODULES ON)
mininav_set_warnings(sim_v0)

add_executable(robot_model_tests)

target_sources(robot_model_tests
    PRIVATE
        src/core/robot_model.cpp
        tests/robot_model_tests.cpp
    PRIVATE
        FILE_SET CXX_MODULES FILES
            src/core/types.ixx
            src/core/math.ixx
            src/core/robot_model.ixx
)

set_property(TARGET robot_model_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
mininav_set_warnings(robot_model_tests)

enable_testing()
add_test(NAME robot_model_tests COMMAND robot_model_tests)