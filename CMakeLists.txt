cmake_minimum_required(VERSION 3.28)

project(mininav LANGUAGES CXX)

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
        src/apps/sim_v0_main.cpp
    PRIVATE
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
            src/core/types.ixx
            src/core/math.ixx
            src/core/robot_model.ixx
)

target_compile_features(sim_v0 PRIVATE cxx_std_23)

set_property(TARGET sim_v0 PROPERTY CXX_SCAN_FOR_MODULES ON)

mininav_set_warnings(sim_v0)