cmake_minimum_required(VERSION 3.28)

project(MiniNav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)

# ------------------------------------------------------------------
# 第三方依赖
# ------------------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include(rerun)
include(cli11)
include(yaml_cpp)
include(spdlog)

# ------------------------------------------------------------------
# 子模块库
# ------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/viz)
add_subdirectory(src/sensors)
add_subdirectory(src/simulation)
add_subdirectory(src/localization)
add_subdirectory(src/planning)

# ------------------------------------------------------------------
# 目标: sim
# ------------------------------------------------------------------
# main.cpp 做 CLI11 子命令解析与分派;各模式是模块 mininav.apps.sim 的实现单元。
add_executable(sim)

target_sources(sim
        PRIVATE
        src/apps/sim/main.cpp
        src/apps/sim/common.cpp
        src/apps/sim/ekf_mode.cpp
        src/apps/sim/plan_mode.cpp

        PRIVATE
        FILE_SET CXX_MODULES FILES
        src/apps/sim/sim.ixx
)

target_link_libraries(sim PRIVATE core viz simulation localization planning CLI11::CLI11)

target_compile_definitions(sim PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

mininav_set_warnings(sim)

# ------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------
enable_testing()
include(google_test)
add_subdirectory(tests)