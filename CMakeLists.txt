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

# ------------------------------------------------------------------
# 子模块库
# ------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/viz)
add_subdirectory(src/sensors)
add_subdirectory(src/localization)

# ------------------------------------------------------------------
# 目标: sim_v0
# ------------------------------------------------------------------
add_executable(sim_v0
        src/apps/sim_v0_main.cpp
)

target_link_libraries(sim_v0 PRIVATE core viz)

target_compile_definitions(sim_v0 PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

mininav_set_warnings(sim_v0)

# ------------------------------------------------------------------
# 目标: sim_v1
# ------------------------------------------------------------------
add_executable(sim_v1
        src/apps/sim_v1_main.cpp)

target_link_libraries(sim_v1 PRIVATE core viz sensors localization CLI11::CLI11)

target_compile_definitions(sim_v1 PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

mininav_set_warnings(sim_v1)

# ------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------
enable_testing()
include(google_test)
add_subdirectory(tests)