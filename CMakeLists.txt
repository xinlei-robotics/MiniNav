cmake_minimum_required(VERSION 3.28)

project(MiniNav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)

# ------------------------------------------------------------------
# 构建选项
# ------------------------------------------------------------------
# 缺省全开,与以前的构建相同。ROS 2 工作空间只需要不依赖 ROS 的算法库:关掉 viz
# (连带 sim 与测试)就不再拉 Rerun / CLI11 / GoogleTest。依赖链:sim 用 viz,
# 测试要跑 sim(golden 回归、nav)并测 viz。
include(CMakeDependentOption)
option(MININAV_WITH_VIZ "Build the Rerun-backed viz library" ON)
cmake_dependent_option(MININAV_BUILD_APPS "Build the sim application (requires viz)" ON
        "MININAV_WITH_VIZ" OFF)
cmake_dependent_option(MININAV_BUILD_TESTS "Build the unit, regression and nav tests (requires sim)" ON
        "MININAV_BUILD_APPS" OFF)
option(MININAV_REQUIRE_SYSTEM_DEPS
        "Take Eigen, spdlog and yaml-cpp from the system only; never download them" OFF)
option(MININAV_INSTALL
        "Install the ROS-free libraries as the CMake package MiniNav (requires CMake >= 3.31)" OFF)

message(STATUS "MiniNav: viz=${MININAV_WITH_VIZ} apps=${MININAV_BUILD_APPS} "
        "tests=${MININAV_BUILD_TESTS} system-deps-only=${MININAV_REQUIRE_SYSTEM_DEPS} "
        "install=${MININAV_INSTALL}")

# Nav2 插件是共享库,要把这些静态库链接进去,所以统一编成位置无关代码。
# PIC 只改变寻址方式,不改变浮点语义;golden 回归逐字节验证这一点。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------
# 第三方依赖
# ------------------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
if (MININAV_WITH_VIZ)
    include(rerun)
endif ()
if (MININAV_BUILD_APPS)
    include(cli11)
endif ()
include(yaml_cpp)
include(spdlog)

# ------------------------------------------------------------------
# 子模块库
# ------------------------------------------------------------------
add_subdirectory(src/core)
if (MININAV_WITH_VIZ)
    add_subdirectory(src/viz)
endif ()
add_subdirectory(src/sensors)
add_subdirectory(src/simulation)
add_subdirectory(src/localization)
add_subdirectory(src/planning)
add_subdirectory(src/control)

# 安装与导出(ROS 2 工作空间的 mininav_core 包装包打开它)
if (MININAV_INSTALL)
    include(mininav_install)
endif ()

# ------------------------------------------------------------------
# 目标: sim
# ------------------------------------------------------------------
if (MININAV_BUILD_APPS)
    # main.cpp 做 CLI11 子命令解析与分派;各模式是模块 mininav.apps.sim 的实现单元。
    add_executable(sim)

    target_sources(sim
            PRIVATE
            src/apps/sim/main.cpp
            src/apps/sim/common.cpp
            src/apps/sim/ekf_mode.cpp
            src/apps/sim/plan_mode.cpp
            src/apps/sim/nav_mode.cpp

            PRIVATE
            FILE_SET CXX_MODULES FILES
            src/apps/sim/sim.ixx
    )

    target_link_libraries(sim PRIVATE core viz simulation localization planning control
            CLI11::CLI11 yaml-cpp::yaml-cpp)

    target_compile_definitions(sim PRIVATE
            PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
    )

    mininav_set_warnings(sim)
endif ()

# ------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------
if (MININAV_BUILD_TESTS)
    enable_testing()
    include(google_test)
    add_subdirectory(tests)
endif ()
