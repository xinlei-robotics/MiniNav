cmake_minimum_required(VERSION 3.28)

project(MiniNav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)

# --------------------------------------------------
# 依赖项
# --------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ------------------------------------------------------------------
# 公共模块与实现源文件
# ------------------------------------------------------------------
set(CORE_MODULE_SOURCES
        src/core/types.ixx
        src/core/math.ixx
        src/core/kinematics.ixx
        src/core/robot_model.ixx
)

set(CORE_IMPL_SOURCES
        src/core/types.cpp
        src/core/kinematics.cpp
        src/core/robot_model.cpp
)

# ------------------------------------------------------------------
# 目标：sim_v0
# ------------------------------------------------------------------
add_executable(sim_v0)

target_sources(sim_v0
        PRIVATE
        ${CORE_IMPL_SOURCES}
        src/core/csv_format.cpp
        src/core/staged_command_source.cpp
        src/apps/sim_v0_main.cpp
        PRIVATE
        FILE_SET CXX_MODULES FILES
        ${CORE_MODULE_SOURCES}
        src/core/trajectory.ixx
        src/core/csv_format.ixx
        src/core/csv_writer.ixx
        src/core/logger.ixx
        src/core/command_source.ixx
)

target_link_libraries(sim_v0 PRIVATE Eigen3::Eigen)

# 将项目根目录作为宏传递给代码，便于使用绝对路径输出文件
target_compile_definitions(sim_v0 PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

set_property(TARGET sim_v0 PROPERTY CXX_SCAN_FOR_MODULES ON)

# 显式指定用 C++ 链接驱动，防止 CMake 误用 clang 替代 clang++
set_target_properties(sim_v0 PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(sim_v0)

# ------------------------------------------------------------------
# 目标：robot_model_tests
# ------------------------------------------------------------------
add_executable(robot_model_tests)

target_sources(robot_model_tests
        PRIVATE
        ${CORE_IMPL_SOURCES}
        tests/robot_model_tests.cpp
        PRIVATE
        FILE_SET CXX_MODULES FILES
        ${CORE_MODULE_SOURCES}
)

target_link_libraries(robot_model_tests PRIVATE Eigen3::Eigen)

set_property(TARGET robot_model_tests PROPERTY CXX_SCAN_FOR_MODULES ON)

set_target_properties(robot_model_tests PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(robot_model_tests)

# ------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------
enable_testing()
add_test(NAME robot_model_tests COMMAND robot_model_tests)