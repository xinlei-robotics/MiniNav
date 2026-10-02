# ---------------------------------------------------------------------------
# src/core/CMakeLists.txt
#
# core 库:封装项目核心数据结构、运动学、路径几何、机器人描述、轨迹、CSV I/O
# 等基础设施。
# 以 STATIC 库形式提供给 sim、core_tests 等下游目标使用。
# ---------------------------------------------------------------------------

add_library(core STATIC)

target_sources(core
        PRIVATE
        types.cpp
        path.cpp
        robot_description.cpp
        integrators.cpp
        kinematics.cpp
        csv_format.cpp
        staged_command_source.cpp
        logger.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        types.ixx
        path.ixx
        robot_description.ixx
        math.ixx
        integrators.ixx
        kinematics.ixx
        trajectory.ixx
        csv_format.ixx
        csv_writer.ixx
        logger.ixx
        command_source.ixx
        random.ixx
)

# spdlog 与 yaml-cpp 只出现在 .cpp 的全局模块片段里,PRIVATE 链接,不向下游泄漏。
target_link_libraries(core
        PUBLIC Eigen3::Eigen
        PRIVATE spdlog::spdlog yaml-cpp::yaml-cpp)

set_property(TARGET core PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(core PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(core)
