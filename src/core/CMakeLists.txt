# ---------------------------------------------------------------------------
# src/core/CMakeLists.txt
#
# core 库:封装项目核心数据结构、运动学、轨迹、CSV I/O 等基础设施。
# 以 STATIC 库形式提供给 sim_v0、core_tests 等下游目标使用。
#
# 模块文件 (.ixx) 通过 PUBLIC FILE_SET CXX_MODULES 暴露,
# 下游目标 link 到 core 后即可直接 import 对应模块。
# 实现文件 (.cpp) 是 PRIVATE,只参与 core 自身编译。
# ---------------------------------------------------------------------------

add_library(core STATIC)

target_sources(core
        PRIVATE
        types.cpp
        kinematics.cpp
        robot_model.cpp
        csv_format.cpp
        staged_command_source.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        types.ixx
        math.ixx
        kinematics.ixx
        robot_model.ixx
        trajectory.ixx
        csv_format.ixx
        csv_writer.ixx
        logger.ixx
        command_source.ixx
)

target_link_libraries(core PUBLIC Eigen3::Eigen)

set_property(TARGET core PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(core PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(core)