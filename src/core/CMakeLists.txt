# ---------------------------------------------------------------------------
# src/core/CMakeLists.txt
#
# core 库:封装项目核心数据结构、运动学、轨迹、CSV I/O 等基础设施。
# 以 STATIC 库形式提供给 sim、core_tests 等下游目标使用。
# ---------------------------------------------------------------------------

add_library(core STATIC)

target_sources(core
        PRIVATE
        types.cpp
        integrators.cpp
        kinematics.cpp
        csv_format.cpp
        staged_command_source.cpp
        logger.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        types.ixx
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

target_link_libraries(core PUBLIC Eigen3::Eigen)

set_property(TARGET core PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(core PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(core)
