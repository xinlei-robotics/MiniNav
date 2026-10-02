# ---------------------------------------------------------------------------
# src/control/CMakeLists.txt
#
# control 库(V4 新增):路径跟踪。接口对齐 nav2_core 的 Controller / GoalChecker /
# ProgressChecker,实现为 Regulated Pure Pursuit 子集 + 速度平滑器。
# 只依赖 core —— 控制器只认 Path / Pose2D / Twist2D,不认识栅格、EKF、传感器,
# 就像 Nav2 控制器只认 nav_msgs/Path 与位姿。yaml-cpp 只用于配置解析(.cpp),
# PRIVATE 链接。
# ---------------------------------------------------------------------------

add_library(control STATIC)

target_sources(control
        PRIVATE
        pure_pursuit.cpp
        velocity_smoother.cpp
        goal_checker.cpp
        progress_checker.cpp
        controller_config.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        controller.ixx
        pure_pursuit.ixx
        velocity_smoother.ixx
        goal_checker.ixx
        progress_checker.ixx
        controller_config.ixx
)

target_link_libraries(control
        PUBLIC core
        PRIVATE yaml-cpp::yaml-cpp
)

set_property(TARGET control PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(control PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(control)
