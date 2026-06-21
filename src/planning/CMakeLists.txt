# ---------------------------------------------------------------------------
# src/planning/CMakeLists.txt
#
# planning 库(V3 新增):占据栅格地图 + 全局路径规划。
# 只依赖 core(几何类型 Pose2D)与 yaml-cpp(配置解析),
# 不依赖 sensors / localization —— 层与层之间只通过 plain 类型耦合。
# yaml-cpp 仅在实现单元(.cpp 全局模块片段)使用,故 PRIVATE 链接,
# 不向下游(tests / sim)泄漏。
# ---------------------------------------------------------------------------

add_library(planning STATIC)

target_sources(planning
        PRIVATE
        grid_types.cpp
        planner_config.cpp
        occupancy_grid.cpp
        map_io.cpp
        inflation.cpp
        astar.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        grid_types.ixx
        planner_config.ixx
        occupancy_grid.ixx
        map_io.ixx
        inflation.ixx
        astar.ixx
)

target_link_libraries(planning
        PUBLIC core
        PRIVATE yaml-cpp::yaml-cpp
)

set_property(TARGET planning PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(planning PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(planning)
