module;

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

module mininav.viz.plan_log;

import mininav.core.types;
import mininav.viz.sink;

namespace mininav
{
    namespace
    {
        // 配色(RGB):占据=深灰、膨胀=中灰、扩展=蓝、路径=绿。
        constexpr std::array<std::uint8_t, 3> kObstacleColor{60, 60, 70};
        constexpr std::array<std::uint8_t, 3> kInflatedColor{150, 150, 165};
        constexpr std::array<std::uint8_t, 3> kExpansionColor{70, 130, 220};
        constexpr std::array<std::uint8_t, 3> kPathColor{40, 200, 90};
    } // namespace

    void log_plan(VizSink& sink, const PlanScene& scene,
                  const std::string_view entity_root)
    {
        const std::string root{entity_root};

        // 占据 cell 与膨胀层:方块用点 + 半径近似(cell_radius ≈ 0.5·分辨率)。
        sink.log_points_static(root + "/map", scene.obstacle_cells,
                               kObstacleColor, scene.cell_radius);
        sink.log_points_static(root + "/map/inflated", scene.inflated_cells,
                               kInflatedColor, scene.cell_radius);

        // A* 扩展顺序(可选):仅当上层填了 expansion 才画。
        if (!scene.expansion.empty())
        {
            sink.log_points_static(root + "/plan/expansion", scene.expansion,
                                   kExpansionColor, scene.cell_radius * 0.5F);
        }

        // 最终路径:折线 + waypoint 点(便于看清台阶)。
        sink.log_line_strip_static(root + "/plan/path", scene.path, kPathColor);
        sink.log_points_static(root + "/plan/path/waypoints", scene.path,
                               kPathColor, scene.cell_radius * 0.6F);

        // 起点 / 目标:复用 pose 轴(world 系)。
        sink.log_pose(root + "/robot/start", scene.start);
        sink.log_axes(root + "/robot/start", 0.3F);
        sink.log_pose(root + "/robot/goal", scene.goal);
        sink.log_axes(root + "/robot/goal", 0.3F);
    }
}
