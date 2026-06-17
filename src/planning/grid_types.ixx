module;

#include <cstddef>
#include <vector>

export module mininav.planning.grid_types;

import mininav.core.types;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // GridCoord: 离散栅格索引
    //   x = col(列), y = row(行),均为整型 cell 下标。
    // world↔grid 的转换由 OccupancyGrid 持有(分辨率 / 原点),见 PR1。
    // ---------------------------------------------------------------------------
    struct GridCoord
    {
        int x{0}; // col
        int y{0}; // row

        bool operator==(const GridCoord&) const noexcept = default;
    };

    // ---------------------------------------------------------------------------
    // Heuristic: A* 启发式选择。
    //   admissible 性(≤ 真实代价,保证最优)的证明留给 docs/math/astar_planning.md。
    // ---------------------------------------------------------------------------
    enum class Heuristic
    {
        Manhattan,
        Euclidean,
        Octile,
    };

    // ---------------------------------------------------------------------------
    // Connectivity: 栅格邻接度。数值即邻居数,便于 YAML 序列化为 4 / 8。
    // ---------------------------------------------------------------------------
    enum class Connectivity
    {
        Four = 4,
        Eight = 8,
    };

    // ---------------------------------------------------------------------------
    // Path: 几何路径 = world 坐标的 waypoint 序列。
    //   yaw 在 V3 非必需(跟踪在 V4 用 look-ahead 自算朝向),但保留 Pose2D
    //   以便 V4 直接消费。length() 是路径-最优性比对的核心量。
    // ---------------------------------------------------------------------------
    struct Path
    {
        std::vector<Pose2D> poses;

        [[nodiscard]] bool empty() const noexcept { return poses.empty(); }
        [[nodiscard]] std::size_t size() const noexcept { return poses.size(); }

        // 相邻 waypoint 的累积欧氏长度(忽略 yaw)。
        [[nodiscard]] double length() const noexcept;
    };

    // ---------------------------------------------------------------------------
    // PlannerConfig: 规划器配置(来自 planner.yaml)。
    //   PR0 仅定义数据形态;inflation_radius / cost_weight 等字段在 PR3 / PR4
    //   才被膨胀与 A* 算法消费。cost_weight 在 MVP 布尔膨胀下为 0。
    // ---------------------------------------------------------------------------
    struct PlannerConfig
    {
        double inflation_radius{0.0};                 // 米
        Heuristic heuristic{Heuristic::Euclidean};
        Connectivity connectivity{Connectivity::Eight};
        bool allow_unknown{false};
        double cost_weight{0.0};                      // 代价梯度权重,MVP 下为 0
    };
}
