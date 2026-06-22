module;

#include <cstddef>
#include <vector>

export module mininav.planning.astar;

import mininav.core.types;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // PlanResult: 一次规划的产出 + 实验指标。
    //   path           world 坐标的 waypoint 序列(失败时为空)
    //   success        是否找到路径
    //   expanded_nodes A* 实际展开(出队处理)的节点数 —— 启发式效率指标
    //   plan_time_ms   规划耗时(毫秒)—— 对齐里程碑的 50ms 指标
    // ---------------------------------------------------------------------------
    struct PlanResult
    {
        Path path;
        bool success{false};
        std::size_t expanded_nodes{0};
        double plan_time_ms{0.0};
    };

    // ---------------------------------------------------------------------------
    // GlobalPlanner: 全局规划器门面,签名形态对齐 nav2_core::GlobalPlanner
    // (用项目自己的 Pose2D / Path,不依赖任何 ROS 消息类型)。V4 引入 ROS 2
    // 时只需一层薄适配器。
    // ---------------------------------------------------------------------------
    class GlobalPlanner
    {
    public:
        GlobalPlanner() = default;
        virtual ~GlobalPlanner() = default;
        GlobalPlanner(const GlobalPlanner&) = default;
        GlobalPlanner& operator=(const GlobalPlanner&) = default;
        GlobalPlanner(GlobalPlanner&&) = default;
        GlobalPlanner& operator=(GlobalPlanner&&) = default;

        [[nodiscard]] virtual PlanResult plan(const Pose2D& start,
                                              const Pose2D& goal) const = 0;
    };

    // ---------------------------------------------------------------------------
    // AStarPlanner: 栅格上的 A* 最优搜索。
    //
    // 构造时按 cfg.inflation_radius 在内部把地图膨胀成 costmap(详见 inflate);
    // 若 cfg.cost_weight > 0,再算一张代价梯度场(近障碍高代价),让等长路径
    // 倾向远离障碍——A* 的 g 里叠加 cost_weight * cost(cell)。cost_weight = 0
    // 时退化为纯最短路径(此时各 admissible 启发式给出相同最优长度)。
    //
    // 连通度 / 启发式 / 是否可走 unknown 均来自 cfg。8 连通带防穿角
    // (corner-cutting:对角移动要求两个正交邻居都可走)。
    // ---------------------------------------------------------------------------
    class AStarPlanner final : public GlobalPlanner
    {
    public:
        AStarPlanner(OccupancyGrid grid, PlannerConfig cfg);

        [[nodiscard]] PlanResult plan(const Pose2D& start,
                                      const Pose2D& goal) const override;

    private:
        OccupancyGrid grid_;        // 膨胀后的 costmap
        PlannerConfig cfg_;
        std::vector<double> cost_;  // 每 cell 代价梯度(cost_weight<=0 时为空)
    };
}
