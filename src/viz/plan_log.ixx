module;

#include <Eigen/Core>

#include <string_view>
#include <vector>

export module mininav.viz.plan_log;

import mininav.core.types;
import mininav.viz.sink;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // PlanScene: 一次全局规划的**纯几何**快照,供可视化下沉。
    //
    // 关键边界:本结构只认 Eigen::Vector2d(world 坐标)与 Pose2D —— 不认任何
    // planning 类型(OccupancyGrid / Path / PlanResult)。因此 viz 库不依赖
    // planning 库(见 docs/v3_plan.md §2.1 的依赖图)。把 OccupancyGrid 的占据
    // cell、膨胀 cell、A* 路径折线转成点集,是上层(app)的职责;log_plan 只负责
    // 把这些点集按约定的实体树布局推到 VizSink。
    //
    // 所有点都是 world 坐标 cell 中心 / waypoint;z 隐含为 0(规划是 2D)。
    // ---------------------------------------------------------------------------
    struct PlanScene
    {
        std::vector<Eigen::Vector2d> obstacle_cells;  // 原始占据 cell 中心
        std::vector<Eigen::Vector2d> inflated_cells;  // 膨胀新增的 cell 中心(安全裕度)
        std::vector<Eigen::Vector2d> path;            // 最终路径 waypoint 序列
        std::vector<Eigen::Vector2d> expansion;       // A* 扩展顺序(可选,空则不画)
        Pose2D start;                                 // 规划起点(world)
        Pose2D goal;                                  // 规划目标(world)
        float cell_radius{0.05F};                     // cell 方块的渲染半径(≈ 0.5 分辨率)
    };

    // ---------------------------------------------------------------------------
    // log_plan: 把一次规划场景推到 VizSink,实体树布局对齐 docs/v3_plan.md §6.1:
    //   {root}/map            占据 cell(深色点)
    //   {root}/map/inflated   膨胀层(半透明灰)
    //   {root}/plan/expansion A* 扩展顺序(可选,蓝)
    //   {root}/plan/path      最终路径折线(绿)+ waypoint
    //   {root}/robot/start    起点轴(world pose)
    //   {root}/robot/goal     目标轴(world pose)
    //
    // 一次性场景全部用 static log(set_time 之外亦可见)。需要 viz 后端时由
    // RerunSink 实现;测试用 MockVizSink 验证调用契约,不起 Viewer。
    // ---------------------------------------------------------------------------
    void log_plan(VizSink& sink, const PlanScene& scene, std::string_view entity_root);
}
