module;

#include <Eigen/Core>

#include <string_view>

export module mininav.planning.path_smoothing;

import mininav.core.types;
import mininav.core.path;
import mininav.planning.grid_types;
import mininav.planning.occupancy_grid;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // PathSmoothingConfig: A* 原始路径的后处理(config/nav.yaml 的 path_smoothing 段)。
    //
    //   snap_endpoints  首尾 waypoint 换成真实起止点(A* 只给格心,docs/v3_summary.md §8.5)
    //   shortcut        视线捷径:8 连通台阶折线拉直(§8.1)。台阶的每个 45° 拐角
    //                   都是曲率冲激,是 Pure Pursuit 最差的输入。
    // ---------------------------------------------------------------------------
    struct PathSmoothingConfig
    {
        bool snap_endpoints{true};
        bool shortcut{true};
    };

    // 解析 path_smoothing 段的 YAML 映射。缺失字段取默认值;未知 key、类型错误抛
    // std::runtime_error。空文本 = 全部默认。
    [[nodiscard]] PathSmoothingConfig parse_path_smoothing_config(std::string_view yaml_text);

    // ---------------------------------------------------------------------------
    // segment_is_free: 线段 a→b 经过的每个 cell 都可通行(world 坐标)。
    //
    // supercover 遍历(Amanatides–Woo DDA 的保守版):线段恰好穿过格角时,角上的两个
    // 正交邻居都算"被经过"并检查 —— 与 A* 的防穿角规则一致,所以 A* 的原始路径
    // 每一段都满足本检查。端点所在 cell 也检查;越界即不可通行。
    // ---------------------------------------------------------------------------
    [[nodiscard]] bool segment_is_free(const OccupancyGrid& costmap, const Eigen::Vector2d& a,
                                       const Eigen::Vector2d& b, bool allow_unknown);

    // ---------------------------------------------------------------------------
    // smooth_path: 在 A* 用的同一张膨胀 costmap、同一套可通行规则下后处理原始路径。
    //
    //   1. 首尾替换:首点换成 start、末点换成 goal;若真实端点到相邻 waypoint 的线段
    //      不可通行,保留格心作为过渡点(端点与格心同在一个 cell,这一小段必可通行)。
    //   2. 视线捷径:从锚点 i 向前,只要 i 到下一个 waypoint 的连线仍可通行就继续延伸,
    //      取最后一个可见的 waypoint 作为新锚点,直到终点。
    //      性质:端点不变;每段都可通行;长度不增(三角不等式);只在原 waypoint 中挑,
    //      不保证 any-angle 最短(那需要 Theta* 或可见图)。
    //   3. yaw:每个 waypoint 指向下一个,末点沿用前一段方向(与 A* 相同);
    //      需要目标朝向时由调用方改写末点 yaw。
    //
    // planner.cost_weight > 0 时跳过捷径并告警:代价梯度刻意让路径远离障碍,
    // 捷径会把它重新拉近障碍。raw 为空(规划失败)时返回空路径。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Path smooth_path(const Path& raw, const OccupancyGrid& costmap,
                                   const PlannerConfig& planner, const Pose2D& start,
                                   const Pose2D& goal, const PathSmoothingConfig& cfg);
}
