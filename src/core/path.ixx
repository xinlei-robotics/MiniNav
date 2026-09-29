module;

#include <Eigen/Core>

#include <cstddef>
#include <vector>

export module mininav.core.path;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // Path: 几何路径 = world 坐标的 waypoint 序列(折线)。
    //
    // 放在 core 而不是 planning:规划器产出它、控制器消费它,就像 Nav2 里的
    // nav_msgs/Path 是公共消息类型而不属于规划器包。控制器因此只依赖 core,不认识
    // 栅格。yaw 由生产者填写(A* 取相邻 waypoint 的朝向);几何工具只用 (x, y)。
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
    // PathProjection: 点在折线上的最近投影。
    //
    //   segment    所在线段下标 i(线段 i 连接 poses[i] → poses[i+1])
    //   t          段内参数 ∈ [0, 1]
    //   arclength  投影点距路径起点的弧长
    //   point      投影点
    //   distance   点到投影点的距离 = 点到路径的横向误差
    //
    // 单点路径没有线段:投影就是该点本身(segment = 0, t = 0)。
    // ---------------------------------------------------------------------------
    struct PathProjection
    {
        std::size_t segment{0};
        double t{0.0};
        double arclength{0.0};
        Eigen::Vector2d point{Eigen::Vector2d::Zero()};
        double distance{0.0};
    };

    // ---------------------------------------------------------------------------
    // project_onto: 在线段窗口 [from_segment, from_segment + window) 内求最近投影。
    //
    // 只在窗口内搜索,是为了让跟踪进度单调推进:路径回折(U 形弯、走廊里折返)时,
    // 回程段可能比当前段离车更近,全局最近投影会"跳段"。距离相同取下标小的段。
    // from_segment 越界时钳到最后一段;window 至少为 1。前置条件:path 非空。
    // ---------------------------------------------------------------------------
    [[nodiscard]] PathProjection project_onto(const Path& path, const Eigen::Vector2d& p,
                                              std::size_t from_segment, std::size_t window);

    // ---------------------------------------------------------------------------
    // lookahead_point: 从投影点沿路径向前,找第一个与 p 相距 L 的点(圆-线段求交)。
    //
    // 从 from.point 出发逐段向前:当前点在以 p 为圆心、半径 L 的圆内时,本段的出圆点
    // 是二次方程 |a + s·(b − a) − p|² = L² 的较大根。投影点之前的路径(车后方)
    // 不参与,所以不会选到身后的交点。
    //   - 剩余路径全在圆内(接近终点):返回终点;
    //   - 投影点本身已在圆外(车离路径超过 L):返回投影点,即先驶回路径。
    // 前置条件:path 非空,L > 0,from 来自 project_onto(path, p, ...)。
    // ---------------------------------------------------------------------------
    [[nodiscard]] Eigen::Vector2d lookahead_point(const Path& path, const PathProjection& from,
                                                  const Eigen::Vector2d& p, double L);
}
