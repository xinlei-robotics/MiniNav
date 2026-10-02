module;

#include <Eigen/Core>

#include <string_view>
#include <vector>

export module mininav.viz.nav_log;

import mininav.core.types;
import mininav.viz.sink;
import mininav.viz.plan_log;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // NavScene: 闭环导航的静态场景(纯几何,viz 不依赖 planning / control)。
    //   plan      地图、膨胀层、平滑后的路径、起止位姿(复用 PlanScene / log_plan)
    //   raw_path  A* 原始路径(台阶折线),与平滑结果对照
    // ---------------------------------------------------------------------------
    struct NavScene
    {
        PlanScene plan;
        std::vector<Eigen::Vector2d> raw_path;
    };

    // 静态实体:log_plan 的全部实体 + {world_root}/plan/raw(灰色折线)。
    void log_nav_scene(VizSink& sink, const NavScene& scene, std::string_view world_root);

    // ---------------------------------------------------------------------------
    // log_to_rerun(NavStep):闭环一帧(ADL 重载,与 SimState 的同名函数并列)。
    //
    // robot_root(如 "/world/robot")下先走 SimState 的全部通道(truth / odom / ekf
    // 位姿、trail、指令、编码器……),再加导航通道。几何量都是 world 坐标,所以放在
    // robot_root 的父级下,不挂在带 Transform3D 的位姿实体下面:
    //   {robot_root}/actuator/{v,w}       电机输出 u(饱和 + 滞后后)
    //   {world}/control/lookahead         look-ahead 点
    //   {world}/control/arc               追踪圆弧:控制器输入位姿 → look-ahead 点
    //   {world}/estimate/ekf_cov          EKF 位置 3σ 椭圆
    //   /plots/error/{ctrl,est,true}      误差分解(docs/v4_plan.md §3.6)
    //   /plots/clearance                  真值净空
    //   /plots/regime                     控制工况编号
    // ---------------------------------------------------------------------------
    void log_to_rerun(VizSink& sink, const NavStep& step, std::string_view robot_root);

    // 位置协方差 [[a, b], [b, c]] 的 k-σ 椭圆(闭合折线,segments + 1 个点)。
    [[nodiscard]] std::vector<Eigen::Vector2d> covariance_ellipse(const Eigen::Vector2d& center, double a,
                                                                  double b, double c, double k,
                                                                  int segments);

    // 从 pose 出发、曲率 κ、到 target 为止的圆弧(Pure Pursuit 的追踪弧,segments + 1 个点)。
    // target 在车后方(|方位角| > 90°)时弧不存在,返回空。
    [[nodiscard]] std::vector<Eigen::Vector2d> pursuit_arc(const Pose2D& pose, double curvature,
                                                           const Eigen::Vector2d& target, int segments);
}
