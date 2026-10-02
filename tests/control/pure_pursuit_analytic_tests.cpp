import mininav.control.pure_pursuit;
import mininav.control.velocity_smoother;
import mininav.control.goal_checker;
import mininav.core.types;
import mininav.core.path;
import mininav.core.kinematics;
import mininav.core.robot_description;

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

// ===========================================================================
// 解析用例:闭环行为与 docs/v4_plan.md §3 的第一性原理推导对照。
//
// 直线附近线性化后(e 横向偏差,s 走过的距离):
//   e'' + (2/L)·e' + (2/L²)·e = 0,ζ = 1/√2,且与速度无关;
//   e(0) = e₀、与路径平行出发时 e(s) = e₀·e^(−s/L)·(cos(s/L) + sin(s/L)),
//   反向超调 e₀·e^(−π)(在 s = π·L 处),2% 调节距离 L·ln(50√2) ≈ 4.26·L。
// 圆弧上稳态误差为零;折线转角的最大切角 δ = f(θ)·L,f(90°) ≈ 0.271。
//
// 这里模拟连续时间的纯运动学闭环:积分步 1 ms,每步都调用控制器,没有执行器
// 滞后与零阶保持 —— 正是推导所假设的对象。控制器取经典 PP(固定 L、不限速、
// 不原地转向),角速度上限放宽到不起作用。
// ===========================================================================

namespace {

using mininav::Path;
using mininav::Pose2D;
using mininav::RobotLimits;
using mininav::Twist2D;
using mininav::control::PurePursuitConfig;
using mininav::control::PurePursuitController;

constexpr double kDt = 1e-3;
constexpr RobotLimits kLooseLimits{.max_linear_vel = 1.0, .max_angular_vel = 50.0};

[[nodiscard]] PurePursuitConfig classic_pp(const double L, const double v) {
  PurePursuitConfig cfg{};
  cfg.desired_linear_vel = v;
  cfg.use_velocity_scaled_lookahead = false;
  cfg.lookahead_dist = L;
  cfg.use_curvature_regulation = false;
  cfg.use_rotate_to_heading = false;
  cfg.min_approach_vel = 0.0;
  return cfg;
}

[[nodiscard]] Path polyline(const std::vector<Eigen::Vector2d>& points) {
  Path path;
  for (const Eigen::Vector2d& q : points) {
    path.poses.emplace_back(q.x(), q.y(), 0.0);
  }
  return path;
}

struct Trace {
  std::vector<double> s;     // 走过的距离
  std::vector<Pose2D> pose;  // 真值位姿
};

// 连续时间近似的闭环:走满 distance 为止。
[[nodiscard]] Trace run_closed_loop(const PurePursuitConfig& cfg, const Path& path,
                                    const Pose2D& start, const double distance) {
  PurePursuitController ctrl{cfg, kLooseLimits};
  ctrl.set_plan(path);
  Trace trace;
  Pose2D pose = start;
  double s = 0.0;
  while (s < distance) {
    trace.s.push_back(s);
    trace.pose.push_back(pose);
    const Twist2D cmd = ctrl.compute_velocity_commands(pose, Twist2D{cfg.desired_linear_vel, 0.0}, nullptr);
    pose = mininav::differential_drive_step(pose, cmd, kDt);
    s += cmd.v() * kDt;
  }
  return trace;
}

// 直线 y = 0 上、初始偏置 e0 且与路径平行出发时的横向偏差曲线。
[[nodiscard]] Trace straight_line_response(const double L, const double v, const double e0) {
  const Path line = polyline({{-1.0, 0.0}, {100.0, 0.0}});
  return run_closed_loop(classic_pp(L, v), line, Pose2D{0.0, e0, 0.0}, 12.0 * L);
}

// 最后一次 |e| > 2%·e0 的位置(之后一直在 2% 带内)。
[[nodiscard]] double settling_distance(const Trace& t, const double e0) {
  double settle = 0.0;
  for (std::size_t i = 0; i < t.s.size(); ++i) {
    if (std::abs(t.pose[i].y()) > 0.02 * e0) {
      settle = t.s[i];
    }
  }
  return settle;
}

const double kOvershootRatio = std::exp(-std::numbers::pi);           // ≈ 4.32%
const double kSettlingInL = std::log(50.0 * std::numbers::sqrt2);     // ≈ 4.26

} // namespace

TEST(PurePursuitAnalytic, ReverseOvershootIsExpMinusPiAtPiL) {
  constexpr double e0 = 0.05;
  for (const double L : {0.2, 0.3, 0.5}) {
    const Trace t = straight_line_response(L, 0.3, e0);
    const auto it = std::ranges::min_element(t.pose, {}, [](const Pose2D& p) { return p.y(); });
    const std::size_t i = static_cast<std::size_t>(it - t.pose.begin());
    const double overshoot = -it->y() / e0;
    EXPECT_NEAR(overshoot, kOvershootRatio, 0.10 * kOvershootRatio) << "L = " << L;
    EXPECT_NEAR(t.s[i] / L, std::numbers::pi, 0.10 * std::numbers::pi) << "L = " << L;
  }
}

TEST(PurePursuitAnalytic, SettlingDistanceIsLn50Sqrt2TimesL) {
  constexpr double e0 = 0.05;
  for (const double L : {0.2, 0.3, 0.5}) {
    const Trace t = straight_line_response(L, 0.3, e0);
    EXPECT_NEAR(settling_distance(t, e0) / L, kSettlingInL, 0.10 * kSettlingInL) << "L = " << L;
  }
}

// 距离域方程里速度消失:同一 L 下,0.1 m/s 与 0.5 m/s 走出同一条空间曲线。
TEST(PurePursuitAnalytic, DistanceDomainResponseIsIndependentOfSpeed) {
  constexpr double L = 0.3;
  constexpr double e0 = 0.05;
  const Trace slow = straight_line_response(L, 0.1, e0);
  const Trace fast = straight_line_response(L, 0.5, e0);
  const auto min_y = [](const Trace& t) {
    return std::ranges::min(t.pose, {}, [](const Pose2D& p) { return p.y(); }).y();
  };
  EXPECT_NEAR(min_y(slow), min_y(fast), 0.01 * e0);
  EXPECT_NEAR(settling_distance(slow, e0), settling_distance(fast, e0), 0.01 * L);
}

// 恒曲率路径:过车、与车头相切、过 look-ahead 点的圆唯一,就是路径圆本身 ⇒ 零稳态误差。
TEST(PurePursuitAnalytic, ConstantCurvaturePathHasZeroSteadyStateError) {
  constexpr double R = 1.0;
  constexpr std::size_t kVertices = 720;  // 0.5° 一段,弦的拱高 ≈ 1e-5 m
  std::vector<Eigen::Vector2d> circle;
  for (std::size_t i = 0; i <= kVertices; ++i) {
    const double a = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(kVertices);
    circle.emplace_back(R * std::sin(a), R * (1.0 - std::cos(a)));  // 从原点出发,朝 +x,逆时针
  }
  const Path path = polyline(circle);
  PurePursuitConfig cfg = classic_pp(0.3, 0.3);
  cfg.max_projection_search_dist = 0.2;  // 稠密折线:缩小投影窗口,只为测试提速
  const Trace t = run_closed_loop(cfg, path, Pose2D{0.0, 0.0, 0.0}, 2.5);
  double worst = 0.0;
  for (const Pose2D& p : t.pose) {
    worst = std::max(worst, std::abs((p.position() - Eigen::Vector2d{0.0, R}).norm() - R));
  }
  EXPECT_LT(worst, 1e-4);
}

// 单个转角没有内禀长度 ⇒ 最大切角 δ = f(θ)·L:L 加倍,δ 加倍。f(90°) ≈ 0.271。
TEST(PurePursuitAnalytic, CornerCuttingScalesWithLookahead) {
  const auto corner_cut = [](const double L) {
    const Path path = polyline({{-20.0 * L, 0.0}, {0.0, 0.0}, {0.0, 40.0 * L}});
    const Trace t = run_closed_loop(classic_pp(L, 0.3), path, Pose2D{-15.0 * L, 0.0, 0.0}, 30.0 * L);
    double delta = 0.0;
    for (const Pose2D& p : t.pose) {
      delta = std::max(delta, mininav::project_onto(path, p.position(), 0, 2).distance);
    }
    return delta;
  };
  const double ratio_small = corner_cut(0.2) / 0.2;
  const double ratio_large = corner_cut(0.4) / 0.4;
  EXPECT_NEAR(ratio_small, ratio_large, 0.01 * ratio_large);
  EXPECT_NEAR(ratio_large, 0.271, 0.05 * 0.271);
}

// ---------------------------------------------------------------------------
// 完整 RPP(速度自适应 L、曲率限速、接近减速)+ 速度平滑器,20 Hz 零阶保持,
// 默认参数,L 形路径(转角在 (2, 0),左转 90°):
//   - 到达目标;
//   - 转角降速到 §3.5 的上限 v_corner ≈ 0.16 m/s 以下;
//   - 转角内侧的切角(§3.4 的 f(θ)·L 机制)留在有效裕度 m_eff ≈ 0.043 m 以内。
//
// 出弯后的外侧偏移是另一种机制,§3.5 的预算没有覆盖:曲率限速只看前方 L_κ 处的
// 路径几何,一过转角就解除,车带着航向误差 ψ 加速回巡航,L 随之变大。直线附近
// e(0) = 0、e'(0) = ψ 的线性响应是 e(s) = ψ·L·e^(−s/L)·sin(s/L),峰值
// 0.322·ψ·L(s = πL/4)。实测出弯时 ψ ≈ 0.56 rad、L 从 0.17 涨到 0.3 m,
// 外侧偏移 ≈ 0.044 m,与推导同量级。T_L 与这项偏移的取舍留给 E2 / E3
// (docs/v4_plan.md §11.5);这里只锁住当前默认参数下的量级。
// ---------------------------------------------------------------------------
TEST(PurePursuitAnalytic, RegulatedPursuitTakesCornerWithinInsideMarginBudget) {
  const Path path = polyline({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}});
  const RobotLimits limits{.max_linear_vel = 0.5, .max_angular_vel = 2.0};
  PurePursuitController ctrl{PurePursuitConfig{}, limits};
  ctrl.set_plan(path);
  mininav::control::VelocitySmoother smoother{mininav::control::VelocitySmootherConfig{}};
  mininav::control::SimpleGoalChecker goal{mininav::control::GoalCheckerConfig{}, false};

  constexpr double kSimDt = 0.01;
  constexpr int kControlEvery = 5;  // 20 Hz
  Pose2D pose{0.0, 0.0, 0.0};
  Twist2D cmd{};
  double inside_cut = 0.0;       // 转角内侧象限(x < 2, y > 0)
  double exit_overshoot = 0.0;   // 其余:出弯后越到外侧
  double corner_speed = 1.0;
  bool arrived = false;
  for (int k = 0; k < 6000 && !arrived; ++k) {
    if (k % kControlEvery == 0) {
      cmd = smoother.smooth(ctrl.compute_velocity_commands(pose, cmd, &goal), kControlEvery * kSimDt);
    }
    pose = mininav::differential_drive_step(pose, cmd, kSimDt);
    const double d = mininav::project_onto(path, pose.position(), 0, 2).distance;
    if (pose.x() < 2.0 && pose.y() > 0.0) {
      inside_cut = std::max(inside_cut, d);
    } else {
      exit_overshoot = std::max(exit_overshoot, d);
    }
    if ((pose.position() - Eigen::Vector2d{2.0, 0.0}).norm() < 0.05) {
      corner_speed = std::min(corner_speed, cmd.v());
    }
    arrived = goal.is_goal_reached(pose, path.poses.back(), cmd);
  }
  EXPECT_TRUE(arrived);
  EXPECT_LT(corner_speed, 0.16);
  EXPECT_LT(inside_cut, 0.043);
  EXPECT_LT(exit_overshoot, 0.05);
}
