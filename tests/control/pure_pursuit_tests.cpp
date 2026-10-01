import mininav.control.pure_pursuit;
import mininav.control.goal_checker;
import mininav.core.types;
import mininav.core.path;
import mininav.core.robot_description;

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <initializer_list>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace {

using mininav::Path;
using mininav::Pose2D;
using mininav::RobotLimits;
using mininav::Twist2D;
using mininav::control::GoalCheckerConfig;
using mininav::control::PurePursuitConfig;
using mininav::control::PurePursuitController;
using mininav::control::PursuitRegime;
using mininav::control::SimpleGoalChecker;

constexpr double kEps = 1e-12;
constexpr RobotLimits kLimits{.max_linear_vel = 0.5, .max_angular_vel = 2.0};

[[nodiscard]] Path make_path(std::initializer_list<std::pair<double, double>> xy, double last_yaw = 0.0) {
  Path path;
  for (const auto& [x, y] : xy) {
    path.poses.emplace_back(x, y, 0.0);
  }
  path.poses.back().set_yaw(last_yaw);
  return path;
}

// 经典 PP:固定 L、不限速、不原地转向 —— 用于手算核对几何。
[[nodiscard]] PurePursuitConfig fixed_lookahead(const double L) {
  PurePursuitConfig cfg{};
  cfg.use_velocity_scaled_lookahead = false;
  cfg.lookahead_dist = L;
  cfg.use_curvature_regulation = false;
  cfg.use_rotate_to_heading = false;
  return cfg;
}

[[nodiscard]] Twist2D command(PurePursuitController& ctrl, const Path& path, const Pose2D& pose,
                              const double speed = 0.0) {
  ctrl.set_plan(path);
  return ctrl.compute_velocity_commands(pose, Twist2D{speed, 0.0}, nullptr);
}

} // namespace

// ---------------------------------------------------------------------------
// 几何
// ---------------------------------------------------------------------------

// 车在原点朝 +x,路径 y = 0.1,L = 0.5:G = (√0.24, 0.1),κ = 2·0.1/0.5² = 0.8。
TEST(PurePursuit, CurvatureMatchesHandCalculation) {
  PurePursuitController ctrl{fixed_lookahead(0.5), kLimits};
  const Twist2D cmd = command(ctrl, make_path({{-1.0, 0.1}, {5.0, 0.1}}), Pose2D{0.0, 0.0, 0.0});
  EXPECT_NEAR(ctrl.debug().lookahead_point.x(), std::sqrt(0.24), kEps);
  EXPECT_NEAR(ctrl.debug().lookahead_point.y(), 0.1, kEps);
  EXPECT_NEAR(ctrl.debug().curvature, 0.8, kEps);
  EXPECT_NEAR(ctrl.debug().cross_track_error, 0.1, kEps);
  EXPECT_NEAR(cmd.v(), 0.3, kEps);
  EXPECT_NEAR(cmd.w(), 0.24, kEps);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Track);
}

// 曲率在车体系里求:同一几何旋转到任意朝向,指令不变。
TEST(PurePursuit, CurvatureIsComputedInRobotFrame) {
  PurePursuitController ctrl{fixed_lookahead(0.5), kLimits};
  const double yaw = 2.0;
  const Eigen::Vector2d dir{std::cos(yaw), std::sin(yaw)};
  const Eigen::Vector2d left{-std::sin(yaw), std::cos(yaw)};
  const Eigen::Vector2d a = dir * -1.0 + left * 0.1;
  const Eigen::Vector2d b = dir * 5.0 + left * 0.1;
  const Twist2D cmd = command(ctrl, make_path({{a.x(), a.y()}, {b.x(), b.y()}}), Pose2D{0.0, 0.0, yaw});
  EXPECT_NEAR(cmd.w(), 0.24, 1e-12);
}

TEST(PurePursuit, LookaheadScalesWithSpeedWithinBounds) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Path path = make_path({{0.0, 0.0}, {10.0, 0.0}});
  (void)command(ctrl, path, Pose2D{1.0, 0.0, 0.0}, 0.0);
  EXPECT_NEAR(ctrl.debug().lookahead_dist, 0.10, kEps);  // L_min
  (void)command(ctrl, path, Pose2D{1.0, 0.0, 0.0}, 0.3);
  EXPECT_NEAR(ctrl.debug().lookahead_dist, 0.30, kEps);  // T_L·v
  (void)command(ctrl, path, Pose2D{1.0, 0.0, 0.0}, -0.3);
  EXPECT_NEAR(ctrl.debug().lookahead_dist, 0.30, kEps);  // |v|
  (void)command(ctrl, path, Pose2D{1.0, 0.0, 0.0}, 2.0);
  EXPECT_NEAR(ctrl.debug().lookahead_dist, 0.60, kEps);  // L_max
}

// ---------------------------------------------------------------------------
// 工况
// ---------------------------------------------------------------------------

// 车头朝 −y、路径沿 +x:look-ahead 点在左侧 90°,超过 45° 阈值 → 原地左转。
TEST(PurePursuit, RotatesInPlaceTowardPathWhenAngleIsLarge) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Twist2D cmd =
      command(ctrl, make_path({{0.0, 0.0}, {5.0, 0.0}}), Pose2D{0.0, 0.0, -std::numbers::pi / 2.0});
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::RotateToPath);
  EXPECT_EQ(cmd.v(), 0.0);
  EXPECT_EQ(cmd.w(), 1.0);
}

TEST(PurePursuit, RotatesInPlaceWhenPathIsBehind) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Twist2D cmd = command(ctrl, make_path({{0.0, 0.0}, {5.0, 0.0}}), Pose2D{0.0, 0.0, std::numbers::pi});
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::RotateToPath);
  EXPECT_EQ(cmd.v(), 0.0);
  EXPECT_EQ(std::abs(cmd.w()), 1.0);
}

// 转角前 0.2 m:L_κ = 0.4 的调节点在转角后 (1, √0.12),半径 R = 0.16/(2·√0.12) ≈ 0.231 < R_min
// ⇒ v = 0.3·R/0.6 ≈ 0.115;转向用的 L_d = L_min 仍在直线段上,κ = 0。
TEST(PurePursuit, CurvatureRegulationSlowsDownBeforeCorner) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Twist2D cmd = command(ctrl, make_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 5.0}}), Pose2D{0.8, 0.0, 0.0});
  const double radius = 0.16 / (2.0 * std::sqrt(0.12));
  EXPECT_NEAR(cmd.v(), 0.3 * radius / 0.6, 1e-12);
  EXPECT_NEAR(cmd.w(), 0.0, 1e-12);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Track);
}

TEST(PurePursuit, StraightPathRunsAtDesiredSpeed) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Twist2D cmd = command(ctrl, make_path({{0.0, 0.0}, {5.0, 0.0}}), Pose2D{1.0, 0.0, 0.0}, 0.3);
  EXPECT_NEAR(cmd.v(), 0.3, kEps);
  EXPECT_NEAR(cmd.w(), 0.0, kEps);
}

// 剩余 0.2 m < approach_dist 0.4:v = 0.3·0.2/0.4 = 0.15;剩余很少时保底 0.05。
TEST(PurePursuit, ApproachSlowsDownNearGoalWithFloor) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Path path = make_path({{0.0, 0.0}, {2.0, 0.0}});
  Twist2D cmd = command(ctrl, path, Pose2D{1.8, 0.0, 0.0}, 0.3);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Approach);
  EXPECT_NEAR(cmd.v(), 0.15, 1e-12);
  cmd = command(ctrl, path, Pose2D{1.99, 0.0, 0.0}, 0.05);
  EXPECT_NEAR(cmd.v(), 0.05, 1e-12);
}

// |v·κ| 超过执行器的 ω 上限时降线速度、保持曲率:κ = 0.8,ω_max = 0.2 ⇒ v = 0.25。
TEST(PurePursuit, AngularVelocityLimitReducesLinearSpeedKeepingCurvature) {
  PurePursuitConfig cfg = fixed_lookahead(0.5);
  cfg.rotate_vel = 0.2;
  PurePursuitController ctrl{cfg, RobotLimits{.max_linear_vel = 0.5, .max_angular_vel = 0.2}};
  const Twist2D cmd = command(ctrl, make_path({{-1.0, 0.1}, {5.0, 0.1}}), Pose2D{0.0, 0.0, 0.0});
  EXPECT_NEAR(cmd.v(), 0.25, 1e-12);
  EXPECT_NEAR(cmd.w(), 0.2, 1e-12);
}

TEST(PurePursuit, SpeedLimitOnlyLowersDesiredSpeed) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Path path = make_path({{0.0, 0.0}, {5.0, 0.0}});
  const Pose2D pose{1.0, 0.0, 0.0};
  ctrl.set_speed_limit(50.0, true);
  EXPECT_NEAR(command(ctrl, path, pose, 0.3).v(), 0.15, kEps);
  ctrl.set_speed_limit(0.1, false);
  EXPECT_NEAR(command(ctrl, path, pose, 0.3).v(), 0.1, kEps);
  ctrl.set_speed_limit(0.45, false);  // 高于期望速度:不抬高
  EXPECT_NEAR(command(ctrl, path, pose, 0.3).v(), 0.3, kEps);
  ctrl.set_speed_limit(0.0, false);   // 取消限速
  EXPECT_NEAR(command(ctrl, path, pose, 0.3).v(), 0.3, kEps);
}

TEST(PurePursuit, EmptyPlanOrPathEndStops) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  Twist2D cmd = command(ctrl, Path{}, Pose2D{});
  EXPECT_EQ(cmd.v(), 0.0);
  EXPECT_EQ(cmd.w(), 0.0);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Stopped);

  cmd = command(ctrl, make_path({{0.0, 0.0}, {2.0, 0.0}}), Pose2D{2.0, 0.0, 0.0});
  EXPECT_EQ(cmd.v(), 0.0);
  EXPECT_EQ(cmd.w(), 0.0);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Stopped);
}

// 位置已进 xy 容差、目标给了朝向:原地转向目标朝向;不要求朝向时照常接近。
TEST(PurePursuit, RotatesToGoalHeadingOnceInPosition) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Path path = make_path({{0.0, 0.0}, {2.0, 0.0}}, std::numbers::pi / 2.0);
  ctrl.set_plan(path);
  const Pose2D near_goal{1.98, 0.0, 0.0};

  const SimpleGoalChecker with_yaw{GoalCheckerConfig{}, true};
  Twist2D cmd = ctrl.compute_velocity_commands(near_goal, Twist2D{}, &with_yaw);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::RotateToGoal);
  EXPECT_EQ(cmd.v(), 0.0);
  EXPECT_EQ(cmd.w(), 1.0);  // 目标朝向在左侧

  const SimpleGoalChecker xy_only{GoalCheckerConfig{}, false};
  cmd = ctrl.compute_velocity_commands(near_goal, Twist2D{}, &xy_only);
  EXPECT_EQ(ctrl.debug().regime, PursuitRegime::Approach);
  EXPECT_GT(cmd.v(), 0.0);
}

// U 形回折:车在去程上但离回程更近;回程在 1 m 搜索范围之外,进度留在去程。
TEST(PurePursuit, ProgressStaysOnOutboundLegOfHairpin) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  (void)command(ctrl, make_path({{0.0, 0.0}, {4.0, 0.0}, {4.0, 0.2}, {0.0, 0.2}}), Pose2D{1.0, 0.12, 0.0}, 0.3);
  EXPECT_NEAR(ctrl.debug().arclength, 1.0, kEps);
  EXPECT_NEAR(ctrl.debug().cross_track_error, 0.12, kEps);
}

// 进度单调:推进到第二段后,即使车回到第一段附近,投影也不回退;reset 后从头开始。
TEST(PurePursuit, ProgressIsMonotoneUntilReset) {
  PurePursuitController ctrl{PurePursuitConfig{}, kLimits};
  const Path path = make_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
  (void)command(ctrl, path, Pose2D{1.0, 0.5, std::numbers::pi / 2.0}, 0.3);
  EXPECT_NEAR(ctrl.debug().arclength, 1.5, kEps);

  (void)ctrl.compute_velocity_commands(Pose2D{0.5, 0.0, 0.0}, Twist2D{0.3, 0.0}, nullptr);
  EXPECT_NEAR(ctrl.debug().arclength, 1.0, kEps);  // 钳在第二段起点,不回到第一段

  ctrl.reset();
  (void)ctrl.compute_velocity_commands(Pose2D{0.5, 0.0, 0.0}, Twist2D{0.3, 0.0}, nullptr);
  EXPECT_NEAR(ctrl.debug().arclength, 0.5, kEps);
}

// ---------------------------------------------------------------------------
// 参数校验
// ---------------------------------------------------------------------------

TEST(PurePursuit, RejectsInvalidConfiguration) {
  PurePursuitConfig cfg{};
  cfg.desired_linear_vel = 0.6;  // > max_linear_vel
  EXPECT_THROW((PurePursuitController{cfg, kLimits}), std::invalid_argument);

  cfg = PurePursuitConfig{};
  cfg.min_lookahead = 0.7;  // > max_lookahead
  EXPECT_THROW((PurePursuitController{cfg, kLimits}), std::invalid_argument);

  cfg = PurePursuitConfig{};
  cfg.rotate_vel = 3.0;  // > max_angular_vel
  EXPECT_THROW((PurePursuitController{cfg, kLimits}), std::invalid_argument);

  cfg = PurePursuitConfig{};
  cfg.rotate_to_heading_angle = 4.0;  // > π
  EXPECT_THROW((PurePursuitController{cfg, kLimits}), std::invalid_argument);
}

TEST(PurePursuit, RegimeNamesAreStable) {
  using mininav::control::to_string;
  EXPECT_EQ(to_string(PursuitRegime::Stopped), "stopped");
  EXPECT_EQ(to_string(PursuitRegime::RotateToPath), "rotate_to_path");
  EXPECT_EQ(to_string(PursuitRegime::Track), "track");
  EXPECT_EQ(to_string(PursuitRegime::Approach), "approach");
  EXPECT_EQ(to_string(PursuitRegime::RotateToGoal), "rotate_to_goal");
}
