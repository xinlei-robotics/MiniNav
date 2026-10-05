import mininav.control.controller_config;
import mininav.control.pure_pursuit;
import mininav.control.velocity_smoother;
import mininav.control.goal_checker;
import mininav.control.progress_checker;

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace {

using mininav::control::ControllerConfig;
using mininav::control::parse_controller_config;
using mininav::control::parse_goal_checker_config;
using mininav::control::parse_progress_checker_config;

constexpr double kEps = 1e-12;

}  // namespace

// 空段 = 全部默认;默认值即 docs/math/pure_pursuit.md §10 推导出的参数。
TEST(ControllerConfig, EmptySectionGivesDerivedDefaults) {
  const ControllerConfig cfg = parse_controller_config("");
  EXPECT_NEAR(cfg.control_frequency, 20.0, kEps);
  EXPECT_NEAR(cfg.pursuit.desired_linear_vel, 0.30, kEps);
  EXPECT_NEAR(cfg.pursuit.lookahead_time, 1.0, kEps);
  EXPECT_NEAR(cfg.pursuit.min_lookahead, 0.10, kEps);
  EXPECT_NEAR(cfg.pursuit.max_lookahead, 0.60, kEps);
  EXPECT_NEAR(cfg.pursuit.curvature_lookahead_dist, 0.40, kEps);
  EXPECT_NEAR(cfg.pursuit.regulated_min_radius, 0.60, kEps);
  EXPECT_NEAR(cfg.smoother.max_accel, 0.5, kEps);
  EXPECT_NEAR(cfg.smoother.max_angular_accel, 3.0, kEps);
  EXPECT_TRUE(cfg.pursuit.use_velocity_scaled_lookahead);
  EXPECT_TRUE(cfg.pursuit.use_curvature_regulation);
  EXPECT_TRUE(cfg.pursuit.use_rotate_to_heading);
}

TEST(ControllerConfig, ParsesFlatSectionIntoSubConfigs) {
  const ControllerConfig cfg = parse_controller_config(R"(
control_frequency: 10.0
desired_linear_vel: 0.25
use_velocity_scaled_lookahead: false
lookahead_dist: 0.35
use_curvature_regulation: false
approach_dist: 0.5
rotate_vel: 0.8
max_projection_search_dist: 2.0
max_accel: 0.4
max_angular_accel: 2.5
)");
  EXPECT_NEAR(cfg.control_frequency, 10.0, kEps);
  EXPECT_NEAR(cfg.pursuit.desired_linear_vel, 0.25, kEps);
  EXPECT_FALSE(cfg.pursuit.use_velocity_scaled_lookahead);
  EXPECT_NEAR(cfg.pursuit.lookahead_dist, 0.35, kEps);
  EXPECT_FALSE(cfg.pursuit.use_curvature_regulation);
  EXPECT_NEAR(cfg.pursuit.approach_dist, 0.5, kEps);
  EXPECT_NEAR(cfg.pursuit.rotate_vel, 0.8, kEps);
  EXPECT_NEAR(cfg.pursuit.max_projection_search_dist, 2.0, kEps);
  EXPECT_NEAR(cfg.smoother.max_accel, 0.4, kEps);
  EXPECT_NEAR(cfg.smoother.max_angular_accel, 2.5, kEps);
  EXPECT_NEAR(cfg.pursuit.lookahead_time, 1.0, kEps);  // 未写的字段保留默认
}

TEST(ControllerConfig, RejectsUnknownKey) {
  EXPECT_THROW((void)parse_controller_config("lookahed_time: 1.0"), std::runtime_error);
}

TEST(ControllerConfig, RejectsWrongType) {
  EXPECT_THROW((void)parse_controller_config("use_curvature_regulation: maybe"), std::runtime_error);
  EXPECT_THROW((void)parse_controller_config("lookahead_time: fast"), std::runtime_error);
}

TEST(ControllerConfig, RejectsInconsistentValues) {
  EXPECT_THROW((void)parse_controller_config("min_lookahead: 0.8"), std::runtime_error);  // > max
  EXPECT_THROW((void)parse_controller_config("control_frequency: 0"), std::runtime_error);
  EXPECT_THROW((void)parse_controller_config("min_approach_vel: 0.5"), std::runtime_error);  // > desired
  EXPECT_THROW((void)parse_controller_config("max_accel: -1"), std::runtime_error);
}

TEST(ControllerConfig, RejectsNonMapping) {
  EXPECT_THROW((void)parse_controller_config("- 1\n- 2"), std::runtime_error);
}

TEST(GoalCheckerConfig, ParsesAndValidates) {
  const auto cfg = parse_goal_checker_config("xy_tolerance: 0.08\nyaw_tolerance: 0.2");
  EXPECT_NEAR(cfg.xy_tolerance, 0.08, kEps);
  EXPECT_NEAR(cfg.yaw_tolerance, 0.2, kEps);
  EXPECT_NEAR(parse_goal_checker_config("").xy_tolerance, 0.05, kEps);
  EXPECT_THROW((void)parse_goal_checker_config("xy_tol: 0.1"), std::runtime_error);
  EXPECT_THROW((void)parse_goal_checker_config("yaw_tolerance: 0"), std::runtime_error);
}

TEST(ProgressCheckerConfig, ParsesAndValidates) {
  const auto cfg = parse_progress_checker_config("required_movement: 0.3\ntime_allowance: 5");
  EXPECT_NEAR(cfg.required_movement, 0.3, kEps);
  EXPECT_NEAR(cfg.time_allowance, 5.0, kEps);
  EXPECT_NEAR(parse_progress_checker_config("").time_allowance, 10.0, kEps);
  EXPECT_THROW((void)parse_progress_checker_config("timeout: 5"), std::runtime_error);
  EXPECT_THROW((void)parse_progress_checker_config("time_allowance: -1"), std::runtime_error);
}
