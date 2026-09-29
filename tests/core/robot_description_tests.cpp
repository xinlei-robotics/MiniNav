import mininav.core.robot_description;

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

constexpr double kEps = 1e-12;

using mininav::load_robot_description;
using mininav::parse_robot_description;
using mininav::RobotDescription;

const std::string kValidYaml = R"(
wheel_radius: 0.032
wheel_base: 0.150
ticks_per_rev: 1024
footprint:
  length: 0.22
  width: 0.16
limits:
  max_linear_vel: 0.5
  max_angular_vel: 2.0
actuator_time_constant: 0.10
)";

// 把 kValidYaml 中第一处 from 替换为 to。
[[nodiscard]] std::string with(const std::string& from, const std::string& to) {
  std::string s = kValidYaml;
  s.replace(s.find(from), from.size(), to);
  return s;
}

} // namespace

TEST(RobotDescription, ParsesAllFields) {
  const RobotDescription r = parse_robot_description(kValidYaml);
  EXPECT_NEAR(r.wheel_radius, 0.032, kEps);
  EXPECT_NEAR(r.wheel_base, 0.150, kEps);
  EXPECT_EQ(r.ticks_per_rev, 1024);
  EXPECT_NEAR(r.footprint.length, 0.22, kEps);
  EXPECT_NEAR(r.footprint.width, 0.16, kEps);
  EXPECT_NEAR(r.limits.max_linear_vel, 0.5, kEps);
  EXPECT_NEAR(r.limits.max_angular_vel, 2.0, kEps);
  EXPECT_NEAR(r.actuator_time_constant, 0.10, kEps);
}

TEST(RobotDescription, CircumscribedRadiusIsHalfDiagonal) {
  const RobotDescription r = parse_robot_description(kValidYaml);
  EXPECT_NEAR(r.footprint.circumscribed_radius(), std::sqrt(0.11 * 0.11 + 0.08 * 0.08), kEps);
}

// 与 V1–V3 的 kDistancePerTick 同一表达式:逐位相同,EKF 模式的 golden 才能不变。
TEST(RobotDescription, DistancePerTickMatchesLegacyExpressionBitForBit) {
  const RobotDescription r = parse_robot_description(kValidYaml);
  const double legacy = 2.0 * std::numbers::pi * 0.032 / static_cast<double>(1024);
  EXPECT_EQ(r.distance_per_tick(), legacy);
}

// 仓库里的 config/robot.yaml 是 sim 的默认描述;几何量必须与 V3 之前的常量一致。
TEST(RobotDescription, ShippedConfigMatchesLegacyGeometry) {
  const RobotDescription r =
      load_robot_description(std::string{PROJECT_ROOT_DIR} + "/config/robot.yaml");
  EXPECT_EQ(r.wheel_radius, 0.032);
  EXPECT_EQ(r.wheel_base, 0.150);
  EXPECT_EQ(r.ticks_per_rev, 1024);
  EXPECT_GT(r.footprint.circumscribed_radius(), 0.5 * r.wheel_base);
}

TEST(RobotDescription, ZeroActuatorTimeConstantMeansNoLag) {
  const RobotDescription r =
      parse_robot_description(with("actuator_time_constant: 0.10", "actuator_time_constant: 0"));
  EXPECT_EQ(r.actuator_time_constant, 0.0);
}

// ---------------------------------------------------------------------------
// 严格解析
// ---------------------------------------------------------------------------

TEST(RobotDescription, RejectsUnknownTopLevelKey) {
  EXPECT_THROW((void)parse_robot_description(kValidYaml + "wheel_radus: 0.03\n"),
               std::runtime_error);
}

TEST(RobotDescription, RejectsUnknownNestedKey) {
  EXPECT_THROW((void)parse_robot_description(with("  width: 0.16", "  width: 0.16\n  height: 0.1")),
               std::runtime_error);
  EXPECT_THROW(
      (void)parse_robot_description(with("  max_angular_vel: 2.0", "  max_angular_vel: 2.0\n  max_accel: 1.0")),
      std::runtime_error);
}

TEST(RobotDescription, RejectsMissingKey) {
  EXPECT_THROW((void)parse_robot_description(with("wheel_base: 0.150\n", "")), std::runtime_error);
  EXPECT_THROW((void)parse_robot_description(with("  length: 0.22\n", "")), std::runtime_error);
}

TEST(RobotDescription, RejectsNonPositiveValues) {
  EXPECT_THROW((void)parse_robot_description(with("wheel_radius: 0.032", "wheel_radius: 0")),
               std::runtime_error);
  EXPECT_THROW((void)parse_robot_description(with("wheel_base: 0.150", "wheel_base: -0.15")),
               std::runtime_error);
  EXPECT_THROW((void)parse_robot_description(with("ticks_per_rev: 1024", "ticks_per_rev: 0")),
               std::runtime_error);
  EXPECT_THROW((void)parse_robot_description(with("max_linear_vel: 0.5", "max_linear_vel: .nan")),
               std::runtime_error);
  EXPECT_THROW((void)parse_robot_description(
                   with("actuator_time_constant: 0.10", "actuator_time_constant: -0.1")),
               std::runtime_error);
}

TEST(RobotDescription, RejectsNonIntegerTicks) {
  EXPECT_THROW((void)parse_robot_description(with("ticks_per_rev: 1024", "ticks_per_rev: 1024.5")),
               std::runtime_error);
}

TEST(RobotDescription, RejectsNonMappingSection) {
  EXPECT_THROW((void)parse_robot_description(with("footprint:\n  length: 0.22\n  width: 0.16",
                                                   "footprint: 0.22")),
               std::runtime_error);
}

TEST(RobotDescription, MissingFileThrows) {
  EXPECT_THROW((void)load_robot_description("/nonexistent/robot.yaml"), std::runtime_error);
}
