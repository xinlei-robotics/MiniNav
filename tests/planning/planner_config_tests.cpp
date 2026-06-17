import mininav.planning.planner_config;
import mininav.planning.grid_types;

#include <gtest/gtest.h>

#include <string>

namespace {

constexpr double kEps = 1e-12;

} // namespace

using mininav::planning::Connectivity;
using mininav::planning::Heuristic;
using mininav::planning::parse_planner_config;
using mininav::planning::PlannerConfig;
using mininav::planning::to_yaml;

// ---------------------------------------------------------------------------
// 解析最小 YAML(PR0 验收点:planning 库能解析一份 planner.yaml)
// ---------------------------------------------------------------------------

TEST(PlannerConfig, ParsesMinimalYaml) {
  const std::string yaml = R"(
inflation_radius: 0.25
heuristic: octile
connectivity: 8
allow_unknown: true
cost_weight: 1.5
)";
  const PlannerConfig cfg = parse_planner_config(yaml);
  EXPECT_NEAR(cfg.inflation_radius, 0.25, kEps);
  EXPECT_EQ(cfg.heuristic, Heuristic::Octile);
  EXPECT_EQ(cfg.connectivity, Connectivity::Eight);
  EXPECT_TRUE(cfg.allow_unknown);
  EXPECT_NEAR(cfg.cost_weight, 1.5, kEps);
}

TEST(PlannerConfig, MissingFieldsFallBackToDefaults) {
  const PlannerConfig cfg = parse_planner_config("inflation_radius: 0.5");
  const PlannerConfig def{};
  EXPECT_NEAR(cfg.inflation_radius, 0.5, kEps);
  EXPECT_EQ(cfg.heuristic, def.heuristic);
  EXPECT_EQ(cfg.connectivity, def.connectivity);
  EXPECT_EQ(cfg.allow_unknown, def.allow_unknown);
  EXPECT_NEAR(cfg.cost_weight, def.cost_weight, kEps);
}

// round-trip:序列化再解析,字段逐项保持 —— 证明 yaml-cpp 读写双向打通。
TEST(PlannerConfig, YamlRoundTripPreservesFields) {
  PlannerConfig cfg{};
  cfg.inflation_radius = 0.35;
  cfg.heuristic = Heuristic::Manhattan;
  cfg.connectivity = Connectivity::Four;
  cfg.allow_unknown = true;
  cfg.cost_weight = 2.0;

  const PlannerConfig parsed = parse_planner_config(to_yaml(cfg));
  EXPECT_NEAR(parsed.inflation_radius, cfg.inflation_radius, kEps);
  EXPECT_EQ(parsed.heuristic, cfg.heuristic);
  EXPECT_EQ(parsed.connectivity, cfg.connectivity);
  EXPECT_EQ(parsed.allow_unknown, cfg.allow_unknown);
  EXPECT_NEAR(parsed.cost_weight, cfg.cost_weight, kEps);
}

TEST(PlannerConfig, RejectsInvalidConnectivity) {
  EXPECT_THROW((void)parse_planner_config("connectivity: 6"), std::runtime_error);
}

TEST(PlannerConfig, RejectsUnknownHeuristic) {
  EXPECT_THROW((void)parse_planner_config("heuristic: diagonal"), std::runtime_error);
}
