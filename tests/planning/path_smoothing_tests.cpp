import mininav.planning.path_smoothing;
import mininav.planning.astar;
import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;
import mininav.planning.map_io;
import mininav.planning.planner_config;
import mininav.core.types;
import mininav.core.path;

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using mininav::Path;
using mininav::Pose2D;
using mininav::planning::AStarPlanner;
using mininav::planning::GridCoord;
using mininav::planning::kFree;
using mininav::planning::kOccupied;
using mininav::planning::kUnknown;
using mininav::planning::OccupancyGrid;
using mininav::planning::PathSmoothingConfig;
using mininav::planning::PlannerConfig;
using mininav::planning::PlanResult;
using mininav::planning::segment_is_free;
using mininav::planning::smooth_path;

constexpr double kEps = 1e-12;

// resolution = 1,origin = (0, 0):cell (x, y) 覆盖 [x, x+1) × [y, y+1)。
OccupancyGrid make_grid(int w, int h, const std::vector<GridCoord>& occupied,
                        const std::vector<GridCoord>& unknown = {}) {
  std::vector<std::int8_t> data(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), kFree);
  const auto set = [&](const GridCoord c, const std::int8_t v) {
    data[static_cast<std::size_t>(c.y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(c.x)] = v;
  };
  for (const GridCoord c : occupied) {
    set(c, kOccupied);
  }
  for (const GridCoord c : unknown) {
    set(c, kUnknown);
  }
  return OccupancyGrid{w, h, 1.0, Eigen::Vector2d{0.0, 0.0}, std::move(data)};
}

// 与 astar_tests 相同的蛇形迷宫:每 3 行一道横墙,缺口左右交替。
OccupancyGrid serpentine_map(int n) {
  std::vector<GridCoord> walls;
  int wall = 0;
  for (int y = 2; y < n; y += 3, ++wall) {
    const int gap = (wall % 2 == 0) ? n - 1 : 0;
    for (int x = 0; x < n; ++x) {
      if (x != gap) {
        walls.push_back({x, y});
      }
    }
  }
  return make_grid(n, n, walls);
}

PlannerConfig planner_cfg(double cost_weight = 0.0) {
  PlannerConfig c{};
  c.inflation_radius = 0.0;
  c.cost_weight = cost_weight;
  return c;
}

bool free_segment(const OccupancyGrid& g, double ax, double ay, double bx, double by,
                  bool allow_unknown = false) {
  return segment_is_free(g, Eigen::Vector2d{ax, ay}, Eigen::Vector2d{bx, by}, allow_unknown);
}

// 平滑结果的不变量:端点精确、每段都可通行。
void expect_valid(const Path& path, const OccupancyGrid& costmap, const Pose2D& start, const Pose2D& goal) {
  ASSERT_GE(path.size(), 1u);
  EXPECT_NEAR(path.poses.front().x(), start.x(), kEps);
  EXPECT_NEAR(path.poses.front().y(), start.y(), kEps);
  EXPECT_NEAR(path.poses.back().x(), goal.x(), kEps);
  EXPECT_NEAR(path.poses.back().y(), goal.y(), kEps);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(segment_is_free(costmap, path.poses[i - 1].position(), path.poses[i].position(), false))
        << "segment " << i - 1 << " -> " << i;
  }
}

struct Smoothed {
  PlanResult raw;
  Path snapped;   // 只做首尾替换
  Path smoothed;  // 首尾替换 + 捷径
};

Smoothed plan_and_smooth(const OccupancyGrid& grid, const PlannerConfig& cfg, const Pose2D& start,
                         const Pose2D& goal) {
  const AStarPlanner planner{grid, cfg};
  Smoothed s;
  s.raw = planner.plan(start, goal);
  s.snapped = smooth_path(s.raw.path, planner.costmap(), planner.config(), start, goal,
                          PathSmoothingConfig{.snap_endpoints = true, .shortcut = false});
  s.smoothed = smooth_path(s.raw.path, planner.costmap(), planner.config(), start, goal, PathSmoothingConfig{});
  return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// segment_is_free:supercover 遍历
// ---------------------------------------------------------------------------

TEST(SegmentIsFree, DiagonalInFreeGridIsFree) {
  EXPECT_TRUE(free_segment(make_grid(5, 5, {}), 0.5, 0.5, 4.5, 4.5));
}

TEST(SegmentIsFree, SegmentThroughOccupiedCellIsBlocked) {
  const OccupancyGrid g = make_grid(5, 5, {{2, 2}});
  EXPECT_FALSE(free_segment(g, 0.5, 2.5, 4.5, 2.5));
  EXPECT_TRUE(free_segment(g, 0.5, 1.5, 4.5, 1.5));
}

// 恰好穿过格角 (1, 1):角上两个正交邻居都算被经过 —— 与 A* 的防穿角规则一致。
TEST(SegmentIsFree, CornerCrossingChecksBothOrthogonalNeighbours) {
  EXPECT_FALSE(free_segment(make_grid(3, 3, {{1, 0}}), 0.5, 0.5, 1.5, 1.5));
  EXPECT_FALSE(free_segment(make_grid(3, 3, {{0, 1}}), 0.5, 0.5, 1.5, 1.5));
  EXPECT_TRUE(free_segment(make_grid(3, 3, {{2, 0}}), 0.5, 0.5, 1.5, 1.5));
}

TEST(SegmentIsFree, PassingCloseWithoutTouchingIsFree) {
  const OccupancyGrid g = make_grid(5, 3, {{2, 1}});
  EXPECT_TRUE(free_segment(g, 0.5, 0.5, 4.5, 0.99));  // 一直在第 0 行
}

// 负方向、陡斜率:(3.5, 4.5) → (0.5, 0.5) 在 x ∈ [1, 2) 时 y ∈ (1.17, 2.5),经过 (1,1)、(1,2)。
TEST(SegmentIsFree, HandlesNegativeDirectionsAndSteepSlopes) {
  EXPECT_TRUE(free_segment(make_grid(5, 5, {{1, 3}}), 3.5, 4.5, 0.5, 0.5));
  EXPECT_FALSE(free_segment(make_grid(5, 5, {{1, 2}}), 3.5, 4.5, 0.5, 0.5));
}

TEST(SegmentIsFree, EndpointCellsAndOutOfBoundsAreChecked) {
  const OccupancyGrid g = make_grid(5, 5, {{4, 4}});
  EXPECT_FALSE(free_segment(g, 0.5, 0.5, 4.5, 4.5));  // 终点落在占据 cell
  EXPECT_FALSE(free_segment(g, 2.5, 2.5, 6.5, 2.5));  // 越界即不可通行
  EXPECT_FALSE(free_segment(g, -0.5, 2.5, 2.5, 2.5));
}

TEST(SegmentIsFree, UnknownCellsFollowAllowUnknown) {
  const OccupancyGrid g = make_grid(5, 5, {}, {{2, 2}});
  EXPECT_FALSE(free_segment(g, 0.5, 2.5, 4.5, 2.5, false));
  EXPECT_TRUE(free_segment(g, 0.5, 2.5, 4.5, 2.5, true));
}

TEST(SegmentIsFree, ZeroLengthSegmentChecksItsCell) {
  const OccupancyGrid g = make_grid(3, 3, {{1, 1}});
  EXPECT_TRUE(free_segment(g, 0.3, 0.3, 0.3, 0.3));
  EXPECT_FALSE(free_segment(g, 1.3, 1.3, 1.3, 1.3));
}

// ---------------------------------------------------------------------------
// smooth_path
// ---------------------------------------------------------------------------

TEST(PathSmoothing, EmptyMapCollapsesToOneSegment) {
  const OccupancyGrid grid = make_grid(20, 20, {});
  const Pose2D start{1.2, 1.7, 0.0};
  const Pose2D goal{17.9, 13.3, 0.0};
  const Smoothed s = plan_and_smooth(grid, planner_cfg(), start, goal);
  ASSERT_TRUE(s.raw.success);
  ASSERT_EQ(s.smoothed.size(), 2u);
  expect_valid(s.smoothed, grid, start, goal);
  EXPECT_NEAR(s.smoothed.length(), std::hypot(16.7, 11.6), 1e-9);
}

// 蛇形迷宫:必须折返穿过全部走廊。平滑后仍可行,且 waypoint 只剩折返处。
TEST(PathSmoothing, SerpentineMazeStaysFeasible) {
  const OccupancyGrid grid = serpentine_map(60);  // 墙在 y = 2, 5, …, 59;最后一条走廊是 57–58 行
  const Pose2D start{0.3, 0.8, 0.0};
  const Pose2D goal{59.6, 58.2, 0.0};
  const Smoothed s = plan_and_smooth(grid, planner_cfg(), start, goal);
  ASSERT_TRUE(s.raw.success);
  expect_valid(s.smoothed, grid, start, goal);
  EXPECT_LT(s.smoothed.size(), s.raw.path.size() / 4);
  EXPECT_LE(s.smoothed.length(), s.snapped.length() + kEps);
}

// 长度不增:捷径是用直线替换子折线(三角不等式)。首尾替换最多各多出半格对角线。
TEST(PathSmoothing, LengthNeverIncreases) {
  const OccupancyGrid grid = make_grid(30, 20, {{10, 0}, {10, 1}, {10, 2}, {10, 3}, {10, 4}, {10, 5},
                                                {10, 6}, {10, 7}, {10, 8}, {10, 9}, {10, 10}, {10, 11},
                                                {20, 19}, {20, 18}, {20, 17}, {20, 16}, {20, 15}, {20, 14},
                                                {20, 13}, {20, 12}, {20, 11}, {20, 10}, {20, 9}});
  const Pose2D start{1.1, 1.9, 0.0};
  const Pose2D goal{28.4, 2.6, 0.0};
  const Smoothed s = plan_and_smooth(grid, planner_cfg(), start, goal);
  ASSERT_TRUE(s.raw.success);
  expect_valid(s.smoothed, grid, start, goal);
  expect_valid(s.snapped, grid, start, goal);
  EXPECT_LE(s.smoothed.length(), s.snapped.length() + kEps);
  const double endpoint_slack = (start.position() - s.raw.path.poses.front().position()).norm() +
                                (goal.position() - s.raw.path.poses.back().position()).norm();
  EXPECT_LE(s.smoothed.length(), s.raw.path.length() + endpoint_slack + kEps);
  EXPECT_LT(s.smoothed.length(), s.raw.path.length());  // 台阶确实被拉直了
}

// 真实起点到下一个 waypoint 的线段擦到障碍时,保留格心作过渡点。
// 原始段 (0.5, 0.5) → (4.5, 2.5) 只经过 (0,0)、(1,0)、(1,1)、(2,1)、(3,1)、(3,2)、(4,2);
// 从 (0.95, 0.05) 出发则在 x = 2 处 y ≈ 0.78,经过占据的 (2, 0)。
TEST(PathSmoothing, KeepsCellCenterWhenTrueEndpointSegmentIsBlocked) {
  const OccupancyGrid grid = make_grid(5, 3, {{2, 0}});
  ASSERT_TRUE(free_segment(grid, 0.5, 0.5, 4.5, 2.5));
  ASSERT_FALSE(free_segment(grid, 0.95, 0.05, 4.5, 2.5));

  Path raw;
  raw.poses.emplace_back(0.5, 0.5, 0.0);
  raw.poses.emplace_back(4.5, 2.5, 0.0);
  const Pose2D start{0.95, 0.05, 0.0};
  const Pose2D goal{4.5, 2.5, 0.0};
  const Path out = smooth_path(raw, grid, planner_cfg(), start, goal, PathSmoothingConfig{});
  ASSERT_EQ(out.size(), 3u);
  EXPECT_NEAR(out.poses[1].x(), 0.5, kEps);
  EXPECT_NEAR(out.poses[1].y(), 0.5, kEps);
  expect_valid(out, grid, start, goal);
}

TEST(PathSmoothing, StartAndGoalInSameCell) {
  const OccupancyGrid grid = make_grid(5, 5, {});
  const Pose2D start{2.1, 2.2, 0.0};
  const Pose2D goal{2.8, 2.9, 0.0};
  const Smoothed s = plan_and_smooth(grid, planner_cfg(), start, goal);
  ASSERT_EQ(s.raw.path.size(), 1u);
  ASSERT_EQ(s.smoothed.size(), 2u);
  expect_valid(s.smoothed, grid, start, goal);
}

TEST(PathSmoothing, FailedPlanGivesEmptyPath) {
  const OccupancyGrid grid = make_grid(3, 3, {{1, 0}, {1, 1}, {1, 2}});
  const Smoothed s = plan_and_smooth(grid, planner_cfg(), Pose2D{0.5, 0.5, 0.0}, Pose2D{2.5, 2.5, 0.0});
  EXPECT_FALSE(s.raw.success);
  EXPECT_TRUE(s.smoothed.empty());
}

// cost_weight > 0:代价梯度让路径远离障碍,捷径会抵消它 —— 只做首尾替换。
TEST(PathSmoothing, CostWeightSkipsShortcut) {
  const OccupancyGrid grid = make_grid(20, 20, {{10, 5}, {10, 6}, {10, 7}});
  const Pose2D start{1.5, 6.5, 0.0};
  const Pose2D goal{18.5, 6.5, 0.0};
  const Smoothed s = plan_and_smooth(grid, planner_cfg(1.0), start, goal);
  ASSERT_TRUE(s.raw.success);
  EXPECT_EQ(s.smoothed.size(), s.snapped.size());
  expect_valid(s.smoothed, grid, start, goal);
}

TEST(PathSmoothing, ShortcutWithoutSnappingKeepsCellCenters) {
  const OccupancyGrid grid = make_grid(20, 20, {});
  const AStarPlanner planner{grid, planner_cfg()};
  const PlanResult raw = planner.plan(Pose2D{1.2, 1.7, 0.0}, Pose2D{17.9, 13.3, 0.0});
  const Path out = smooth_path(raw.path, planner.costmap(), planner.config(), Pose2D{1.2, 1.7, 0.0},
                               Pose2D{17.9, 13.3, 0.0},
                               PathSmoothingConfig{.snap_endpoints = false, .shortcut = true});
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out.poses.front().x(), 1.5, kEps);
  EXPECT_NEAR(out.poses.back().y(), 13.5, kEps);
}

TEST(PathSmoothing, YawPointsAlongEachSegment) {
  const OccupancyGrid grid = serpentine_map(30);  // 墙在 y = 2, 5, …, 29
  const Pose2D start{0.3, 0.8, 0.0};
  const Pose2D goal{29.6, 28.2, 0.0};
  const Path out = plan_and_smooth(grid, planner_cfg(), start, goal).smoothed;
  ASSERT_GE(out.size(), 3u);
  for (std::size_t i = 0; i + 1 < out.size(); ++i) {
    const Eigen::Vector2d d = out.poses[i + 1].position() - out.poses[i].position();
    EXPECT_NEAR(out.poses[i].yaw(), std::atan2(d.y(), d.x()), kEps);
  }
  EXPECT_EQ(out.poses.back().yaw(), out.poses[out.size() - 2].yaw());
}

// ---------------------------------------------------------------------------
// office500:平滑前后的路径长度与 waypoint 数(#78 的验收数字)
// ---------------------------------------------------------------------------

TEST(PathSmoothing, Office500FloorPlan) {
  const std::filesystem::path root{PROJECT_ROOT_DIR};
  const OccupancyGrid grid = mininav::planning::load_occupancy_grid((root / "maps/office500.yaml").string());
  const PlannerConfig cfg = mininav::planning::load_planner_config((root / "config/planner.yaml").string());
  const Pose2D start{1.175, 1.175, 0.0};
  const Pose2D goal{23.875, 23.875, 0.0};
  const Smoothed s = plan_and_smooth(grid, cfg, start, goal);
  ASSERT_TRUE(s.raw.success);

  const AStarPlanner planner{grid, cfg};
  expect_valid(s.smoothed, planner.costmap(), start, goal);
  EXPECT_LT(s.smoothed.length(), s.raw.path.length());
  EXPECT_LT(s.smoothed.size(), s.raw.path.size() / 5);

  RecordProperty("raw_waypoints", static_cast<int>(s.raw.path.size()));
  RecordProperty("raw_length_m", std::to_string(s.raw.path.length()));
  RecordProperty("smoothed_waypoints", static_cast<int>(s.smoothed.size()));
  RecordProperty("smoothed_length_m", std::to_string(s.smoothed.length()));
  std::cout << "[ office500 ] raw: " << s.raw.path.size() << " waypoints, " << s.raw.path.length()
            << " m; smoothed: " << s.smoothed.size() << " waypoints, " << s.smoothed.length() << " m\n";
}

// ---------------------------------------------------------------------------
// 配置解析
// ---------------------------------------------------------------------------

TEST(PathSmoothingConfig, ParsesStrictly) {
  using mininav::planning::parse_path_smoothing_config;
  const PathSmoothingConfig defaults = parse_path_smoothing_config("");
  EXPECT_TRUE(defaults.snap_endpoints);
  EXPECT_TRUE(defaults.shortcut);
  const PathSmoothingConfig off = parse_path_smoothing_config("snap_endpoints: false\nshortcut: false");
  EXPECT_FALSE(off.snap_endpoints);
  EXPECT_FALSE(off.shortcut);
  EXPECT_THROW((void)parse_path_smoothing_config("shortcuts: true"), std::runtime_error);
  EXPECT_THROW((void)parse_path_smoothing_config("shortcut: sometimes"), std::runtime_error);
}
