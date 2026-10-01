import mininav.planning.path_smoothing;
import mininav.planning.astar;
import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;
import mininav.planning.map_io;
import mininav.planning.inflation;
import mininav.core.types;
import mininav.core.path;

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <queue>
#include <string>
#include <utility>
#include <vector>

// ===========================================================================
// maps/apartment:真实尺度的平面图(scripts/v4/gen_floorplan.py 从
// maps/src/apartment.toml 生成)。V4 的闭环场景要求它在按车体膨胀后仍连通:
// r_infl = 0.25 m ≥ 外接圆半径 0.136 m + 离散化与控制余量(docs/v4_plan.md §3.5)。
// ===========================================================================

namespace {

using mininav::Path;
using mininav::Pose2D;
using mininav::planning::AStarPlanner;
using mininav::planning::GridCoord;
using mininav::planning::OccupancyGrid;
using mininav::planning::PathSmoothingConfig;
using mininav::planning::PlannerConfig;
using mininav::planning::PlanResult;

constexpr double kFootprintInflation = 0.25;

OccupancyGrid load_apartment() {
  const std::filesystem::path root{PROJECT_ROOT_DIR};
  return mininav::planning::load_occupancy_grid((root / "maps/apartment.yaml").string());
}

// free cell 的 4-连通分量数。A* 是 8 连通但防穿角(对角移动要求两个正交邻居可走),
// 因此可达性与 4-连通等价。
std::size_t free_components(const OccupancyGrid& g) {
  const int w = g.width();
  const int h = g.height();
  std::vector<bool> seen(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), false);
  const auto idx = [w](int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
  };
  std::size_t components = 0;
  for (int sy = 0; sy < h; ++sy) {
    for (int sx = 0; sx < w; ++sx) {
      if (seen[idx(sx, sy)] || !g.is_free(GridCoord{sx, sy})) {
        continue;
      }
      ++components;
      std::queue<GridCoord> q;
      q.push({sx, sy});
      seen[idx(sx, sy)] = true;
      while (!q.empty()) {
        const GridCoord c = q.front();
        q.pop();
        for (const auto& [dx, dy] : {std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
          const GridCoord n{c.x + dx, c.y + dy};
          if (g.is_free(n) && !seen[idx(n.x, n.y)]) {
            seen[idx(n.x, n.y)] = true;
            q.push(n);
          }
        }
      }
    }
  }
  return components;
}

// 各房间里的代表点(world 米)。到墙与家具都留 ≥ 0.5 m,膨胀后仍可走。
struct Place {
  const char* name;
  Eigen::Vector2d xy;
};
const std::vector<Place> kPlaces{
    {"hallway west", {0.6, 3.6}},  {"hallway east", {9.4, 3.6}}, {"living", {2.9, 1.6}},
    {"kitchen", {9.25, 1.45}},     {"bedroom 1", {2.9, 5.6}},    {"bathroom", {4.9, 5.4}},
    {"bedroom 2", {7.2, 5.6}},
};

}  // namespace

TEST(ApartmentMap, IsRealScale) {
  const OccupancyGrid g = load_apartment();
  EXPECT_EQ(g.resolution(), 0.05);
  EXPECT_NEAR(g.width() * g.resolution(), 10.0, 1e-9);
  EXPECT_NEAR(g.height() * g.resolution(), 7.0, 1e-9);
}

TEST(ApartmentMap, FreeSpaceStaysConnectedAtFootprintInflation) {
  const OccupancyGrid g = load_apartment();
  EXPECT_EQ(free_components(g), 1u);
  EXPECT_EQ(free_components(mininav::planning::inflate(g, kFootprintInflation)), 1u);
}

// 每个房间都能从走廊西端到达,平滑后每段仍可通行、端点精确。
TEST(ApartmentMap, EveryRoomIsReachableAndSmoothable) {
  const OccupancyGrid g = load_apartment();
  PlannerConfig cfg{};
  cfg.inflation_radius = kFootprintInflation;
  const AStarPlanner planner{g, cfg};
  const Pose2D start{kPlaces.front().xy, 0.0};
  for (const Place& place : kPlaces) {
    const Pose2D goal{place.xy, 0.0};
    const PlanResult r = planner.plan(start, goal);
    ASSERT_TRUE(r.success) << place.name;
    const Path smoothed = mininav::planning::smooth_path(r.path, planner.costmap(), planner.config(), start,
                                                         goal, PathSmoothingConfig{});
    ASSERT_GE(smoothed.size(), 1u) << place.name;
    EXPECT_EQ(smoothed.poses.back().position(), goal.position()) << place.name;
    EXPECT_LE(smoothed.length(), r.path.length() + 0.1) << place.name;
    for (std::size_t i = 1; i < smoothed.size(); ++i) {
      EXPECT_TRUE(mininav::planning::segment_is_free(planner.costmap(), smoothed.poses[i - 1].position(),
                                                     smoothed.poses[i].position(), false))
          << place.name << " segment " << i;
    }
  }
}

// 手画的 V3 演示图是"玩具屋"尺度:膨胀 0.10 m(还不到车体外接圆半径 0.136 m)时,
// office 的门就被堵死、分成两块,maze 已没有 free cell(docs/v4_plan.md §5.4)。
// 这里锁住这一事实,说明 V4 为什么需要新地图。
TEST(ApartmentMap, HandDrawnDemoMapsDoNotSurviveFootprintInflation) {
  const std::filesystem::path maps{std::filesystem::path{PROJECT_ROOT_DIR} / "maps"};
  const OccupancyGrid office = mininav::planning::load_occupancy_grid((maps / "office.yaml").string());
  const OccupancyGrid maze = mininav::planning::load_occupancy_grid((maps / "maze.yaml").string());
  EXPECT_EQ(free_components(office), 1u);
  EXPECT_EQ(free_components(mininav::planning::inflate(office, 0.10)), 2u);
  EXPECT_EQ(free_components(mininav::planning::inflate(maze, 0.10)), 0u);
}
