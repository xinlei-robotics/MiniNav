import mininav.planning.inflation;
import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using mininav::planning::GridCoord;
using mininav::planning::inflate;
using mininav::planning::kFree;
using mininav::planning::kOccupied;
using mininav::planning::kUnknown;
using mininav::planning::obstacle_distance_cells;
using mininav::planning::OccupancyGrid;

constexpr double kEps = 1e-9;

// resolution=1.0 让 radius_m 直接等于 radius_cells,断言更直观。
OccupancyGrid make_grid(int w, int h, std::vector<std::int8_t> data) {
  return OccupancyGrid{w, h, 1.0, Eigen::Vector2d{0.0, 0.0}, std::move(data)};
}

std::size_t idx(int w, int x, int y) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
         static_cast<std::size_t>(x);
}

}  // namespace

// ---------------------------------------------------------------------------
// r=0 → costmap == 原图
// ---------------------------------------------------------------------------

TEST(Inflation, ZeroRadiusIsIdentity) {
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 2, 2)] = kOccupied;
  data[idx(5, 0, 4)] = kUnknown;
  const OccupancyGrid grid = make_grid(5, 5, data);

  const OccupancyGrid out = inflate(grid, 0.0);
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) {
      EXPECT_EQ(out.at({x, y}), grid.at({x, y})) << "mismatch at (" << x << "," << y << ")";
    }
  }
}

TEST(Inflation, NegativeRadiusIsIdentity) {
  std::vector<std::int8_t> data(9, kFree);
  data[idx(3, 1, 1)] = kOccupied;
  const OccupancyGrid grid = make_grid(3, 3, data);
  const OccupancyGrid out = inflate(grid, -0.5);
  EXPECT_EQ(out.at({1, 1}), kOccupied);
  EXPECT_EQ(out.at({0, 0}), kFree);
}

// ---------------------------------------------------------------------------
// 空地图膨胀仍空
// ---------------------------------------------------------------------------

TEST(Inflation, EmptyMapStaysEmpty) {
  const OccupancyGrid grid = make_grid(6, 6, std::vector<std::int8_t>(36, kFree));
  const OccupancyGrid out = inflate(grid, 5.0);  // 远大于地图
  for (int y = 0; y < 6; ++y) {
    for (int x = 0; x < 6; ++x) {
      EXPECT_EQ(out.at({x, y}), kFree);
    }
  }
}

// ---------------------------------------------------------------------------
// 膨胀正确性:半径内 free cell 被标占据,半径外不动
// ---------------------------------------------------------------------------

TEST(Inflation, MarksOrthogonalNeighborsAtRadiusOne) {
  // 中心单障碍。radius=1 cell:正交邻居(dist=1)被吃,对角(dist=√2≈1.414)不被吃。
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 2, 2)] = kOccupied;
  const OccupancyGrid out = inflate(make_grid(5, 5, data), 1.0);

  EXPECT_EQ(out.at({2, 2}), kOccupied);  // 障碍自身
  EXPECT_EQ(out.at({1, 2}), kOccupied);  // 正交
  EXPECT_EQ(out.at({3, 2}), kOccupied);
  EXPECT_EQ(out.at({2, 1}), kOccupied);
  EXPECT_EQ(out.at({2, 3}), kOccupied);
  EXPECT_EQ(out.at({1, 1}), kFree);      // 对角:√2 > 1 → 不吃
  EXPECT_EQ(out.at({0, 2}), kFree);      // dist=2 → 不吃
}

TEST(Inflation, MarksDiagonalNeighborsAtRadiusOnePointFive) {
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 2, 2)] = kOccupied;
  const OccupancyGrid out = inflate(make_grid(5, 5, data), 1.5);

  EXPECT_EQ(out.at({1, 1}), kOccupied);  // 对角 √2≈1.414 ≤ 1.5 → 吃
  EXPECT_EQ(out.at({3, 3}), kOccupied);
  EXPECT_EQ(out.at({0, 2}), kFree);      // dist=2 > 1.5 → 不吃
}

TEST(Inflation, ResolutionScalesRadius) {
  // resolution=0.5:radius_m=0.5 → radius_cells=1 → 只吃正交邻居。
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 2, 2)] = kOccupied;
  const OccupancyGrid grid{5, 5, 0.5, Eigen::Vector2d{0.0, 0.0}, std::move(data)};
  const OccupancyGrid out = inflate(grid, 0.5);
  EXPECT_EQ(out.at({1, 2}), kOccupied);  // 正交,0.5m=1cell
  EXPECT_EQ(out.at({1, 1}), kFree);      // 对角 √2*0.5≈0.707m > 0.5m
}

// ---------------------------------------------------------------------------
// unknown cell 也会被膨胀吞掉(保守),远处 unknown 保持原值
// ---------------------------------------------------------------------------

TEST(Inflation, SwallowsUnknownWithinRadiusButKeepsItOutside) {
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 0, 0)] = kOccupied;
  data[idx(5, 1, 0)] = kUnknown;  // 紧邻障碍(dist=1)
  data[idx(5, 4, 4)] = kUnknown;  // 远离障碍
  const OccupancyGrid out = inflate(make_grid(5, 5, data), 1.0);

  EXPECT_EQ(out.at({1, 0}), kOccupied);  // 半径内的 unknown → 占据
  EXPECT_EQ(out.at({4, 4}), kUnknown);   // 半径外的 unknown 原样保留
}

// ---------------------------------------------------------------------------
// 多源距离变换正确性
// ---------------------------------------------------------------------------

TEST(Inflation, ObstacleDistanceIsEuclideanToNearest) {
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 2, 2)] = kOccupied;  // 单源,居中
  const OccupancyGrid grid = make_grid(5, 5, data);
  const std::vector<double> dist = obstacle_distance_cells(grid);

  EXPECT_NEAR(dist[idx(5, 2, 2)], 0.0, kEps);            // 源自身
  EXPECT_NEAR(dist[idx(5, 1, 2)], 1.0, kEps);            // 正交
  EXPECT_NEAR(dist[idx(5, 2, 0)], 2.0, kEps);            // 同列两格
  EXPECT_NEAR(dist[idx(5, 1, 1)], std::sqrt(2.0), kEps); // 对角
  EXPECT_NEAR(dist[idx(5, 0, 0)], std::sqrt(8.0), kEps); // 角落 (2,2) 偏移
}

TEST(Inflation, ObstacleDistanceTakesNearestOfMultipleSources) {
  std::vector<std::int8_t> data(25, kFree);
  data[idx(5, 0, 0)] = kOccupied;
  data[idx(5, 4, 4)] = kOccupied;
  const std::vector<double> dist = obstacle_distance_cells(make_grid(5, 5, data));
  // (1,1) 距 (0,0) 为 √2、距 (4,4) 为 √18 → 取更近的 √2。
  EXPECT_NEAR(dist[idx(5, 1, 1)], std::sqrt(2.0), kEps);
  // (3,3) 对称,取距 (4,4) 的 √2。
  EXPECT_NEAR(dist[idx(5, 3, 3)], std::sqrt(2.0), kEps);
}

TEST(Inflation, ObstacleDistanceIsInfiniteWithNoObstacles) {
  const OccupancyGrid grid = make_grid(4, 4, std::vector<std::int8_t>(16, kFree));
  const std::vector<double> dist = obstacle_distance_cells(grid);
  for (double d : dist) {
    EXPECT_TRUE(std::isinf(d));
  }
}
