import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {

using mininav::planning::GridCoord;
using mininav::planning::kFree;
using mininav::planning::kOccupied;
using mininav::planning::kUnknown;
using mininav::planning::OccupancyGrid;

constexpr double kEps = 1e-12;

// 全 free 的 w×h 栅格,便于坐标变换类测试只关心几何。
OccupancyGrid make_free_grid(int w, int h, double res, Eigen::Vector2d origin) {
  std::vector<std::int8_t> data(static_cast<std::size_t>(w) * static_cast<std::size_t>(h),
                                kFree);
  return OccupancyGrid{w, h, res, std::move(origin), std::move(data)};
}

} // namespace

// ---------------------------------------------------------------------------
// 构造与访问器
// ---------------------------------------------------------------------------

TEST(OccupancyGrid, AccessorsReflectConstructorArgs) {
  const OccupancyGrid grid = make_free_grid(4, 3, 0.05, {-1.0, 2.0});
  EXPECT_EQ(grid.width(), 4);
  EXPECT_EQ(grid.height(), 3);
  EXPECT_NEAR(grid.resolution(), 0.05, kEps);
  EXPECT_NEAR(grid.origin().x(), -1.0, kEps);
  EXPECT_NEAR(grid.origin().y(), 2.0, kEps);
}

TEST(OccupancyGrid, ConstructorRejectsNonPositiveDimensions) {
  // 维度在 data-size 之前校验,故空 data 即可触发维度错误。
  EXPECT_THROW((OccupancyGrid{0, 3, 0.05, {0.0, 0.0}, {}}), std::invalid_argument);
  EXPECT_THROW((OccupancyGrid{3, -1, 0.05, {0.0, 0.0}, {}}), std::invalid_argument);
}

TEST(OccupancyGrid, ConstructorRejectsNonPositiveResolution) {
  EXPECT_THROW(make_free_grid(3, 3, 0.0, {0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(make_free_grid(3, 3, -0.05, {0.0, 0.0}), std::invalid_argument);
}

TEST(OccupancyGrid, ConstructorRejectsDataSizeMismatch) {
  std::vector<std::int8_t> too_small(5, kFree); // 需要 3*3=9
  EXPECT_THROW((OccupancyGrid{3, 3, 0.05, {0.0, 0.0}, std::move(too_small)}),
               std::invalid_argument);
}

// ---------------------------------------------------------------------------
// 坐标变换:往返一致 + floor 语义
// ---------------------------------------------------------------------------

TEST(OccupancyGrid, GridToWorldReturnsCellCenter) {
  // origin 在左下角:cell (0,0) 中心 = origin + (0.5, 0.5)*res。
  const OccupancyGrid grid = make_free_grid(10, 10, 0.5, {0.0, 0.0});
  const Eigen::Vector2d c00 = grid.grid_to_world({0, 0});
  EXPECT_NEAR(c00.x(), 0.25, kEps);
  EXPECT_NEAR(c00.y(), 0.25, kEps);

  const Eigen::Vector2d c21 = grid.grid_to_world({2, 1});
  EXPECT_NEAR(c21.x(), 1.25, kEps); // 0 + (2+0.5)*0.5
  EXPECT_NEAR(c21.y(), 0.75, kEps); // 0 + (1+0.5)*0.5
}

TEST(OccupancyGrid, RoundTripCellCenterIsStable) {
  // 含非零 origin 与非平凡 resolution,覆盖完整变换链。
  const OccupancyGrid grid = make_free_grid(20, 15, 0.05, {-1.3, 2.7});
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridCoord c{x, y};
      const GridCoord rt = grid.world_to_grid(grid.grid_to_world(c));
      EXPECT_EQ(rt, c) << "round-trip failed at (" << x << "," << y << ")";
    }
  }
}

TEST(OccupancyGrid, WorldToGridFloorsIntoOwningCell) {
  const OccupancyGrid grid = make_free_grid(10, 10, 1.0, {0.0, 0.0});
  // cell (3,2) 覆盖 world x∈[3,4), y∈[2,3)。
  EXPECT_EQ(grid.world_to_grid({3.0, 2.0}), (GridCoord{3, 2}));  // 左下角点
  EXPECT_EQ(grid.world_to_grid({3.99, 2.99}), (GridCoord{3, 2})); // 右上角内侧
  EXPECT_EQ(grid.world_to_grid({4.0, 3.0}), (GridCoord{4, 3}));   // 跨到下一格
}

TEST(OccupancyGrid, WorldToGridHandlesNegativeWorldCoords) {
  const OccupancyGrid grid = make_free_grid(10, 10, 1.0, {-5.0, -5.0});
  // origin=-5:world -5 → cell 0;world -4.5 仍在 cell 0;world -4 → cell 1。
  EXPECT_EQ(grid.world_to_grid({-5.0, -5.0}), (GridCoord{0, 0}));
  EXPECT_EQ(grid.world_to_grid({-4.5, -4.5}), (GridCoord{0, 0}));
  EXPECT_EQ(grid.world_to_grid({-4.0, -4.0}), (GridCoord{1, 1}));
}

// ---------------------------------------------------------------------------
// 边界判断 + 越界视为占据
// ---------------------------------------------------------------------------

TEST(OccupancyGrid, InBoundsCoversCornersAndRejectsOutside) {
  const OccupancyGrid grid = make_free_grid(5, 4, 1.0, {0.0, 0.0});
  EXPECT_TRUE(grid.in_bounds({0, 0}));
  EXPECT_TRUE(grid.in_bounds({4, 3})); // (width-1, height-1)
  EXPECT_FALSE(grid.in_bounds({-1, 0}));
  EXPECT_FALSE(grid.in_bounds({0, -1}));
  EXPECT_FALSE(grid.in_bounds({5, 0}));  // == width
  EXPECT_FALSE(grid.in_bounds({0, 4}));  // == height
}

TEST(OccupancyGrid, OutOfBoundsReadsAsOccupied) {
  const OccupancyGrid grid = make_free_grid(5, 5, 1.0, {0.0, 0.0});
  EXPECT_EQ(grid.at({-1, 0}), kOccupied);
  EXPECT_EQ(grid.at({5, 5}), kOccupied);
  EXPECT_FALSE(grid.is_free({-1, 0})); // 越界既非 free
}

// ---------------------------------------------------------------------------
// 占据查询 + is_free 对 unknown 的处理(手搭 5×5 地图)
// ---------------------------------------------------------------------------

TEST(OccupancyGrid, OccupancyQueryOnHandBuiltMap) {
  // 5×5,row-major(row 0 = 最小 world y)。布置:
  //   (x=1,y=0) occupied、(x=3,y=2) occupied、(x=0,y=4) unknown,其余 free。
  std::vector<std::int8_t> data(25, kFree);
  const auto idx = [](int x, int y) { return static_cast<std::size_t>(y) * 5 + x; };
  data[idx(1, 0)] = kOccupied;
  data[idx(3, 2)] = kOccupied;
  data[idx(0, 4)] = kUnknown;

  const OccupancyGrid grid{5, 5, 0.1, {0.0, 0.0}, std::move(data)};

  EXPECT_EQ(grid.at({1, 0}), kOccupied);
  EXPECT_EQ(grid.at({3, 2}), kOccupied);
  EXPECT_EQ(grid.at({0, 4}), kUnknown);
  EXPECT_EQ(grid.at({2, 2}), kFree);

  // is_free 仅对真正的 free 为真:occupied / unknown 皆为假。
  EXPECT_TRUE(grid.is_free({2, 2}));
  EXPECT_FALSE(grid.is_free({1, 0})); // occupied
  EXPECT_FALSE(grid.is_free({0, 4})); // unknown
}
