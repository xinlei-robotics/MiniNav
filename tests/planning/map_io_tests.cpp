import mininav.planning.map_io;
import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;

using mininav::planning::GridCoord;
using mininav::planning::kFree;
using mininav::planning::kOccupied;
using mininav::planning::kUnknown;
using mininav::planning::load_occupancy_grid;
using mininav::planning::OccupancyGrid;

constexpr double kEps = 1e-12;

fs::path maps_dir() { return fs::path{PROJECT_ROOT_DIR} / "maps"; }

// 每个错误用例独占一个临时目录,避免并行 ctest 进程相互踩文件。
fs::path make_unique_dir() {
  std::random_device rd;
  const fs::path d = fs::temp_directory_path() / ("mininav_mapio_" + std::to_string(rd()));
  fs::create_directories(d);
  return d;
}

void write_file(const fs::path& p, const std::string& content) {
  std::ofstream out{p, std::ios::binary};
  out << content;
}

}  // namespace

// ---------------------------------------------------------------------------
// 加载手画 corridor 地图:逐 cell 断言内容(含 y 轴翻转验证)
// ---------------------------------------------------------------------------

TEST(MapIo, LoadsCorridorWithCorrectDimsAndMetadata) {
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "corridor.yaml").string());
  EXPECT_EQ(grid.width(), 7);
  EXPECT_EQ(grid.height(), 5);
  EXPECT_NEAR(grid.resolution(), 0.10, kEps);
  EXPECT_NEAR(grid.origin().x(), -1.0, kEps);
  EXPECT_NEAR(grid.origin().y(), -2.0, kEps);
}

TEST(MapIo, CorridorWallsAndInteriorClassifyCorrectly) {
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "corridor.yaml").string());
  // 图像最底行(grid y=0)与最顶行(grid y=4)是墙。
  EXPECT_EQ(grid.at({0, 0}), kOccupied);
  EXPECT_EQ(grid.at({6, 0}), kOccupied);
  EXPECT_EQ(grid.at({3, 4}), kOccupied);
  // 走廊内部为 free。
  EXPECT_EQ(grid.at({0, 2}), kFree);
  EXPECT_TRUE(grid.is_free({0, 2}));
}

TEST(MapIo, CorridorYAxisIsFlippedOnLoad) {
  // 柱子(占据)与 unknown 格子在图像里放在上半部(img row=1)。
  // y 翻转后应出现在 grid 高 y(=3),而非低 y。
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "corridor.yaml").string());
  EXPECT_EQ(grid.at({3, 3}), kOccupied);  // 柱子,上半部 → 高 grid y
  EXPECT_EQ(grid.at({3, 1}), kFree);      // 同列下半部仍空 → 证明确实翻转
  EXPECT_EQ(grid.at({5, 3}), kUnknown);   // 灰(128)→ 介于阈值之间
}

TEST(MapIo, CorridorOriginAndResolutionDriveTransforms) {
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "corridor.yaml").string());
  // cell (0,0) 中心 = origin + (0.5,0.5)*res。
  const auto c00 = grid.grid_to_world({0, 0});
  EXPECT_NEAR(c00.x(), -0.95, kEps);
  EXPECT_NEAR(c00.y(), -1.95, kEps);
}

// ---------------------------------------------------------------------------
// room / maze:加载冒烟(主要服务 PR3/PR4)
// ---------------------------------------------------------------------------

TEST(MapIo, LoadsRoomSmoke) {
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "room.yaml").string());
  EXPECT_EQ(grid.width(), 10);
  EXPECT_EQ(grid.height(), 8);
  EXPECT_EQ(grid.at({0, 0}), kOccupied);  // 角落是墙
  EXPECT_EQ(grid.at({0, 7}), kOccupied);
  EXPECT_TRUE(grid.is_free({5, 4}));      // 内部为空
}

TEST(MapIo, LoadsMazeSmoke) {
  const OccupancyGrid grid = load_occupancy_grid((maps_dir() / "maze.yaml").string());
  EXPECT_EQ(grid.width(), 11);
  EXPECT_EQ(grid.height(), 11);
  EXPECT_EQ(grid.at({0, 0}), kOccupied);  // 外墙
  EXPECT_TRUE(grid.is_free({1, 1}));      // 通路起点
}

// ---------------------------------------------------------------------------
// negate:反色时明暗占据语义翻转
// ---------------------------------------------------------------------------

TEST(MapIo, NegateInvertsOccupancy) {
  const fs::path dir = make_unique_dir();
  write_file(dir / "n.pgm", "P2\n2 1\n255\n0 255\n");
  write_file(dir / "n.yaml",
             "image: n.pgm\nresolution: 0.1\norigin: [0,0,0]\n"
             "occupied_thresh: 0.65\nfree_thresh: 0.25\nnegate: 1\n");

  const OccupancyGrid grid = load_occupancy_grid((dir / "n.yaml").string());
  // negate=1:occ = pixel/maxval。pixel 0 → occ 0 → free;pixel 255 → occ 1 → occupied。
  EXPECT_EQ(grid.at({0, 0}), kFree);
  EXPECT_EQ(grid.at({1, 0}), kOccupied);
  fs::remove_all(dir);
}

// ---------------------------------------------------------------------------
// 错误处理:文件缺失 / 字段缺失 / PGM 损坏 → 清晰的 runtime_error
// ---------------------------------------------------------------------------

TEST(MapIo, MissingYamlFileThrows) {
  const fs::path dir = make_unique_dir();
  EXPECT_THROW((void)load_occupancy_grid((dir / "nope.yaml").string()), std::runtime_error);
  fs::remove_all(dir);
}

TEST(MapIo, MissingRequiredKeyThrows) {
  const fs::path dir = make_unique_dir();
  write_file(dir / "m.yaml", "resolution: 0.1\norigin: [0,0,0]\n");  // 缺 image
  EXPECT_THROW((void)load_occupancy_grid((dir / "m.yaml").string()), std::runtime_error);
  fs::remove_all(dir);
}

TEST(MapIo, MissingImageFileThrows) {
  const fs::path dir = make_unique_dir();
  write_file(dir / "g.yaml", "image: ghost.pgm\nresolution: 0.1\norigin: [0,0,0]\n");
  EXPECT_THROW((void)load_occupancy_grid((dir / "g.yaml").string()), std::runtime_error);
  fs::remove_all(dir);
}

TEST(MapIo, BadPgmMagicThrows) {
  const fs::path dir = make_unique_dir();
  write_file(dir / "b.pgm", "P9\n2 2\n255\n0 0 0 0\n");
  write_file(dir / "b.yaml", "image: b.pgm\nresolution: 0.1\norigin: [0,0,0]\n");
  EXPECT_THROW((void)load_occupancy_grid((dir / "b.yaml").string()), std::runtime_error);
  fs::remove_all(dir);
}

TEST(MapIo, PixelCountMismatchThrows) {
  const fs::path dir = make_unique_dir();
  write_file(dir / "s.pgm", "P2\n3 3\n255\n0 0 0\n");  // 声明 9 像素,只给 3
  write_file(dir / "s.yaml", "image: s.pgm\nresolution: 0.1\norigin: [0,0,0]\n");
  EXPECT_THROW((void)load_occupancy_grid((dir / "s.yaml").string()), std::runtime_error);
  fs::remove_all(dir);
}
