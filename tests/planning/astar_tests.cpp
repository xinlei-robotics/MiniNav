import mininav.planning.astar;
import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;
import mininav.core.types;

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace {

using mininav::Pose2D;
using mininav::planning::AStarPlanner;
using mininav::planning::Connectivity;
using mininav::planning::GridCoord;
using mininav::planning::Heuristic;
using mininav::planning::kFree;
using mininav::planning::kOccupied;
using mininav::planning::OccupancyGrid;
using mininav::planning::PlannerConfig;
using mininav::planning::PlanResult;

constexpr double kEps = 1e-9;
constexpr double kSqrt2 = std::numbers::sqrt2;

// resolution=1.0,origin=(0,0):cell (x,y) 中心在 world (x+0.5, y+0.5),
// 故 length 直接以 cell 为单位读。
OccupancyGrid make_grid(int w, int h, const std::vector<GridCoord>& occupied) {
  std::vector<std::int8_t> data(static_cast<std::size_t>(w) * static_cast<std::size_t>(h),
                                kFree);
  for (const GridCoord c : occupied) {
    data[static_cast<std::size_t>(c.y) * static_cast<std::size_t>(w) +
         static_cast<std::size_t>(c.x)] = kOccupied;
  }
  return OccupancyGrid{w, h, 1.0, Eigen::Vector2d{0.0, 0.0}, std::move(data)};
}

// cell 中心的 world 位姿,作为 plan 的 start/goal。
Pose2D at_cell(int x, int y) {
  return Pose2D{static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5, 0.0};
}

PlannerConfig cfg(Connectivity conn, Heuristic h, double cost_weight = 0.0) {
  PlannerConfig c{};
  c.connectivity = conn;
  c.heuristic = h;
  c.inflation_radius = 0.0;
  c.cost_weight = cost_weight;
  c.allow_unknown = false;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// 基本最优长度(空地图)
// ---------------------------------------------------------------------------

TEST(AStar, StraightDiagonalOnEmptyMapEightConnected) {
  const AStarPlanner planner{make_grid(5, 5, {}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(4, 4));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), 4.0 * kSqrt2, kEps);  // 4 个对角步
  EXPECT_GT(r.expanded_nodes, 0u);
}

TEST(AStar, ManhattanLengthOnEmptyMapFourConnected) {
  const AStarPlanner planner{make_grid(5, 5, {}),
                             cfg(Connectivity::Four, Heuristic::Manhattan)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(3, 2));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), 5.0, kEps);  // |3|+|2| 步
}

// ---------------------------------------------------------------------------
// 最优性:绕墙的已知最短路(手算 = 12)
// ---------------------------------------------------------------------------

OccupancyGrid wall_map() {
  // 5x5,x=2 列在 y=0..3 是墙,仅 (2,4) 可通过。
  return make_grid(5, 5, {{2, 0}, {2, 1}, {2, 2}, {2, 3}});
}

TEST(AStar, FindsKnownShortestPathAroundWall) {
  const AStarPlanner planner{wall_map(), cfg(Connectivity::Four, Heuristic::Manhattan)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(4, 0));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), 12.0, kEps);
}

// 启发式 admissible:4 连通下 Manhattan / Euclidean / Octile 均 admissible,
// 给出相同最优长度(只是展开数可能不同)。
TEST(AStar, AdmissibleHeuristicsAgreeOnOptimalLength) {
  double len[3] = {0, 0, 0};
  const Heuristic hs[3] = {Heuristic::Manhattan, Heuristic::Euclidean, Heuristic::Octile};
  for (int i = 0; i < 3; ++i) {
    const AStarPlanner planner{wall_map(), cfg(Connectivity::Four, hs[i])};
    const PlanResult r = planner.plan(at_cell(0, 0), at_cell(4, 0));
    ASSERT_TRUE(r.success);
    len[i] = r.path.length();
  }
  EXPECT_NEAR(len[0], 12.0, kEps);
  EXPECT_NEAR(len[1], len[0], kEps);
  EXPECT_NEAR(len[2], len[0], kEps);
}

// 8 连通下只有 Euclidean / Octile admissible,二者同样给出相同最优长度。
// 绕墙最短 = 8 + 2√2:(0,0)→(1,4) 与 (3,4)→(4,0) 各为 3 直 + 1 斜;门洞 (2,4)
// 两侧的斜穿被防穿角规则禁止((2,3) 是墙),只能走 (1,4)→(2,4)→(3,4) 两个正交步。
TEST(AStar, AdmissibleHeuristicsAgreeOnOptimalLengthEightConnected) {
  for (const Heuristic h : {Heuristic::Euclidean, Heuristic::Octile}) {
    const AStarPlanner planner{wall_map(), cfg(Connectivity::Eight, h)};
    const PlanResult r = planner.plan(at_cell(0, 0), at_cell(4, 0));
    ASSERT_TRUE(r.success);
    EXPECT_NEAR(r.path.length(), 8.0 + 2.0 * kSqrt2, kEps);
  }
}

// Manhattan 在 8 连通下高估对角步(记 2,实为 √2),不 admissible —— 最优性
// 保证会被悄悄打破,所以构造期直接拒绝,而不是返回一条次优路径。
TEST(AStar, RejectsInadmissibleManhattanUnderEightConnectivity) {
  EXPECT_THROW((AStarPlanner{make_grid(5, 5, {}),
                             cfg(Connectivity::Eight, Heuristic::Manhattan)}),
               std::invalid_argument);
}

// ---------------------------------------------------------------------------
// 不可达 / 退化 / 非法起止点
// ---------------------------------------------------------------------------

TEST(AStar, UnreachableGoalReturnsNoPath) {
  // 目标 (2,2) 被四面墙围死(4 连通)。
  const AStarPlanner planner{make_grid(5, 5, {{1, 2}, {3, 2}, {2, 1}, {2, 3}}),
                             cfg(Connectivity::Four, Heuristic::Manhattan)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(2, 2));
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_GT(r.expanded_nodes, 0u);  // 搜过其余区域,不是死循环
}

TEST(AStar, StartEqualsGoalReturnsSinglePoint) {
  const AStarPlanner planner{make_grid(5, 5, {}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(2, 2), at_cell(2, 2));
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.poses.size(), 1u);
  EXPECT_NEAR(r.path.length(), 0.0, kEps);
}

TEST(AStar, StartOrGoalOnObstacleFails) {
  const OccupancyGrid grid = make_grid(5, 5, {{2, 2}});
  const AStarPlanner planner{grid, cfg(Connectivity::Eight, Heuristic::Octile)};
  EXPECT_FALSE(planner.plan(at_cell(2, 2), at_cell(4, 4)).success);  // start 在障碍
  EXPECT_FALSE(planner.plan(at_cell(0, 0), at_cell(2, 2)).success);  // goal 在障碍
}

TEST(AStar, OutOfBoundsStartFails) {
  const AStarPlanner planner{make_grid(5, 5, {}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(Pose2D{-100.0, -100.0, 0.0}, at_cell(4, 4));
  EXPECT_FALSE(r.success);
}

// ---------------------------------------------------------------------------
// 8 连通防穿角
// ---------------------------------------------------------------------------

TEST(AStar, DiagonalBlockedAtObstacleCorner) {
  // (1,0) 是障碍:(0,0)→(1,1) 的对角穿越被禁,必须绕 (0,1)。
  const AStarPlanner planner{make_grid(3, 3, {{1, 0}}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(1, 1));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), 2.0, kEps);  // 两个正交步,而非 √2 穿角
}

TEST(AStar, DiagonalAllowedWhenBothCornersFree) {
  const AStarPlanner planner{make_grid(3, 3, {}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(1, 1));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), kSqrt2, kEps);  // 允许对角
}

// ---------------------------------------------------------------------------
// tie-break 确定性(A* 无 RNG → 同输入逐点一致)
// ---------------------------------------------------------------------------

TEST(AStar, DeterministicAcrossRuns) {
  const AStarPlanner planner{wall_map(), cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult a = planner.plan(at_cell(0, 0), at_cell(4, 0));
  const PlanResult b = planner.plan(at_cell(0, 0), at_cell(4, 0));
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  ASSERT_EQ(a.path.poses.size(), b.path.poses.size());
  for (std::size_t i = 0; i < a.path.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.path.poses[i].x(), b.path.poses[i].x());
    EXPECT_DOUBLE_EQ(a.path.poses[i].y(), b.path.poses[i].y());
  }
}

// ---------------------------------------------------------------------------
// 代价梯度:cost_weight>0 时路径在等长下走中线、远离障碍
// ---------------------------------------------------------------------------

TEST(AStar, CostGradientPrefersClearanceAtEqualLength) {
  // 3 格宽走廊:y=0、y=4 是墙,y=1/2/3 为空。中线 y=2 离两墙最远。
  std::vector<GridCoord> walls;
  for (int x = 0; x < 7; ++x) {
    walls.push_back({x, 0});
    walls.push_back({x, 4});
  }
  const OccupancyGrid grid = make_grid(7, 5, walls);

  const PlanResult base =
      AStarPlanner{grid, cfg(Connectivity::Four, Heuristic::Manhattan, 0.0)}
          .plan(at_cell(0, 2), at_cell(6, 2));
  const PlanResult grad =
      AStarPlanner{grid, cfg(Connectivity::Four, Heuristic::Manhattan, 5.0)}
          .plan(at_cell(0, 2), at_cell(6, 2));

  ASSERT_TRUE(base.success);
  ASSERT_TRUE(grad.success);
  // 等长(都是直着走 6 步),但梯度版严格走中线。
  EXPECT_NEAR(grad.path.length(), base.path.length(), kEps);
  for (const Pose2D& p : grad.path.poses) {
    EXPECT_EQ(grid.world_to_grid(p.position()).y, 2);
  }
}

// ---------------------------------------------------------------------------
// 规模与性能:里程碑 "200x200 单次规划 ≤ 50ms(Release)"
//
// wall-clock 只在 Release 下有意义,而 CI 跑的是 Debug —— 所以另用与构建类型、
// 机器负载都无关的确定性断言守护"算法工作量":启发式确实在收束搜索(展开数
// 上界),以及压力图上的精确最优长度与展开数区间。
// ---------------------------------------------------------------------------

namespace {

// 蛇形迷宫:y = 2, 5, 8, … 每 3 行一道横墙,缺口左右交替,墙间是 2 格高的走廊。
// 从 (0,0) 到 (n-1,n-1) 必须蛇行穿过全部走廊;Octile 启发式一直指向右上角,
// 对这种来回折返的结构几乎失效,A* 被迫展开接近全部 free cell —— 接近最坏
// 情况的负载,而不是空图对角线那种最好情况。
OccupancyGrid serpentine_map(int n) {
  std::vector<GridCoord> walls;
  int wall = 0;
  for (int y = 2; y < n; y += 3, ++wall) {
    const int gap = (wall % 2 == 0) ? n - 1 : 0;  // 缺口左右交替
    for (int x = 0; x < n; ++x) {
      if (x != gap) {
        walls.push_back({x, y});
      }
    }
  }
  return make_grid(n, n, walls);
}

std::size_t count_free(const OccupancyGrid& grid) {
  std::size_t n = 0;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (grid.is_free(GridCoord{x, y})) {
        ++n;
      }
    }
  }
  return n;
}

}  // namespace

// 空图对角线:只有对角线上的 cell 满足 f = f*(偏离一步 f 至少大 2 − √2 ≈ 0.586),
// 所以 A* 恰好展开 n 个节点;启发式若失效退化成 Dijkstra,同一问题要展开近 n² 个。
TEST(AStar, HeuristicFocusesSearchOnOpenMap) {
  constexpr int kN = 200;
  const AStarPlanner planner{make_grid(kN, kN, {}),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(kN - 1, kN - 1));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), (kN - 1) * kSqrt2, kEps);
  // 理论值恰为 kN;留 2 倍余量,不把 tie-break 细节写死。
  EXPECT_LE(r.expanded_nodes, 2 * static_cast<std::size_t>(kN));
}

// 200x200 蛇形迷宫上的精确最短路:67 条走廊各 198 直 + 1 斜(198 + √2);66 个
// 门洞各 2 个正交步(防穿角禁止斜着进出门洞)→ 67·(198 + √2) + 132。
TEST(AStar, FindsExactShortestPathThroughSerpentine200x200) {
  const OccupancyGrid grid = serpentine_map(200);
  const AStarPlanner planner{grid, cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(199, 199));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.path.length(), 67.0 * (198.0 + kSqrt2) + 132.0, 1e-6);

  // 负载前提:这张图确实逼 A* 搜遍大半张图(防止计时测试再次退化成最好情况);
  // closed 表保证每个 cell 至多展开一次。
  const std::size_t free_cells = count_free(grid);
  EXPECT_GE(r.expanded_nodes, free_cells / 2);
  EXPECT_LE(r.expanded_nodes, free_cells);
}

// 里程碑指标本身。Debug 下显式跳过而不是静默通过,CI 日志里能看出它没被执行。
TEST(AStar, MeetsTimingBudgetOnSerpentine200x200) {
#ifndef NDEBUG
  GTEST_SKIP() << "wall-clock budget is a Release-build milestone";
#else
  const AStarPlanner planner{serpentine_map(200),
                             cfg(Connectivity::Eight, Heuristic::Octile)};
  const PlanResult r = planner.plan(at_cell(0, 0), at_cell(199, 199));
  ASSERT_TRUE(r.success);
  EXPECT_LT(r.plan_time_ms, 50.0);
#endif
}
