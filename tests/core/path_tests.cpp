import mininav.core.path;
import mininav.core.types;

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <initializer_list>
#include <utility>

namespace {

constexpr double kEps = 1e-12;

using mininav::Path;
using mininav::PathProjection;
using mininav::lookahead_point;
using mininav::project_onto;

[[nodiscard]] Path make_path(std::initializer_list<std::pair<double, double>> xy) {
  Path path;
  for (const auto& [x, y] : xy) {
    path.poses.emplace_back(x, y, 0.0);
  }
  return path;
}

// L 形:(0,0) → (2,0) → (2,2),总长 4。
[[nodiscard]] Path l_shape() { return make_path({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}); }

// U 形回折:去程 y = 0,回程 y = 0.2 —— 回程段离去程上的车可能更近。
[[nodiscard]] Path hairpin() {
  return make_path({{0.0, 0.0}, {4.0, 0.0}, {4.0, 0.2}, {0.0, 0.2}});
}

// 车在 p 时的 look-ahead 点(从路径起点开始、窗口覆盖全部线段)。
[[nodiscard]] Eigen::Vector2d lookahead_from(const Path& path, const Eigen::Vector2d& p, double L) {
  const PathProjection proj = project_onto(path, p, 0, path.size());
  return lookahead_point(path, proj, p, L);
}

} // namespace

// ---------------------------------------------------------------------------
// Path 基本量(自 planning 迁入)
// ---------------------------------------------------------------------------

TEST(Path, EmptyByDefault) {
  const Path path;
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(path.size(), 0u);
  EXPECT_NEAR(path.length(), 0.0, kEps);
}

TEST(Path, SinglePoseHasZeroLength) {
  Path path;
  path.poses.emplace_back(1.0, 2.0, 0.5);
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1u);
  EXPECT_NEAR(path.length(), 0.0, kEps);
}

TEST(Path, LengthIsCumulativeEuclidean) {
  const Path path = make_path({{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}});
  EXPECT_EQ(path.size(), 3u);
  EXPECT_NEAR(path.length(), 7.0, kEps);
}

// ---------------------------------------------------------------------------
// project_onto
// ---------------------------------------------------------------------------

TEST(PathProjection, PerpendicularFootOnSegment) {
  const PathProjection r = project_onto(l_shape(), Eigen::Vector2d{1.0, 0.5}, 0, 2);
  EXPECT_EQ(r.segment, 0u);
  EXPECT_NEAR(r.t, 0.5, kEps);
  EXPECT_NEAR(r.arclength, 1.0, kEps);
  EXPECT_NEAR(r.point.x(), 1.0, kEps);
  EXPECT_NEAR(r.point.y(), 0.0, kEps);
  EXPECT_NEAR(r.distance, 0.5, kEps);  // 横向误差
}

TEST(PathProjection, ArclengthAccumulatesEarlierSegments) {
  const PathProjection r = project_onto(l_shape(), Eigen::Vector2d{2.3, 1.0}, 0, 2);
  EXPECT_EQ(r.segment, 1u);
  EXPECT_NEAR(r.t, 0.5, kEps);
  EXPECT_NEAR(r.arclength, 3.0, kEps);
  EXPECT_NEAR(r.distance, 0.3, kEps);
}

TEST(PathProjection, ClampsToPathEndpoints) {
  const PathProjection before = project_onto(l_shape(), Eigen::Vector2d{-1.0, 0.5}, 0, 2);
  EXPECT_EQ(before.segment, 0u);
  EXPECT_NEAR(before.t, 0.0, kEps);
  EXPECT_NEAR(before.distance, std::sqrt(1.25), kEps);

  const PathProjection after = project_onto(l_shape(), Eigen::Vector2d{2.5, 3.0}, 0, 2);
  EXPECT_EQ(after.segment, 1u);
  EXPECT_NEAR(after.t, 1.0, kEps);
  EXPECT_NEAR(after.arclength, 4.0, kEps);
  EXPECT_NEAR(after.point.y(), 2.0, kEps);
}

// 回折路径:车在去程上但离回程更近。窗口只覆盖当前段附近时,投影留在去程,
// 进度单调;窗口放开到全部线段时,就会跳到回程 —— 这正是窗口要防的情况。
TEST(PathProjection, WindowKeepsProgressOnHairpinOutboundLeg) {
  const Eigen::Vector2d p{1.0, 0.12};

  const PathProjection windowed = project_onto(hairpin(), p, 0, 2);
  EXPECT_EQ(windowed.segment, 0u);
  EXPECT_NEAR(windowed.distance, 0.12, kEps);

  const PathProjection global = project_onto(hairpin(), p, 0, 3);
  EXPECT_EQ(global.segment, 2u);
  EXPECT_NEAR(global.distance, 0.08, kEps);
}

TEST(PathProjection, SearchStartsAtFromSegmentAndClampsIt) {
  const PathProjection r = project_onto(l_shape(), Eigen::Vector2d{1.0, 0.5}, 1, 1);
  EXPECT_EQ(r.segment, 1u);  // 段 0 更近,但已在车后方
  EXPECT_NEAR(r.t, 0.25, kEps);  // 垂足 (2, 0.5)
  EXPECT_NEAR(r.arclength, 2.5, kEps);
  EXPECT_NEAR(r.distance, 1.0, kEps);

  const PathProjection clamped = project_onto(l_shape(), Eigen::Vector2d{1.0, 0.5}, 99, 1);
  EXPECT_EQ(clamped.segment, 1u);
}

TEST(PathProjection, SinglePosePathProjectsToThatPose) {
  const PathProjection r = project_onto(make_path({{1.0, 1.0}}), Eigen::Vector2d{4.0, 5.0}, 0, 1);
  EXPECT_EQ(r.segment, 0u);
  EXPECT_NEAR(r.point.x(), 1.0, kEps);
  EXPECT_NEAR(r.point.y(), 1.0, kEps);
  EXPECT_NEAR(r.distance, 5.0, kEps);
}

TEST(PathProjection, DuplicateWaypointIsHarmless) {
  const Path path = make_path({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}});
  const PathProjection r = project_onto(path, Eigen::Vector2d{0.5, 0.2}, 0, 2);
  EXPECT_EQ(r.segment, 1u);
  EXPECT_NEAR(r.arclength, 0.5, kEps);
  EXPECT_NEAR(r.distance, 0.2, kEps);
}

// ---------------------------------------------------------------------------
// lookahead_point
// ---------------------------------------------------------------------------

TEST(PathLookahead, OnStraightPathIsLAheadNotBehind) {
  const Path path = make_path({{0.0, 0.0}, {5.0, 0.0}});
  const Eigen::Vector2d g = lookahead_from(path, Eigen::Vector2d{2.0, 0.0}, 1.0);
  EXPECT_NEAR(g.x(), 3.0, kEps);  // (1, 0) 同样距车 1.0,但在车后方
  EXPECT_NEAR(g.y(), 0.0, kEps);
}

// 横向偏差 e 时,look-ahead 点比投影点超前 √(L² − e²)。
TEST(PathLookahead, WithLateralOffsetIntersectsCircle) {
  const Path path = make_path({{0.0, 0.0}, {5.0, 0.0}});
  const Eigen::Vector2d g = lookahead_from(path, Eigen::Vector2d{1.0, 0.3}, 0.5);
  EXPECT_NEAR(g.x(), 1.4, kEps);
  EXPECT_NEAR(g.y(), 0.0, kEps);
}

TEST(PathLookahead, ContinuesAcrossCorner) {
  const Path path = make_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  const Eigen::Vector2d g = lookahead_from(path, Eigen::Vector2d{0.8, 0.0}, 0.5);
  EXPECT_NEAR(g.x(), 1.0, kEps);
  EXPECT_NEAR(g.y(), std::sqrt(0.21), kEps);  // 0.2² + y² = 0.5²
}

// 回折时 look-ahead 沿路径绕过 U 形弯,落在回程上,而不是去程上的某个交点。
TEST(PathLookahead, FollowsPathAroundHairpin) {
  const Eigen::Vector2d g = lookahead_from(hairpin(), Eigen::Vector2d{3.9, 0.0}, 0.5);
  EXPECT_NEAR(g.x(), 3.9 - std::sqrt(0.21), kEps);
  EXPECT_NEAR(g.y(), 0.2, kEps);
}

TEST(PathLookahead, NearEndReturnsFinalWaypoint) {
  const Path path = make_path({{0.0, 0.0}, {5.0, 0.0}});
  const Eigen::Vector2d g = lookahead_from(path, Eigen::Vector2d{4.8, 0.0}, 0.5);
  EXPECT_NEAR(g.x(), 5.0, kEps);
  EXPECT_NEAR(g.y(), 0.0, kEps);
}

// 车离路径超过 L:圆与前方路径不相交,先驶回最近点。
TEST(PathLookahead, FarFromPathReturnsProjection) {
  const Path path = make_path({{0.0, 0.0}, {5.0, 0.0}});
  const Eigen::Vector2d g = lookahead_from(path, Eigen::Vector2d{2.0, 1.0}, 0.5);
  EXPECT_NEAR(g.x(), 2.0, kEps);
  EXPECT_NEAR(g.y(), 0.0, kEps);
}

TEST(PathLookahead, ResultIsExactlyLFromRobot) {
  const Path path = make_path({{0.0, 0.0}, {1.0, 0.3}, {2.1, -0.4}, {3.0, 1.0}});
  const Eigen::Vector2d p{0.9, 0.05};
  for (const double L : {0.3, 0.5, 1.0}) {  // 均大于车到路径的距离 ≈ 0.21
    const Eigen::Vector2d g = lookahead_from(path, p, L);
    EXPECT_NEAR((g - p).norm(), L, 1e-9) << "L = " << L;
  }
}
