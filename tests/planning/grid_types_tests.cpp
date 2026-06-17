import mininav.planning.grid_types;
import mininav.core.types;

#include <gtest/gtest.h>

namespace {

constexpr double kEps = 1e-12;

} // namespace

// ---------------------------------------------------------------------------
// GridCoord
// ---------------------------------------------------------------------------

TEST(GridCoord, DefaultIsOrigin) {
  const mininav::planning::GridCoord c;
  EXPECT_EQ(c.x, 0);
  EXPECT_EQ(c.y, 0);
}

TEST(GridCoord, EqualityComparesBothComponents) {
  using mininav::planning::GridCoord;
  EXPECT_EQ((GridCoord{3, 4}), (GridCoord{3, 4}));
  EXPECT_NE((GridCoord{3, 4}), (GridCoord{4, 3}));
  EXPECT_NE((GridCoord{3, 4}), (GridCoord{3, 5}));
}

// ---------------------------------------------------------------------------
// Path
// ---------------------------------------------------------------------------

TEST(Path, EmptyByDefault) {
  const mininav::planning::Path path;
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(path.size(), 0u);
  EXPECT_NEAR(path.length(), 0.0, kEps);
}

TEST(Path, SinglePoseHasZeroLength) {
  mininav::planning::Path path;
  path.poses.emplace_back(1.0, 2.0, 0.5);
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1u);
  EXPECT_NEAR(path.length(), 0.0, kEps);
}

TEST(Path, LengthIsCumulativeEuclidean) {
  mininav::planning::Path path;
  path.poses.emplace_back(0.0, 0.0, 0.0);
  path.poses.emplace_back(3.0, 0.0, 0.0); // +3 (horizontal)
  path.poses.emplace_back(3.0, 4.0, 0.0); // +4 (vertical)
  EXPECT_EQ(path.size(), 3u);
  EXPECT_NEAR(path.length(), 7.0, kEps);
}
