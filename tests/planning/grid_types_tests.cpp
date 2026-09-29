import mininav.planning.grid_types;

#include <gtest/gtest.h>

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

