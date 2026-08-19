#include "autonomy/geometry.hpp"
#include <gtest/gtest.h>

TEST(Geometry, PairsCones)
{
  const auto result = autonomy::pair_cones(
    {{5.0, 2.0}, {1.0, 2.0}, {3.0, 2.0}},
    {{3.1, -2.0}, {5.1, -2.0}, {1.1, -2.0}}, 1.5, 6.0, 2.0);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_NEAR(result[0].x, 1.05, 1e-9);
  EXPECT_NEAR(result[2].x, 5.05, 1e-9);
}

TEST(Geometry, RejectsNarrowTrack)
{
  EXPECT_TRUE(autonomy::pair_cones({{2.0, 0.2}}, {{2.0, -0.2}}, 1.5, 6.0, 2.0).empty());
}

TEST(Geometry, BuildsCenterlineFromLeftBoundary)
{
  const auto result = autonomy::centerline_from_boundary(
    {{6.0, 2.0}, {2.0, 2.0}, {4.0, 2.0}}, true, 2.0);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_DOUBLE_EQ(result[0].x, 2.0);
  EXPECT_NEAR(result[0].y, 0.0, 1e-9);
  EXPECT_NEAR(result[2].y, 0.0, 1e-9);
}

TEST(Geometry, BuildsCenterlineFromRightBoundary)
{
  const auto result = autonomy::centerline_from_boundary(
    {{2.0, -2.0}, {4.0, -2.0}, {6.0, -2.0}}, false, 2.0);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_NEAR(result[0].y, 0.0, 1e-9);
  EXPECT_NEAR(result[2].y, 0.0, 1e-9);
}

TEST(Geometry, LookaheadAndCurvature)
{
  const auto target = autonomy::select_lookahead(
    {{-1.0, 0.0}, {1.0, 0.0}, {3.0, 1.0}}, 2.0);
  ASSERT_TRUE(target.has_value());
  EXPECT_DOUBLE_EQ(target->x, 3.0);
  EXPECT_GT(autonomy::pure_pursuit_curvature(*target), 0.0);
}
