#include "autonomy/geometry.hpp"
#include <cmath>
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

TEST(Geometry, DelaunayPlannerBuildsStraightCenterline)
{
  autonomy::DelaunayPlannerOptions options;
  options.sensor_horizon = 8.0;
  const auto result = autonomy::plan_delaunay_centerline(
    {{1.0, 2.0}, {3.0, 2.0}, {5.0, 2.0}, {7.0, 2.0}, {9.0, 2.0}},
    {{1.0, -2.0}, {3.0, -2.0}, {5.0, -2.0}, {7.0, -2.0}, {9.0, -2.0}}, options);

  ASSERT_GE(result.selected_path.size(), 2U);
  EXPECT_FALSE(result.candidate_paths.empty());
  EXPECT_FALSE(result.triangulation_edges.empty());
  EXPECT_FALSE(result.gates.empty());
  for (std::size_t index = 0; index < result.selected_path.size(); ++index) {
    EXPECT_NEAR(result.selected_path[index].y, 0.0, 1.0e-9);
    if (index > 0) {
      EXPECT_GT(result.selected_path[index].x, result.selected_path[index - 1].x);
    }
  }
}

TEST(Geometry, DelaunayPlannerFollowsCurvedCorridor)
{
  autonomy::DelaunayPlannerOptions options;
  options.sensor_horizon = 10.0;
  const auto result = autonomy::plan_delaunay_centerline(
    {{1.0, 2.0}, {3.0, 2.1}, {5.0, 2.5}, {7.0, 3.2}, {9.0, 4.2}},
    {{1.0, -2.0}, {3.0, -1.9}, {5.0, -1.5}, {7.0, -0.8}, {9.0, 0.2}}, options);

  ASSERT_GE(result.selected_path.size(), 3U);
  EXPECT_GT(result.selected_path.back().y, result.selected_path.front().y);
  EXPECT_TRUE(std::isfinite(result.selected_cost));
}

TEST(Geometry, DelaunayPlannerRequiresBothBoundaries)
{
  const auto result = autonomy::plan_delaunay_centerline(
    {{1.0, 2.0}, {3.0, 2.0}, {5.0, 2.0}}, {});
  EXPECT_TRUE(result.selected_path.empty());
  EXPECT_TRUE(result.candidate_paths.empty());
}
