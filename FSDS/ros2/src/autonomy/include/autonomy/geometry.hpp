#pragma once

#include <optional>
#include <vector>

namespace autonomy
{
struct Point2 {double x{0.0}; double y{0.0};};

std::vector<Point2> pair_cones(
  const std::vector<Point2> & left, const std::vector<Point2> & right,
  double min_width, double max_width, double max_longitudinal_offset);
std::vector<Point2> centerline_from_boundary(
  const std::vector<Point2> & boundary, bool boundary_is_left,
  double half_track_width);
std::vector<Point2> smooth_path(const std::vector<Point2> & points);
std::optional<Point2> select_lookahead(
  const std::vector<Point2> & points, double lookahead_distance);
double pure_pursuit_curvature(const Point2 & target);
}  // namespace autonomy
