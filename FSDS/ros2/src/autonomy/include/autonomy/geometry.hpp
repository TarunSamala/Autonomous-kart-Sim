#pragma once

#include <optional>
#include <utility>
#include <vector>

namespace autonomy
{
struct Point2 {double x{0.0}; double y{0.0};};

struct DelaunayPlannerOptions
{
  double min_track_width{1.5};
  double max_track_width{6.0};
  double sensor_horizon{12.0};
  double minimum_forward_x{0.1};
  double maximum_segment_length{6.0};
  std::size_t beam_width{64};
  std::size_t maximum_depth{14};
  std::size_t maximum_debug_paths{40};
  double heading_weight{5.0};
  double width_weight{1.5};
  double spacing_weight{0.8};
  double horizon_weight{2.0};
  double start_weight{1.0};
  double color_order_weight{8.0};
};

struct DelaunayPlan
{
  std::vector<Point2> selected_path;
  std::vector<std::vector<Point2>> candidate_paths;
  std::vector<std::pair<Point2, Point2>> triangulation_edges;
  std::vector<std::pair<Point2, Point2>> gates;
  double selected_cost{0.0};
};

DelaunayPlan plan_delaunay_centerline(
  const std::vector<Point2> & left, const std::vector<Point2> & right,
  const DelaunayPlannerOptions & options = {});

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
