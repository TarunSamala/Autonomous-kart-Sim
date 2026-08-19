#include "autonomy/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <unordered_set>

namespace autonomy
{
std::vector<Point2> pair_cones(
  const std::vector<Point2> & left, const std::vector<Point2> & right,
  const double min_width, const double max_width, const double max_longitudinal_offset)
{
  using Candidate = std::tuple<double, std::size_t, std::size_t>;
  std::vector<Candidate> candidates;
  for (std::size_t li = 0; li < left.size(); ++li) {
    for (std::size_t ri = 0; ri < right.size(); ++ri) {
      const double dx = left[li].x - right[ri].x;
      const double width = std::hypot(dx, left[li].y - right[ri].y);
      if (width >= min_width && width <= max_width &&
        std::abs(dx) <= max_longitudinal_offset)
      {
        candidates.emplace_back(std::abs(dx) + 0.1 * width, li, ri);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());
  std::unordered_set<std::size_t> used_left;
  std::unordered_set<std::size_t> used_right;
  std::vector<Point2> midpoints;
  for (const auto & [cost, li, ri] : candidates) {
    (void)cost;
    if (used_left.count(li) || used_right.count(ri)) {continue;}
    used_left.insert(li);
    used_right.insert(ri);
    midpoints.push_back({0.5 * (left[li].x + right[ri].x),
      0.5 * (left[li].y + right[ri].y)});
  }
  std::sort(midpoints.begin(), midpoints.end(),
    [](const Point2 & a, const Point2 & b) {return a.x < b.x;});
  return midpoints;
}

std::vector<Point2> centerline_from_boundary(
  const std::vector<Point2> & boundary, const bool boundary_is_left,
  const double half_track_width)
{
  if (boundary.size() < 2 || half_track_width <= 0.0) {
    return {};
  }

  auto ordered = boundary;
  std::sort(ordered.begin(), ordered.end(), [](const Point2 & a, const Point2 & b) {
    return a.x < b.x;
  });

  std::vector<Point2> centerline;
  centerline.reserve(ordered.size());
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    const Point2 & previous = ordered[i == 0 ? i : i - 1];
    const Point2 & next = ordered[i + 1 < ordered.size() ? i + 1 : i];
    const double tangent_x = next.x - previous.x;
    const double tangent_y = next.y - previous.y;
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length < 1e-6) {
      continue;
    }

    const double unit_x = tangent_x / tangent_length;
    const double unit_y = tangent_y / tangent_length;
    const double side = boundary_is_left ? 1.0 : -1.0;
    Point2 center{
      ordered[i].x + side * unit_y * half_track_width,
      ordered[i].y - side * unit_x * half_track_width};
    if (center.x > 0.0) {
      centerline.push_back(center);
    }
  }
  return centerline;
}

std::vector<Point2> smooth_path(const std::vector<Point2> & points)
{
  if (points.size() < 3) {return points;}
  std::vector<Point2> result{points.front()};
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    result.push_back({(points[i - 1].x + points[i].x + points[i + 1].x) / 3.0,
      (points[i - 1].y + points[i].y + points[i + 1].y) / 3.0});
  }
  result.push_back(points.back());
  return result;
}

std::optional<Point2> select_lookahead(
  const std::vector<Point2> & points, const double lookahead_distance)
{
  std::optional<Point2> last_forward;
  for (const auto & point : points) {
    if (point.x <= 0.0) {continue;}
    last_forward = point;
    if (std::hypot(point.x, point.y) >= lookahead_distance) {return point;}
  }
  return last_forward;
}

double pure_pursuit_curvature(const Point2 & target)
{
  const double distance_squared = target.x * target.x + target.y * target.y;
  return distance_squared < 1e-6 ? 0.0 : 2.0 * target.y / distance_squared;
}
}  // namespace autonomy
