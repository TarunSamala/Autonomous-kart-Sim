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
