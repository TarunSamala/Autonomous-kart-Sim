#include "autonomy/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <unordered_set>

namespace autonomy
{
namespace
{
constexpr double kGeometryEpsilon = 1.0e-9;

struct TaggedPoint
{
  Point2 point;
  bool left{false};
};

struct Triangle
{
  std::size_t a{};
  std::size_t b{};
  std::size_t c{};
};

struct Gate
{
  Point2 midpoint;
  Point2 left;
  Point2 right;
  double width{};
};

using Edge = std::pair<std::size_t, std::size_t>;

double squared_distance(const Point2 & a, const Point2 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double orientation(const Point2 & a, const Point2 & b, const Point2 & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

Edge ordered_edge(std::size_t a, std::size_t b)
{
  if (a > b) {
    std::swap(a, b);
  }
  return {a, b};
}

Triangle counter_clockwise(
  const std::vector<TaggedPoint> & points, Triangle triangle)
{
  if (orientation(
      points[triangle.a].point, points[triangle.b].point,
      points[triangle.c].point) < 0.0)
  {
    std::swap(triangle.b, triangle.c);
  }
  return triangle;
}

bool circumcircle_contains(
  const std::vector<TaggedPoint> & points, const Triangle & triangle,
  const Point2 & query)
{
  const Point2 & a = points[triangle.a].point;
  const Point2 & b = points[triangle.b].point;
  const Point2 & c = points[triangle.c].point;
  const double ax = a.x - query.x;
  const double ay = a.y - query.y;
  const double bx = b.x - query.x;
  const double by = b.y - query.y;
  const double cx = c.x - query.x;
  const double cy = c.y - query.y;
  const double determinant =
    (ax * ax + ay * ay) * (bx * cy - by * cx) -
    (bx * bx + by * by) * (ax * cy - ay * cx) +
    (cx * cx + cy * cy) * (ax * by - ay * bx);
  return determinant > kGeometryEpsilon;
}

std::vector<Triangle> triangulate(std::vector<TaggedPoint> & points)
{
  const std::size_t input_size = points.size();
  if (input_size < 3) {
    return {};
  }

  double min_x = points.front().point.x;
  double max_x = min_x;
  double min_y = points.front().point.y;
  double max_y = min_y;
  for (const auto & tagged : points) {
    min_x = std::min(min_x, tagged.point.x);
    max_x = std::max(max_x, tagged.point.x);
    min_y = std::min(min_y, tagged.point.y);
    max_y = std::max(max_y, tagged.point.y);
  }
  const double span = std::max({max_x - min_x, max_y - min_y, 1.0});
  const double center_x = 0.5 * (min_x + max_x);
  const double center_y = 0.5 * (min_y + max_y);
  points.push_back({{center_x - 32.0 * span, center_y - span}, false});
  points.push_back({{center_x, center_y + 32.0 * span}, false});
  points.push_back({{center_x + 32.0 * span, center_y - span}, false});
  std::vector<Triangle> triangles{
    counter_clockwise(points, {input_size, input_size + 1, input_size + 2})};

  for (std::size_t point_index = 0; point_index < input_size; ++point_index) {
    std::vector<bool> bad(triangles.size(), false);
    std::map<Edge, std::size_t> edge_counts;
    for (std::size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index) {
      const Triangle & triangle = triangles[triangle_index];
      if (!circumcircle_contains(points, triangle, points[point_index].point)) {
        continue;
      }
      bad[triangle_index] = true;
      ++edge_counts[ordered_edge(triangle.a, triangle.b)];
      ++edge_counts[ordered_edge(triangle.b, triangle.c)];
      ++edge_counts[ordered_edge(triangle.c, triangle.a)];
    }

    std::vector<Triangle> retained;
    retained.reserve(triangles.size() + edge_counts.size());
    for (std::size_t index = 0; index < triangles.size(); ++index) {
      if (!bad[index]) {
        retained.push_back(triangles[index]);
      }
    }
    for (const auto & [edge, count] : edge_counts) {
      if (count != 1) {
        continue;
      }
      Triangle triangle{edge.first, edge.second, point_index};
      if (std::abs(orientation(
          points[triangle.a].point, points[triangle.b].point,
          points[triangle.c].point)) <= kGeometryEpsilon)
      {
        continue;
      }
      retained.push_back(counter_clockwise(points, triangle));
    }
    triangles = std::move(retained);
  }

  triangles.erase(
    std::remove_if(
      triangles.begin(), triangles.end(),
      [input_size](const Triangle & triangle) {
        return triangle.a >= input_size || triangle.b >= input_size ||
               triangle.c >= input_size;
      }),
    triangles.end());
  points.resize(input_size);
  return triangles;
}

double standard_deviation(const std::vector<double> & values)
{
  if (values.size() < 2) {
    return 0.0;
  }
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
    static_cast<double>(values.size());
  double variance = 0.0;
  for (const double value : values) {
    variance += (value - mean) * (value - mean);
  }
  return std::sqrt(variance / static_cast<double>(values.size()));
}

double path_length(const std::vector<std::size_t> & gate_ids, const std::vector<Gate> & gates)
{
  if (gate_ids.empty()) {
    return 0.0;
  }
  Point2 previous{0.0, 0.0};
  double length = 0.0;
  for (const std::size_t gate_id : gate_ids) {
    length += std::sqrt(squared_distance(previous, gates[gate_id].midpoint));
    previous = gates[gate_id].midpoint;
  }
  return length;
}

double path_cost(
  const std::vector<std::size_t> & gate_ids, const std::vector<Gate> & gates,
  const DelaunayPlannerOptions & options, const bool final_cost)
{
  if (gate_ids.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<double> widths;
  std::vector<double> spacings;
  widths.reserve(gate_ids.size());
  spacings.reserve(gate_ids.size());
  Point2 previous{0.0, 0.0};
  Point2 previous_direction{1.0, 0.0};
  double heading_cost = 0.0;
  double color_order_cost = 0.0;
  for (const std::size_t gate_id : gate_ids) {
    const Gate & gate = gates[gate_id];
    const Point2 direction{
      gate.midpoint.x - previous.x, gate.midpoint.y - previous.y};
    const double segment_length = std::hypot(direction.x, direction.y);
    if (segment_length <= kGeometryEpsilon) {
      return std::numeric_limits<double>::infinity();
    }
    spacings.push_back(segment_length);
    widths.push_back(gate.width);
    const double previous_norm = std::hypot(previous_direction.x, previous_direction.y);
    const double cosine = std::clamp(
      (previous_direction.x * direction.x + previous_direction.y * direction.y) /
      (previous_norm * segment_length), -1.0, 1.0);
    const double heading_change = std::acos(cosine);
    heading_cost += heading_change * heading_change;
    const Point2 right_to_left{gate.left.x - gate.right.x, gate.left.y - gate.right.y};
    if (direction.x * right_to_left.y - direction.y * right_to_left.x < 0.0) {
      color_order_cost += 1.0;
    }
    previous = gate.midpoint;
    previous_direction = direction;
  }
  const double length = path_length(gate_ids, gates);
  const double start_offset = std::abs(gates[gate_ids.front()].midpoint.y);
  double cost =
    options.heading_weight * heading_cost +
    options.width_weight * standard_deviation(widths) +
    options.spacing_weight * standard_deviation(spacings) +
    options.start_weight * start_offset +
    options.color_order_weight * color_order_cost;
  if (final_cost) {
    const double horizon_error = (length - options.sensor_horizon) / options.sensor_horizon;
    cost += options.horizon_weight * horizon_error * horizon_error;
  } else {
    cost -= 0.05 * std::min(length, options.sensor_horizon);
  }
  return cost;
}

std::vector<Point2> gate_path(
  const std::vector<std::size_t> & gate_ids, const std::vector<Gate> & gates)
{
  std::vector<Point2> result;
  result.reserve(gate_ids.size());
  for (const std::size_t id : gate_ids) {
    result.push_back(gates[id].midpoint);
  }
  return result;
}
}  // namespace

DelaunayPlan plan_delaunay_centerline(
  const std::vector<Point2> & left, const std::vector<Point2> & right,
  const DelaunayPlannerOptions & options)
{
  DelaunayPlan result;
  if (left.size() < 2 || right.size() < 2 || options.min_track_width <= 0.0 ||
    options.max_track_width <= options.min_track_width || options.sensor_horizon <= 0.0 ||
    options.maximum_segment_length <= 0.0 || options.beam_width == 0 ||
    options.maximum_depth < 2 || options.maximum_debug_paths == 0)
  {
    return result;
  }

  std::vector<TaggedPoint> points;
  points.reserve(left.size() + right.size());
  const auto append_unique = [&points](const Point2 & point, const bool is_left) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return;
      }
      const auto duplicate = std::find_if(
        points.begin(), points.end(), [&point](const TaggedPoint & existing) {
          return squared_distance(point, existing.point) < 1.0e-8;
        });
      if (duplicate == points.end()) {
        points.push_back({point, is_left});
      }
    };
  for (const auto & point : left) {append_unique(point, true);}
  for (const auto & point : right) {append_unique(point, false);}
  if (points.size() < 4) {
    return result;
  }

  const auto triangles = triangulate(points);
  std::set<Edge> triangulation_edges;
  for (const Triangle & triangle : triangles) {
    triangulation_edges.insert(ordered_edge(triangle.a, triangle.b));
    triangulation_edges.insert(ordered_edge(triangle.b, triangle.c));
    triangulation_edges.insert(ordered_edge(triangle.c, triangle.a));
  }
  for (const Edge & edge : triangulation_edges) {
    result.triangulation_edges.emplace_back(points[edge.first].point, points[edge.second].point);
  }

  std::vector<Gate> gates;
  std::map<Edge, std::size_t> gate_for_edge;
  for (const Edge & edge : triangulation_edges) {
    const TaggedPoint & first = points[edge.first];
    const TaggedPoint & second = points[edge.second];
    if (first.left == second.left) {
      continue;
    }
    const double width = std::sqrt(squared_distance(first.point, second.point));
    const Point2 midpoint{
      0.5 * (first.point.x + second.point.x), 0.5 * (first.point.y + second.point.y)};
    if (width < options.min_track_width || width > options.max_track_width ||
      midpoint.x < options.minimum_forward_x)
    {
      continue;
    }
    const Point2 left_point = first.left ? first.point : second.point;
    const Point2 right_point = first.left ? second.point : first.point;
    gate_for_edge.emplace(edge, gates.size());
    gates.push_back({midpoint, left_point, right_point, width});
    result.gates.emplace_back(left_point, right_point);
  }
  if (gates.size() < 2) {
    return result;
  }

  std::vector<std::set<std::size_t>> adjacency(gates.size());
  for (const Triangle & triangle : triangles) {
    const std::array<Edge, 3> edges{
      ordered_edge(triangle.a, triangle.b), ordered_edge(triangle.b, triangle.c),
      ordered_edge(triangle.c, triangle.a)};
    std::vector<std::size_t> triangle_gates;
    for (const Edge & edge : edges) {
      const auto found = gate_for_edge.find(edge);
      if (found != gate_for_edge.end()) {
        triangle_gates.push_back(found->second);
      }
    }
    for (std::size_t i = 0; i < triangle_gates.size(); ++i) {
      for (std::size_t j = i + 1; j < triangle_gates.size(); ++j) {
        adjacency[triangle_gates[i]].insert(triangle_gates[j]);
        adjacency[triangle_gates[j]].insert(triangle_gates[i]);
      }
    }
  }

  struct SearchState
  {
    std::vector<std::size_t> gates;
    double partial_cost{};
  };
  std::vector<std::size_t> start_ids(gates.size());
  std::iota(start_ids.begin(), start_ids.end(), 0);
  std::sort(start_ids.begin(), start_ids.end(), [&gates](const auto a, const auto b) {
    return squared_distance({0.0, 0.0}, gates[a].midpoint) <
           squared_distance({0.0, 0.0}, gates[b].midpoint);
  });
  if (start_ids.size() > options.beam_width) {
    start_ids.resize(options.beam_width);
  }
  std::vector<SearchState> frontier;
  for (const std::size_t id : start_ids) {
    SearchState state{{id}, 0.0};
    state.partial_cost = path_cost(state.gates, gates, options, false);
    frontier.push_back(std::move(state));
  }
  std::vector<SearchState> completed;
  for (std::size_t depth = 1; depth < options.maximum_depth && !frontier.empty(); ++depth) {
    std::vector<SearchState> next;
    for (const SearchState & state : frontier) {
      if (state.gates.size() >= 2) {
        completed.push_back(state);
      }
      const std::size_t current = state.gates.back();
      const Point2 previous = state.gates.size() >= 2 ?
        gates[state.gates[state.gates.size() - 2]].midpoint : Point2{0.0, 0.0};
      const Point2 incoming{
        gates[current].midpoint.x - previous.x,
        gates[current].midpoint.y - previous.y};
      const double incoming_norm = std::hypot(incoming.x, incoming.y);
      for (const std::size_t neighbour : adjacency[current]) {
        if (std::find(state.gates.begin(), state.gates.end(), neighbour) != state.gates.end()) {
          continue;
        }
        const Point2 outgoing{
          gates[neighbour].midpoint.x - gates[current].midpoint.x,
          gates[neighbour].midpoint.y - gates[current].midpoint.y};
        const double outgoing_norm = std::hypot(outgoing.x, outgoing.y);
        if (outgoing_norm <= kGeometryEpsilon ||
          outgoing_norm > options.maximum_segment_length || incoming_norm <= kGeometryEpsilon)
        {
          continue;
        }
        const double cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_norm * outgoing_norm);
        if (cosine < -0.2) {
          continue;
        }
        SearchState expanded = state;
        expanded.gates.push_back(neighbour);
        expanded.partial_cost = path_cost(expanded.gates, gates, options, false);
        next.push_back(std::move(expanded));
      }
    }
    std::sort(next.begin(), next.end(), [](const auto & a, const auto & b) {
      return a.partial_cost < b.partial_cost;
    });
    if (next.size() > options.beam_width) {
      next.resize(options.beam_width);
    }
    frontier = std::move(next);
  }
  completed.insert(completed.end(), frontier.begin(), frontier.end());
  completed.erase(
    std::remove_if(completed.begin(), completed.end(), [](const SearchState & state) {
      return state.gates.size() < 2;
    }), completed.end());
  std::sort(completed.begin(), completed.end(), [&gates, &options](const auto & a, const auto & b) {
    return path_cost(a.gates, gates, options, true) <
           path_cost(b.gates, gates, options, true);
  });
  if (completed.empty()) {
    return result;
  }

  result.selected_path = gate_path(completed.front().gates, gates);
  result.selected_cost = path_cost(completed.front().gates, gates, options, true);
  const std::size_t debug_count = std::min(options.maximum_debug_paths, completed.size());
  result.candidate_paths.reserve(debug_count);
  for (std::size_t index = 0; index < debug_count; ++index) {
    result.candidate_paths.push_back(gate_path(completed[index].gates, gates));
  }
  return result;
}

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

std::optional<double> stanley_steering_angle(
  const std::vector<Point2> & points, const double speed_mps,
  const double cross_track_gain, const double softening_speed_mps)
{
  if (points.size() < 2 || !std::isfinite(speed_mps) || speed_mps < 0.0 ||
    !std::isfinite(cross_track_gain) || cross_track_gain <= 0.0 ||
    !std::isfinite(softening_speed_mps) || softening_speed_mps <= 0.0)
  {
    return std::nullopt;
  }

  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_cross_track_error = 0.0;
  double best_heading_error = 0.0;
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    const Point2 & start = points[index];
    const Point2 & end = points[index + 1];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= kGeometryEpsilon) {
      continue;
    }

    // Project the vehicle origin onto each local-path segment. The selected
    // segment provides both the signed lateral error and desired heading.
    const double projection = std::clamp(
      -(start.x * dx + start.y * dy) / length_squared, 0.0, 1.0);
    const double closest_x = start.x + projection * dx;
    const double closest_y = start.y + projection * dy;
    const double distance_squared = closest_x * closest_x + closest_y * closest_y;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_cross_track_error = closest_y;
      const double path_heading = std::atan2(dy, dx);
      best_heading_error = std::atan2(std::sin(path_heading), std::cos(path_heading));
    }
  }

  if (!std::isfinite(best_distance_squared)) {
    return std::nullopt;
  }
  return best_heading_error + std::atan2(
    cross_track_gain * best_cross_track_error,
    speed_mps + softening_speed_mps);
}
}  // namespace autonomy
