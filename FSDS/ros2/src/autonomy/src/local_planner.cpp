#include "autonomy/geometry.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class LocalPlanner : public rclcpp::Node
{
public:
  LocalPlanner()
  : Node("local_planner")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    min_track_width_ = declare_parameter<double>("min_track_width", 1.5);
    max_track_width_ = declare_parameter<double>("max_track_width", 6.0);
    max_longitudinal_offset_ = declare_parameter<double>("max_longitudinal_offset", 2.0);
    half_track_width_ = declare_parameter<double>("half_track_width", 2.0);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    use_delaunay_ = declare_parameter<bool>("use_delaunay", true);
    delaunay_options_.min_track_width = min_track_width_;
    delaunay_options_.max_track_width = max_track_width_;
    delaunay_options_.sensor_horizon = declare_parameter<double>("sensor_horizon", 12.0);
    delaunay_options_.minimum_forward_x = declare_parameter<double>("minimum_forward_x", 0.1);
    delaunay_options_.maximum_segment_length =
      declare_parameter<double>("maximum_segment_length", 6.0);
    const int beam_width = declare_parameter<int>("beam_width", 64);
    const int maximum_depth = declare_parameter<int>("maximum_depth", 14);
    const int maximum_debug_paths = declare_parameter<int>("maximum_debug_paths", 40);
    delaunay_options_.heading_weight = declare_parameter<double>("heading_weight", 5.0);
    delaunay_options_.width_weight = declare_parameter<double>("width_weight", 1.5);
    delaunay_options_.spacing_weight = declare_parameter<double>("spacing_weight", 0.8);
    delaunay_options_.horizon_weight = declare_parameter<double>("horizon_weight", 2.0);
    delaunay_options_.start_weight = declare_parameter<double>("start_weight", 1.0);
    delaunay_options_.color_order_weight = declare_parameter<double>("color_order_weight", 8.0);
    if (min_track_width_ <= 0.0 || max_track_width_ <= min_track_width_ ||
      max_longitudinal_offset_ <= 0.0 || half_track_width_ <= 0.0 || input_timeout_ <= 0.0 ||
      delaunay_options_.sensor_horizon <= 0.0 ||
      delaunay_options_.minimum_forward_x < 0.0 ||
      delaunay_options_.maximum_segment_length <= 0.0 || beam_width <= 0 ||
      maximum_depth < 2 || maximum_debug_paths <= 0 ||
      delaunay_options_.heading_weight < 0.0 || delaunay_options_.width_weight < 0.0 ||
      delaunay_options_.spacing_weight < 0.0 || delaunay_options_.horizon_weight < 0.0 ||
      delaunay_options_.start_weight < 0.0 || delaunay_options_.color_order_weight < 0.0)
    {
      throw std::invalid_argument("invalid local planner parameters");
    }
    delaunay_options_.beam_width = static_cast<std::size_t>(beam_width);
    delaunay_options_.maximum_depth = static_cast<std::size_t>(maximum_depth);
    delaunay_options_.maximum_debug_paths = static_cast<std::size_t>(maximum_debug_paths);

    left_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/left", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        left_ = to_points(*message);
        left_time_ = now();
      });
    right_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/right", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        right_ = to_points(*message);
        right_time_ = now();
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/autonomy/local_path", 10);
    debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/autonomy/debug/delaunay_planner", 10);
    timer_ = create_wall_timer(50ms, std::bind(&LocalPlanner::publish_path, this));
    RCLCPP_INFO(
      get_logger(), "Local LiDAR planner ready (%s, beam=%zu, depth=%zu, horizon=%.1f m)",
      use_delaunay_ ? "Delaunay beam search" : "paired fallback",
      delaunay_options_.beam_width, delaunay_options_.maximum_depth,
      delaunay_options_.sensor_horizon);
  }

private:
  static std::vector<autonomy::Point2> to_points(
    const geometry_msgs::msg::PoseArray & message)
  {
    std::vector<autonomy::Point2> points;
    points.reserve(message.poses.size());
    for (const auto & pose : message.poses) {
      if (pose.position.x > 0.0) {
        points.push_back({pose.position.x, pose.position.y});
      }
    }
    return points;
  }

  bool fresh(const std::optional<rclcpp::Time> & stamp) const
  {
    return stamp.has_value() && (now() - *stamp).seconds() <= input_timeout_;
  }

  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = base_frame_;
    path.header.stamp = now();
    if (!fresh(left_time_) || !fresh(right_time_)) {
      report_mode(PathMode::kNoPath, 0U, 0U);
      path_pub_->publish(path);
      debug_pub_->publish(make_debug_markers({}, path.header.stamp));
      return;
    }

    autonomy::DelaunayPlan delaunay_plan;
    std::vector<autonomy::Point2> centerline;
    PathMode mode = PathMode::kNoPath;
    if (use_delaunay_) {
      delaunay_plan = autonomy::plan_delaunay_centerline(
        left_, right_, delaunay_options_);
      centerline = delaunay_plan.selected_path;
      if (centerline.size() >= 2) {
        mode = PathMode::kDelaunay;
      }
    }
    if (centerline.size() < 2) {
      centerline = autonomy::pair_cones(
        left_, right_, min_track_width_, max_track_width_, max_longitudinal_offset_);
      if (centerline.size() >= 2) {
        mode = PathMode::kPairedFallback;
      }
    }
    if (centerline.size() < 2) {
      if (left_.size() >= 2 && left_.size() >= right_.size()) {
        centerline = autonomy::centerline_from_boundary(left_, true, half_track_width_);
        mode = PathMode::kLeftBoundary;
      } else if (right_.size() >= 2) {
        centerline = autonomy::centerline_from_boundary(right_, false, half_track_width_);
        mode = PathMode::kRightBoundary;
      }
    }
    if (centerline.size() < 2) {
      centerline.clear();
      mode = PathMode::kNoPath;
    } else {
      centerline = autonomy::smooth_path(centerline);
    }
    report_mode(mode, centerline.size(), delaunay_plan.candidate_paths.size());
    for (const auto & point : centerline) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    debug_pub_->publish(make_debug_markers(delaunay_plan, path.header.stamp));
  }

  visualization_msgs::msg::MarkerArray make_debug_markers(
    const autonomy::DelaunayPlan & plan, const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    const auto marker = [this, &stamp](
      const std::string & marker_namespace, const int id, const int type,
      const float width, const std::array<float, 4> & color)
      {
        visualization_msgs::msg::Marker value;
        value.header.frame_id = base_frame_;
        value.header.stamp = stamp;
        value.ns = marker_namespace;
        value.id = id;
        value.type = type;
        value.action = visualization_msgs::msg::Marker::ADD;
        value.pose.orientation.w = 1.0;
        value.scale.x = width;
        value.color.r = color[0];
        value.color.g = color[1];
        value.color.b = color[2];
        value.color.a = color[3];
        value.lifetime.nanosec = 150000000U;
        return value;
      };
    const auto append_point = [](auto & output, const autonomy::Point2 & point, const double z) {
        geometry_msgs::msg::Point value;
        value.x = point.x;
        value.y = point.y;
        value.z = z;
        output.points.push_back(value);
      };

    auto triangulation = marker(
      "triangulation", 0, visualization_msgs::msg::Marker::LINE_LIST,
      0.018F, {0.18F, 0.18F, 0.18F, 0.55F});
    for (const auto & edge : plan.triangulation_edges) {
      append_point(triangulation, edge.first, 0.015);
      append_point(triangulation, edge.second, 0.015);
    }
    array.markers.push_back(std::move(triangulation));

    auto gates = marker(
      "valid_gates", 0, visualization_msgs::msg::Marker::LINE_LIST,
      0.028F, {0.25F, 0.55F, 1.0F, 0.35F});
    for (const auto & gate : plan.gates) {
      append_point(gates, gate.first, 0.025);
      append_point(gates, gate.second, 0.025);
    }
    array.markers.push_back(std::move(gates));

    for (std::size_t index = 0; index < plan.candidate_paths.size(); ++index) {
      auto candidate = marker(
        "candidate_paths", static_cast<int>(index),
        visualization_msgs::msg::Marker::LINE_STRIP,
        0.025F, {0.15F, 0.95F, 0.25F, 0.24F});
      geometry_msgs::msg::Point origin;
      origin.z = 0.04;
      candidate.points.push_back(origin);
      for (const auto & point : plan.candidate_paths[index]) {
        append_point(candidate, point, 0.04);
      }
      array.markers.push_back(std::move(candidate));
    }

    if (!plan.selected_path.empty()) {
      auto selected = marker(
        "selected_path", 0, visualization_msgs::msg::Marker::LINE_STRIP,
        0.075F, {1.0F, 0.12F, 0.08F, 1.0F});
      geometry_msgs::msg::Point origin;
      origin.z = 0.06;
      selected.points.push_back(origin);
      for (const auto & point : plan.selected_path) {
        append_point(selected, point, 0.06);
      }
      array.markers.push_back(std::move(selected));
    }
    return array;
  }

  enum class PathMode : std::uint8_t
  {
    kNoPath,
    kDelaunay,
    kPairedFallback,
    kLeftBoundary,
    kRightBoundary
  };

  void report_mode(
    const PathMode mode, const std::size_t point_count,
    const std::size_t candidate_count)
  {
    if (mode == path_mode_) {
      return;
    }
    path_mode_ = mode;
    switch (mode) {
      case PathMode::kDelaunay:
        RCLCPP_INFO(
          get_logger(), "Path mode: Delaunay beam search (%zu points, %zu candidates)",
          point_count, candidate_count);
        break;
      case PathMode::kPairedFallback:
        RCLCPP_WARN(get_logger(), "Path mode: paired-cone fallback (%zu points)", point_count);
        break;
      case PathMode::kLeftBoundary:
        RCLCPP_WARN(get_logger(), "Path mode: left-boundary fallback (%zu points)", point_count);
        break;
      case PathMode::kRightBoundary:
        RCLCPP_WARN(get_logger(), "Path mode: right-boundary fallback (%zu points)", point_count);
        break;
      case PathMode::kNoPath:
        RCLCPP_WARN(get_logger(), "Path mode: no safe path");
        break;
    }
  }

  std::string base_frame_;
  double min_track_width_;
  double max_track_width_;
  double max_longitudinal_offset_;
  double half_track_width_;
  double input_timeout_;
  bool use_delaunay_;
  autonomy::DelaunayPlannerOptions delaunay_options_;
  PathMode path_mode_{PathMode::kNoPath};
  std::vector<autonomy::Point2> left_;
  std::vector<autonomy::Point2> right_;
  std::optional<rclcpp::Time> left_time_;
  std::optional<rclcpp::Time> right_time_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr right_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalPlanner>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("local_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
