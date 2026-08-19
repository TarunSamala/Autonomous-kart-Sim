#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fs_msgs/msg/cone.hpp"
#include "fs_msgs/msg/track.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace
{
struct Point2
{
  double x{};
  double y{};
};

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<Point2> smooth_closed(std::vector<Point2> points, const int passes)
{
  for (int pass = 0; pass < passes && points.size() >= 3; ++pass) {
    std::vector<Point2> smoothed(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & previous = points[(index + points.size() - 1) % points.size()];
      const auto & current = points[index];
      const auto & next = points[(index + 1) % points.size()];
      smoothed[index] = {
        0.25 * previous.x + 0.50 * current.x + 0.25 * next.x,
        0.25 * previous.y + 0.50 * current.y + 0.25 * next.y};
    }
    points = std::move(smoothed);
  }
  return points;
}

std::vector<Point2> resample_closed(
  const std::vector<Point2> & input, const double spacing)
{
  if (input.size() < 3) {
    return input;
  }

  std::vector<double> cumulative(input.size() + 1, 0.0);
  for (std::size_t index = 0; index < input.size(); ++index) {
    cumulative[index + 1] = cumulative[index] +
      distance(input[index], input[(index + 1) % input.size()]);
  }
  const double total_length = cumulative.back();
  const std::size_t sample_count = std::max<std::size_t>(
    3, static_cast<std::size_t>(std::ceil(total_length / spacing)));

  std::vector<Point2> output;
  output.reserve(sample_count);
  std::size_t segment = 0;
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const double target = total_length * static_cast<double>(sample) /
      static_cast<double>(sample_count);
    while (segment + 1 < cumulative.size() && cumulative[segment + 1] < target) {
      ++segment;
    }
    const std::size_t next = (segment + 1) % input.size();
    const double length = cumulative[segment + 1] - cumulative[segment];
    const double ratio = length > 1e-9 ? (target - cumulative[segment]) / length : 0.0;
    output.push_back({
      input[segment].x + ratio * (input[next].x - input[segment].x),
      input[segment].y + ratio * (input[next].y - input[segment].y)});
  }
  return output;
}
}  // namespace

class TrackReferencePlanner : public rclcpp::Node
{
public:
  TrackReferencePlanner()
  : Node("track_reference_planner")
  {
    track_topic_ = declare_parameter<std::string>(
      "track_topic", "/fsds/testing_only/track");
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/fsds/testing_only/odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    global_frame_ = declare_parameter<std::string>("global_frame", "fsds/map");
    path_spacing_ = declare_parameter<double>("path_spacing", 0.40);
    path_horizon_ = declare_parameter<double>("path_horizon", 24.0);
    max_path_deviation_ = declare_parameter<double>("max_path_deviation", 3.0);
    smoothing_passes_ = declare_parameter<int>("smoothing_passes", 3);
    if (path_spacing_ <= 0.0 || path_horizon_ <= 2.0 ||
      max_path_deviation_ <= 0.0 || smoothing_passes_ < 0)
    {
      throw std::invalid_argument("invalid track reference planner parameters");
    }

    auto track_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    track_qos.reliable().transient_local();
    track_sub_ = create_subscription<fs_msgs::msg::Track>(
      track_topic_, track_qos,
      std::bind(&TrackReferencePlanner::track_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrackReferencePlanner::odom_callback, this, std::placeholders::_1));
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/autonomy/local_path", 10);
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/autonomy/reference_path", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/planner_status", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(50ms, std::bind(&TrackReferencePlanner::publish, this));
    set_status("WAITING_FOR_TRACK");
    RCLCPP_WARN(
      get_logger(),
      "Using FSDS testing-only track and odometry; valid for control benchmarking, not final autonomy");
  }

private:
  void set_status(const std::string & value)
  {
    if (value == status_) {
      return;
    }
    status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "State: %s", value.c_str());
  }

  void track_callback(const fs_msgs::msg::Track::SharedPtr message)
  {
    std::vector<Point2> blue;
    std::vector<Point2> yellow;
    for (const auto & cone : message->track) {
      const Point2 point{cone.location.x, cone.location.y};
      if (cone.color == fs_msgs::msg::Cone::BLUE) {
        blue.push_back(point);
      } else if (cone.color == fs_msgs::msg::Cone::YELLOW) {
        yellow.push_back(point);
      }
    }
    if (blue.size() < 3 || yellow.size() < 3) {
      set_status("INVALID_TRACK_BOUNDARIES");
      RCLCPP_ERROR(
        get_logger(), "Track requires blue and yellow boundaries; got %zu and %zu",
        blue.size(), yellow.size());
      return;
    }

    const std::size_t count = std::min(blue.size(), yellow.size());
    std::vector<Point2> centerline;
    centerline.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const std::size_t blue_index = static_cast<std::size_t>(
        std::floor(static_cast<double>(index) * blue.size() / count));
      const std::size_t yellow_index = static_cast<std::size_t>(
        std::floor(static_cast<double>(index) * yellow.size() / count));
      centerline.push_back({
        0.5 * (blue[blue_index].x + yellow[yellow_index].x),
        0.5 * (blue[blue_index].y + yellow[yellow_index].y)});
    }
    centerline = smooth_closed(std::move(centerline), smoothing_passes_);
    centerline_ = resample_closed(centerline, path_spacing_);

    // FSDS track arrays normally progress in driving direction. Verify that
    // the tangent nearest the start points along the car's initial +X axis.
    const Point2 origin{};
    const auto nearest = nearest_index(origin);
    const auto next = (nearest + 1) % centerline_.size();
    if (centerline_[next].x - centerline_[nearest].x < 0.0) {
      std::reverse(centerline_.begin(), centerline_.end());
    }
    progress_index_.reset();
    publish_reference_path();
    set_status("WAITING_FOR_POSE");
    RCLCPP_INFO(
      get_logger(), "Closed reference path ready: %zu samples, blue=%zu, yellow=%zu",
      centerline_.size(), blue.size(), yellow.size());
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    position_ = Point2{message->pose.pose.position.x, message->pose.pose.position.y};
    const auto & q = message->pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    double roll{};
    double pitch{};
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_);
    pose_stamp_ = now();
    if (!message->header.frame_id.empty()) {
      global_frame_ = message->header.frame_id;
    }
  }

  std::size_t nearest_index(const Point2 & position) const
  {
    std::size_t best = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < centerline_.size(); ++index) {
      const double candidate = distance(position, centerline_[index]);
      if (candidate < best_distance) {
        best = index;
        best_distance = candidate;
      }
    }
    return best;
  }

  void publish_reference_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    for (const auto & point : centerline_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    reference_path_pub_->publish(path);
  }

  void publish()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = base_frame_;
    path.header.stamp = now();
    if (centerline_.size() < 3) {
      local_path_pub_->publish(path);
      return;
    }
    if (!pose_stamp_.has_value() || (now() - *pose_stamp_).seconds() > 0.5) {
      set_status("WAITING_FOR_POSE");
      local_path_pub_->publish(path);
      return;
    }

    const std::size_t nearest = nearest_index(position_);
    const double lateral_error = distance(position_, centerline_[nearest]);
    if (lateral_error > max_path_deviation_) {
      set_status("OUTSIDE_TRACK_CORRIDOR");
      local_path_pub_->publish(path);
      return;
    }
    progress_index_ = nearest;

    double travelled = 0.0;
    std::size_t index = nearest;
    const double cosine = std::cos(yaw_);
    const double sine = std::sin(yaw_);
    while (travelled <= path_horizon_ && path.poses.size() < centerline_.size()) {
      const auto & point = centerline_[index];
      const double dx = point.x - position_.x;
      const double dy = point.y - position_.y;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = cosine * dx + sine * dy;
      pose.pose.position.y = -sine * dx + cosine * dy;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);

      const std::size_t next = (index + 1) % centerline_.size();
      travelled += distance(centerline_[index], centerline_[next]);
      index = next;
    }
    set_status("TRACKING_REFERENCE");
    local_path_pub_->publish(path);
  }

  std::string track_topic_;
  std::string pose_topic_;
  std::string base_frame_;
  std::string global_frame_;
  double path_spacing_{};
  double path_horizon_{};
  double max_path_deviation_{};
  int smoothing_passes_{};
  std::vector<Point2> centerline_;
  Point2 position_{};
  double yaw_{};
  std::string status_;
  std::optional<std::size_t> progress_index_;
  std::optional<rclcpp::Time> pose_stamp_;
  rclcpp::Subscription<fs_msgs::msg::Track>::SharedPtr track_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TrackReferencePlanner>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("track_reference_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
