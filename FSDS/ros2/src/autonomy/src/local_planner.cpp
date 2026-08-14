#include "autonomy/geometry.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

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
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    if (min_track_width_ <= 0.0 || max_track_width_ <= min_track_width_ ||
      max_longitudinal_offset_ <= 0.0 || input_timeout_ <= 0.0)
    {
      throw std::invalid_argument("invalid local planner parameters");
    }

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
    timer_ = create_wall_timer(50ms, std::bind(&LocalPlanner::publish_path, this));
    RCLCPP_INFO(get_logger(), "Local LiDAR planner ready");
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
      path_pub_->publish(path);
      return;
    }

    const auto centerline = autonomy::smooth_path(autonomy::pair_cones(
      left_, right_, min_track_width_, max_track_width_, max_longitudinal_offset_));
    for (const auto & point : centerline) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  std::string base_frame_;
  double min_track_width_;
  double max_track_width_;
  double max_longitudinal_offset_;
  double input_timeout_;
  std::vector<autonomy::Point2> left_;
  std::vector<autonomy::Point2> right_;
  std::optional<rclcpp::Time> left_time_;
  std::optional<rclcpp::Time> right_time_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr right_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
