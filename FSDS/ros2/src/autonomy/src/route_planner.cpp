#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{
struct RoutePoint
{
  double x{};
  double y{};
  double yaw{};
};

double wrap_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}
}  // namespace

class RoutePlanner : public rclcpp::Node
{
public:
  RoutePlanner()
  : Node("route_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    route_file_ = declare_parameter<std::string>("route_file", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    global_frame_ = declare_parameter<std::string>("global_frame", "");
    local_path_topic_ = declare_parameter<std::string>(
      "local_path_topic", "/autonomy/local_path");
    path_horizon_m_ = declare_parameter<double>("path_horizon_m", 24.0);
    maximum_route_distance_m_ = declare_parameter<double>(
      "maximum_route_distance_m", 4.0);
    heading_cost_weight_ = declare_parameter<double>("heading_cost_weight", 1.0);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.75);

    if (route_file_.empty() || base_frame_.empty() || path_horizon_m_ <= 0.0 ||
      maximum_route_distance_m_ <= 0.0 || heading_cost_weight_ < 0.0 ||
      transform_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid route planner parameters");
    }
    load_route();

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, 10);
    global_route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/autonomy/global_route", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/route_planner/status", rclcpp::QoS(1).reliable().transient_local());
    publish_global_route();
    timer_ = create_wall_timer(50ms, std::bind(&RoutePlanner::publish_local_path, this));
    set_status("WAITING_FOR_LOCALIZATION");
    RCLCPP_INFO(
      get_logger(), "Loaded %zu-point %s route from %s", route_.size(),
      closed_ ? "closed" : "open", route_file_.c_str());
  }

private:
  void load_route()
  {
    std::ifstream stream(route_file_);
    if (!stream) {
      throw std::invalid_argument("cannot open route_file: " + route_file_);
    }
    const json document = json::parse(stream);
    if (document.at("schema_version").get<int>() != 1) {
      throw std::invalid_argument("unsupported route schema_version");
    }
    const std::string file_frame = document.at("frame_id").get<std::string>();
    if (global_frame_.empty()) {
      global_frame_ = file_frame;
    } else if (global_frame_ != file_frame) {
      throw std::invalid_argument(
              "route frame '" + file_frame + "' does not match global_frame '" +
              global_frame_ + "'");
    }
    closed_ = document.at("closed").get<bool>();
    for (const auto & value : document.at("poses")) {
      if (!value.is_array() || value.size() < 3) {
        throw std::invalid_argument("route pose must be [x, y, yaw]");
      }
      RoutePoint point{value.at(0).get<double>(), value.at(1).get<double>(),
        value.at(2).get<double>()};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw))
      {
        throw std::invalid_argument("route contains a non-finite pose");
      }
      route_.push_back(point);
    }
    if (route_.size() < 3) {
      throw std::invalid_argument("route requires at least three poses");
    }
  }

  void set_status(const std::string & status)
  {
    if (status == status_) {return;}
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "State: %s", status.c_str());
  }

  void publish_global_route()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    for (const auto & point : route_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, point.yaw);
      pose.pose.orientation.x = orientation.x();
      pose.pose.orientation.y = orientation.y();
      pose.pose.orientation.z = orientation.z();
      pose.pose.orientation.w = orientation.w();
      path.poses.push_back(pose);
    }
    global_route_pub_->publish(path);
  }

  std::size_t nearest_index(const double x, const double y, const double yaw) const
  {
    std::size_t best_index = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < route_.size(); ++index) {
      const double dx = route_[index].x - x;
      const double dy = route_[index].y - y;
      const double heading_error = wrap_angle(route_[index].yaw - yaw);
      const double cost = dx * dx + dy * dy +
        heading_cost_weight_ * heading_error * heading_error;
      if (cost < best_cost) {
        best_cost = cost;
        best_index = index;
      }
    }
    return best_index;
  }

  void publish_empty()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = base_frame_;
    path.header.stamp = now();
    local_path_pub_->publish(path);
  }

  void publish_local_path()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      set_status("WAITING_FOR_LOCALIZATION");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Localization TF unavailable: %s", error.what());
      publish_empty();
      return;
    }

    const rclcpp::Time transform_stamp(transform.header.stamp);
    if (transform_stamp.nanoseconds() > 0 &&
      (now() - transform_stamp).seconds() > transform_timeout_s_)
    {
      set_status("STALE_LOCALIZATION");
      publish_empty();
      return;
    }

    const double car_x = transform.transform.translation.x;
    const double car_y = transform.transform.translation.y;
    const auto & rotation = transform.transform.rotation;
    tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double car_yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, car_yaw);

    const std::size_t nearest = nearest_index(car_x, car_y, car_yaw);
    const double route_distance = std::hypot(
      route_[nearest].x - car_x, route_[nearest].y - car_y);
    if (route_distance > maximum_route_distance_m_) {
      set_status("OFF_ROUTE");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Localized pose is %.2f m from route (limit %.2f m)",
        route_distance, maximum_route_distance_m_);
      publish_empty();
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = base_frame_;
    path.header.stamp = now();
    double travelled = 0.0;
    std::size_t previous = nearest;
    for (std::size_t step = 0; step < route_.size(); ++step) {
      const std::size_t index = nearest + step < route_.size() ? nearest + step :
        (closed_ ? (nearest + step) % route_.size() : route_.size());
      if (index >= route_.size()) {break;}
      if (step > 0) {
        travelled += std::hypot(
          route_[index].x - route_[previous].x,
          route_[index].y - route_[previous].y);
      }
      if (travelled > path_horizon_m_) {break;}

      const double dx = route_[index].x - car_x;
      const double dy = route_[index].y - car_y;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = std::cos(car_yaw) * dx + std::sin(car_yaw) * dy;
      pose.pose.position.y = -std::sin(car_yaw) * dx + std::cos(car_yaw) * dy;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
      previous = index;
    }

    if (path.poses.size() < 2) {
      set_status("ROUTE_EXHAUSTED");
      publish_empty();
      return;
    }
    set_status("TRACKING_ROUTE");
    local_path_pub_->publish(path);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "nearest=%zu distance=%.2f m local_points=%zu", nearest, route_distance,
      path.poses.size());
  }

  std::string route_file_;
  std::string base_frame_;
  std::string global_frame_;
  std::string local_path_topic_;
  std::string status_;
  double path_horizon_m_{};
  double maximum_route_distance_m_{};
  double heading_cost_weight_{};
  double transform_timeout_s_{};
  bool closed_{false};
  std::vector<RoutePoint> route_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RoutePlanner>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
