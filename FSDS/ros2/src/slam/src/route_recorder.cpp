#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "fs_msgs/msg/extra_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

class RouteRecorder : public rclcpp::Node
{
public:
  RouteRecorder()
  : Node("route_recorder"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "slam/map");
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    output_path_ = declare_parameter<std::string>("output_path", "");
    sample_distance_m_ = declare_parameter<double>("sample_distance_m", 0.40);
    sample_period_s_ = declare_parameter<double>("sample_period_s", 0.05);
    minimum_route_points_ = declare_parameter<int>("minimum_route_points", 100);
    target_laps_ = declare_parameter<int>("target_laps", 1);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    use_fsds_lap_signal_ = declare_parameter<bool>("use_fsds_lap_signal", false);
    closure_radius_m_ = declare_parameter<double>("closure_radius_m", 2.0);
    minimum_loop_distance_m_ = declare_parameter<double>("minimum_loop_distance_m", 80.0);
    closure_heading_tolerance_rad_ =
      declare_parameter<double>("closure_heading_tolerance_rad", 0.80);

    if (global_frame_.empty() || base_frame_.empty() || output_path_.empty()) {
      throw std::invalid_argument("global_frame, base_frame and output_path must not be empty");
    }
    if (!std::isfinite(sample_distance_m_) || sample_distance_m_ <= 0.0 ||
      !std::isfinite(sample_period_s_) || sample_period_s_ <= 0.0 ||
      minimum_route_points_ < 3 || target_laps_ < 0 || closure_radius_m_ <= 0.0 ||
      minimum_loop_distance_m_ <= 0.0 || closure_heading_tolerance_rad_ <= 0.0 ||
      closure_heading_tolerance_rad_ > 3.14159265358979323846)
    {
      throw std::invalid_argument("invalid route recorder parameters");
    }

    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/slam/recorded_route", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/slam/route_recorder/status", rclcpp::QoS(1).reliable().transient_local());
    extra_info_sub_ = create_subscription<fs_msgs::msg::ExtraInfo>(
      "/fsds/testing_only/extra_info", 10,
      std::bind(&RouteRecorder::extra_info_callback, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/slam/route_recorder/save",
      std::bind(
        &RouteRecorder::save_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/slam/route_recorder/reset",
      std::bind(
        &RouteRecorder::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(sample_period_s_));
    timer_ = create_wall_timer(period, std::bind(&RouteRecorder::sample_pose, this));

    recording_ = auto_start_;
    route_.header.frame_id = global_frame_;
    set_status(recording_ ? "RECORDING" : "READY");
    RCLCPP_INFO(
      get_logger(), "SLAM route recorder %s -> %s (spacing %.2f m)",
      base_frame_.c_str(), fs::absolute(output_path_).c_str(), sample_distance_m_);
  }

private:
  void set_status(const std::string & status)
  {
    if (status == status_) {
      return;
    }
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "State: %s", status.c_str());
  }

  static double planar_distance(
    const geometry_msgs::msg::PoseStamped & first,
    const geometry_msgs::msg::PoseStamped & second)
  {
    return std::hypot(
      first.pose.position.x - second.pose.position.x,
      first.pose.position.y - second.pose.position.y);
  }

  static double yaw(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & orientation = pose.pose.orientation;
    tf2::Quaternion quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double heading = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, heading);
    return heading;
  }

  static double angle_distance(const double first, const double second)
  {
    return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
  }

  std::optional<geometry_msgs::msg::PoseStamped> current_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s -> %s: %s", global_frame_.c_str(), base_frame_.c_str(),
        error.what());
      return std::nullopt;
    }
  }

  void append_pose(const geometry_msgs::msg::PoseStamped & pose, const bool force)
  {
    if (!force && !route_.poses.empty() &&
      planar_distance(route_.poses.back(), pose) < sample_distance_m_)
    {
      return;
    }
    if (force && !route_.poses.empty() && planar_distance(route_.poses.back(), pose) < 0.01) {
      return;
    }
    route_.header.stamp = pose.header.stamp;
    if (route_.poses.empty()) {
      start_pose_ = pose;
      travelled_distance_m_ = 0.0;
    } else {
      travelled_distance_m_ += planar_distance(route_.poses.back(), pose);
    }
    route_.poses.push_back(pose);
    route_pub_->publish(route_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Recorded %zu route points", route_.poses.size());
  }

  void sample_pose()
  {
    if (!recording_) {
      return;
    }
    const auto pose = current_pose();
    if (pose) {
      append_pose(*pose, false);
      check_geometric_loop_closure(*pose);
    }
  }

  void check_geometric_loop_closure(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!recording_ || target_laps_ <= 0 || !start_pose_ ||
      route_.poses.size() < static_cast<std::size_t>(minimum_route_points_) ||
      travelled_distance_m_ < minimum_loop_distance_m_)
    {
      return;
    }
    const double closure_distance = planar_distance(*start_pose_, pose);
    const double heading_error = angle_distance(yaw(*start_pose_), yaw(pose));
    if (closure_distance > closure_radius_m_ ||
      heading_error > closure_heading_tolerance_rad_)
    {
      return;
    }

    ++geometric_laps_completed_;
    if (geometric_laps_completed_ < target_laps_) {
      start_pose_ = pose;
      travelled_distance_m_ = 0.0;
      RCLCPP_INFO(
        get_logger(), "Geometric loop %d/%d accepted; continuing teaching",
        geometric_laps_completed_, target_laps_);
      return;
    }

    recording_ = false;
    try {
      append_pose(pose, true);
      save_route("geometric_loop_closure");
      set_status("COMPLETE");
      RCLCPP_INFO(
        get_logger(),
        "Geometric loop closure accepted after %.1f m (position error %.2f m, heading error %.2f rad)",
        travelled_distance_m_, closure_distance, heading_error);
    } catch (const std::exception & error) {
      set_status("FAILED");
      RCLCPP_ERROR(get_logger(), "Could not save loop-closed route: %s", error.what());
    }
  }

  void extra_info_callback(const fs_msgs::msg::ExtraInfo::SharedPtr message)
  {
    if (!use_fsds_lap_signal_) {
      return;
    }
    const std::size_t lap_count = message->laps.size();
    if (!lap_count_at_start_) {
      lap_count_at_start_ = lap_count;
      RCLCPP_INFO(get_logger(), "Lap baseline: %zu", *lap_count_at_start_);
      return;
    }
    if (!recording_ || target_laps_ == 0 ||
      lap_count < *lap_count_at_start_ + static_cast<std::size_t>(target_laps_))
    {
      return;
    }

    if (route_.poses.size() < static_cast<std::size_t>(minimum_route_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring start-line lap event with only %zu route points (minimum %d)",
        route_.poses.size(), minimum_route_points_);
      lap_count_at_start_ = lap_count;
      return;
    }

    if (const auto pose = current_pose()) {
      append_pose(*pose, true);
    }
    recording_ = false;
    try {
      save_route("lap_target");
      set_status("COMPLETE");
    } catch (const std::exception & error) {
      set_status("FAILED");
      RCLCPP_ERROR(get_logger(), "Could not save route: %s", error.what());
    }
  }

  void save_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (const auto pose = current_pose()) {
      append_pose(*pose, true);
    }
    try {
      save_route("service");
      response->success = true;
      response->message = fs::absolute(output_path_).string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    route_.poses.clear();
    lap_count_at_start_.reset();
    start_pose_.reset();
    travelled_distance_m_ = 0.0;
    geometric_laps_completed_ = 0;
    recording_ = true;
    route_pub_->publish(route_);
    set_status("RECORDING");
    response->success = true;
    response->message = "route cleared and recording started";
  }

  void save_route(const std::string & reason)
  {
    if (route_.poses.size() < static_cast<std::size_t>(minimum_route_points_)) {
      throw std::runtime_error(
              "route has " + std::to_string(route_.poses.size()) +
              " points; requires at least " + std::to_string(minimum_route_points_));
    }

    json poses = json::array();
    for (const auto & pose : route_.poses) {
      const auto & orientation = pose.pose.orientation;
      tf2::Quaternion quaternion(
        orientation.x, orientation.y, orientation.z, orientation.w);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      poses.push_back({pose.pose.position.x, pose.pose.position.y, yaw});
    }

    const fs::path destination = fs::absolute(output_path_);
    fs::create_directories(destination.parent_path());
    const fs::path temporary = destination.string() + ".tmp";
    json document = {
      {"schema_version", 1},
      {"frame_id", global_frame_},
      {"closed", target_laps_ > 0},
      {"sample_distance_m", sample_distance_m_},
      {"point_count", route_.poses.size()},
      {"save_reason", reason},
      {"poses", std::move(poses)}
    };
    {
      std::ofstream stream(temporary);
      if (!stream) {
        throw std::runtime_error("cannot open temporary route file: " + temporary.string());
      }
      stream << std::setw(2) << document << '\n';
      if (!stream) {
        throw std::runtime_error("failed while writing route file: " + temporary.string());
      }
    }
    fs::rename(temporary, destination);
    RCLCPP_INFO(
      get_logger(), "Saved %zu-point route to %s", route_.poses.size(),
      destination.c_str());
  }

  std::string global_frame_;
  std::string base_frame_;
  std::string output_path_;
  std::string status_;
  double sample_distance_m_{};
  double sample_period_s_{};
  int minimum_route_points_{};
  int target_laps_{};
  bool auto_start_{};
  bool use_fsds_lap_signal_{};
  bool recording_{false};
  double closure_radius_m_{};
  double minimum_loop_distance_m_{};
  double closure_heading_tolerance_rad_{};
  double travelled_distance_m_{0.0};
  int geometric_laps_completed_{0};
  std::optional<std::size_t> lap_count_at_start_;
  std::optional<geometry_msgs::msg::PoseStamped> start_pose_;
  nav_msgs::msg::Path route_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RouteRecorder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
