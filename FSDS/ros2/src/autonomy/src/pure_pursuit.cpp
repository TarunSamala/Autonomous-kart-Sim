#include "autonomy/geometry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{
volatile std::sig_atomic_t signal_requested = 0;
void signal_handler(int) {signal_requested = 1;}
}  // namespace

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit()
  : Node("pure_pursuit")
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    require_go_ = declare_parameter<bool>("require_go", true);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 2.5);
    wheelbase_ = declare_parameter<double>("wheelbase", 1.55);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.436332);
    straight_throttle_ = declare_parameter<double>("straight_throttle", 0.20);
    corner_throttle_ = declare_parameter<double>("corner_throttle", 0.11);
    target_speed_ = declare_parameter<double>("target_speed", 1.50);
    max_speed_ = declare_parameter<double>("max_speed", 2.00);
    overspeed_brake_ = declare_parameter<double>("overspeed_brake", 0.50);
    launch_throttle_ = declare_parameter<double>("launch_throttle", 0.45);
    launch_release_speed_ = declare_parameter<double>("launch_release_speed", 0.50);
    launch_timeout_ = declare_parameter<double>("launch_timeout", 0.80);
    path_timeout_ = declare_parameter<double>("path_timeout", 0.5);
    speed_timeout_ = declare_parameter<double>("speed_timeout", 0.5);
    go_timeout_ = declare_parameter<double>("go_timeout", 4.0);

    if (lookahead_distance_ <= 0.0 || wheelbase_ <= 0.0 ||
      max_steering_angle_ <= 0.0 || launch_release_speed_ <= 0.0 ||
      target_speed_ <= launch_release_speed_ || max_speed_ <= target_speed_ ||
      launch_timeout_ <= 0.0 || path_timeout_ <= 0.0 ||
      speed_timeout_ <= 0.0 || go_timeout_ <= 0.0 ||
      !unit(straight_throttle_) || !unit(corner_throttle_) ||
      !unit(launch_throttle_) || !unit(overspeed_brake_))
    {
      throw std::invalid_argument("invalid pure pursuit parameters");
    }

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/autonomy/local_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        path_.clear();
        path_.reserve(message->poses.size());
        for (const auto & pose : message->poses) {
          path_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        path_time_ = now();
      });
    speed_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/fsds/gss", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message) {
        speed_ = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
        speed_time_ = now();
      });
    go_sub_ = create_subscription<fs_msgs::msg::GoSignal>(
      "/fsds/signal/go", 10,
      [this](const fs_msgs::msg::GoSignal::SharedPtr) {go_time_ = now();});
    command_pub_ = create_publisher<fs_msgs::msg::ControlCommand>(
      "/fsds/control_command", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/status", 10);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/autonomy/enable",
      std::bind(&PurePursuit::set_enabled, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&PurePursuit::control_loop, this));

    set_status(enabled_ ? "WAITING_FOR_INPUTS" : "DISABLED");
    RCLCPP_INFO(
      get_logger(),
      "C++ pure pursuit ready (%s); launch assist %.2f throttle, %.2f s, release %.2f m/s",
      enabled_ ? "ARMED" : "DISABLED", launch_throttle_, launch_timeout_,
      launch_release_speed_);
  }

  void publish_stop()
  {
    fs_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.throttle = 0.0;
    command.steering = 0.0;
    command.brake = 1.0;
    command_pub_->publish(command);
  }

private:
  static bool unit(const double value)
  {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
  }

  bool fresh(const std::optional<rclcpp::Time> & stamp, const double timeout) const
  {
    return stamp.has_value() && (now() - *stamp).seconds() <= timeout;
  }

  void set_enabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_ = request->data;
    launch_complete_ = false;
    launch_started_.reset();
    if (enabled_) {
      set_status("WAITING_FOR_INPUTS");
      response->message = "armed; waiting for fresh LiDAR path, GSS and GO";
      RCLCPP_WARN(get_logger(), "AUTONOMY ARMED");
    } else {
      set_status("DISABLED");
      publish_stop();
      response->message = "disabled and braking";
      RCLCPP_WARN(get_logger(), "AUTONOMY DISABLED");
    }
    response->success = true;
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

  void control_loop()
  {
    if (!enabled_) {
      set_status("DISABLED");
      publish_stop();
      return;
    }
    const bool go_is_fresh = !require_go_ || fresh(go_time_, go_timeout_);
    if (path_.size() < 2 || !fresh(path_time_, path_timeout_) ||
      !fresh(speed_time_, speed_timeout_) || !go_is_fresh)
    {
      set_status("WAITING_FOR_INPUTS");
      publish_stop();
      return;
    }

    const auto target = autonomy::select_lookahead(path_, lookahead_distance_);
    if (!target.has_value()) {
      set_status("NO_FORWARD_PATH");
      publish_stop();
      return;
    }
    const double curvature = autonomy::pure_pursuit_curvature(*target);
    const double steering_angle = std::atan(wheelbase_ * curvature);
    // FSDS uses negative steering for left; positive curvature means target-left.
    const double steering = std::clamp(
      -steering_angle / max_steering_angle_, -1.0, 1.0);

    if (!launch_complete_) {
      if (speed_ >= launch_release_speed_) {
        launch_complete_ = true;
        launch_started_.reset();
        RCLCPP_INFO(get_logger(), "Launch assist released at %.3f m/s", speed_);
      } else {
        if (!launch_started_.has_value()) {
          launch_started_ = now();
          set_status("LAUNCH_ASSIST");
        }
        if ((now() - *launch_started_).seconds() > launch_timeout_) {
          enabled_ = false;
          set_status("LAUNCH_FAILED_BRAKING");
          RCLCPP_ERROR(
            get_logger(), "Launch assist timed out at %.3f m/s; disarmed", speed_);
          publish_stop();
          return;
        }
        publish_drive(launch_throttle_, steering);
        return;
      }
    }

    if (speed_ >= max_speed_) {
      set_status("OVERSPEED_BRAKING");
      publish_command(0.0, steering, overspeed_brake_);
      return;
    }
    set_status("TRACKING");
    const double corner_amount = std::min(1.0, std::abs(curvature) / 0.45);
    const double base_throttle = straight_throttle_ +
      (corner_throttle_ - straight_throttle_) * corner_amount;
    const double speed_scale = std::clamp(
      (target_speed_ - speed_) / target_speed_, 0.0, 1.0);
    const double throttle = base_throttle * speed_scale;
    publish_drive(throttle, steering);
  }

  void publish_drive(const double throttle, const double steering)
  {
    publish_command(throttle, steering, 0.0);
  }

  void publish_command(const double throttle, const double steering, const double brake)
  {
    fs_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.throttle = std::clamp(throttle, 0.0, 1.0);
    command.steering = std::clamp(steering, -1.0, 1.0);
    command.brake = std::clamp(brake, 0.0, 1.0);
    command_pub_->publish(command);
  }

  bool enabled_;
  bool require_go_;
  bool launch_complete_{false};
  double lookahead_distance_;
  double wheelbase_;
  double max_steering_angle_;
  double straight_throttle_;
  double corner_throttle_;
  double target_speed_;
  double max_speed_;
  double overspeed_brake_;
  double launch_throttle_;
  double launch_release_speed_;
  double launch_timeout_;
  double path_timeout_;
  double speed_timeout_;
  double go_timeout_;
  double speed_{0.0};
  std::string status_;
  std::vector<autonomy::Point2> path_;
  std::optional<rclcpp::Time> path_time_;
  std::optional<rclcpp::Time> speed_time_;
  std::optional<rclcpp::Time> go_time_;
  std::optional<rclcpp::Time> launch_started_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr go_sub_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions options;
  rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::shared_ptr<PurePursuit> node;
  try {
    node = std::make_shared<PurePursuit>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pure_pursuit"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  while (rclcpp::ok() && !signal_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  for (int attempt = 0; attempt < 5; ++attempt) {
    node->publish_stop();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
