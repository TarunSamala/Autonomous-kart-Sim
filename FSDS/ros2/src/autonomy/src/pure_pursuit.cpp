#include "autonomy/geometry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fs_msgs/msg/extra_info.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
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
    controller_type_ = declare_parameter<std::string>("controller_type", "pure_pursuit");
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 2.5);
    wheelbase_ = declare_parameter<double>("wheelbase", 1.55);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.436332);
    target_speed_ = declare_parameter<double>("target_speed", 1.00);
    corner_speed_ = declare_parameter<double>("corner_speed", 0.70);
    max_speed_ = declare_parameter<double>("max_speed", 1.40);
    full_corner_curvature_ = declare_parameter<double>("full_corner_curvature", 0.35);
    speed_kp_ = declare_parameter<double>("speed_kp", 0.20);
    speed_ki_ = declare_parameter<double>("speed_ki", 0.03);
    speed_integral_limit_ = declare_parameter<double>("speed_integral_limit", 1.0);
    max_tracking_throttle_ = declare_parameter<double>("max_tracking_throttle", 0.16);
    max_tracking_brake_ = declare_parameter<double>("max_tracking_brake", 0.70);
    overspeed_brake_ = declare_parameter<double>("overspeed_brake", 1.00);
    launch_throttle_ = declare_parameter<double>("launch_throttle", 0.45);
    launch_release_speed_ = declare_parameter<double>("launch_release_speed", 0.10);
    launch_pulse_duration_ = declare_parameter<double>("launch_pulse_duration", 0.10);
    launch_timeout_ = declare_parameter<double>("launch_timeout", 0.75);
    path_timeout_ = declare_parameter<double>("path_timeout", 0.5);
    speed_timeout_ = declare_parameter<double>("speed_timeout", 0.5);
    go_timeout_ = declare_parameter<double>("go_timeout", 4.0);
    stop_on_cone_contact_ = declare_parameter<bool>("stop_on_cone_contact", true);
    stop_after_laps_ = declare_parameter<int>("stop_after_laps", 1);
    minimum_lap_distance_m_ = declare_parameter<double>("minimum_lap_distance_m", 100.0);
    extra_info_timeout_ = declare_parameter<double>("extra_info_timeout", 2.0);
    stanley_gain_ = declare_parameter<double>("stanley_gain", 1.0);
    stanley_softening_speed_ = declare_parameter<double>("stanley_softening_speed", 0.5);

    if ((controller_type_ != "pure_pursuit" && controller_type_ != "stanley") ||
      lookahead_distance_ <= 0.0 || wheelbase_ <= 0.0 ||
      max_steering_angle_ <= 0.0 || launch_release_speed_ <= 0.0 ||
      corner_speed_ <= launch_release_speed_ || target_speed_ < corner_speed_ ||
      max_speed_ <= target_speed_ || full_corner_curvature_ <= 0.0 ||
      speed_kp_ <= 0.0 || speed_ki_ < 0.0 || speed_integral_limit_ <= 0.0 ||
      launch_pulse_duration_ <= 0.0 || launch_timeout_ <= launch_pulse_duration_ ||
      path_timeout_ <= 0.0 ||
      speed_timeout_ <= 0.0 || go_timeout_ <= 0.0 || extra_info_timeout_ <= 0.0 ||
      stop_after_laps_ < 0 || minimum_lap_distance_m_ < 0.0 ||
      !std::isfinite(stanley_gain_) || stanley_gain_ <= 0.0 ||
      !std::isfinite(stanley_softening_speed_) || stanley_softening_speed_ <= 0.0 ||
      !unit(max_tracking_throttle_) || !unit(max_tracking_brake_) ||
      !unit(launch_throttle_) || !unit(overspeed_brake_))
    {
      throw std::invalid_argument("invalid path controller parameters");
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
        const auto sample_time = now();
        const double measured_speed = std::hypot(
          message->twist.twist.linear.x, message->twist.twist.linear.y);
        if (enabled_ && distance_sample_time_.has_value()) {
          const double dt = (sample_time - *distance_sample_time_).seconds();
          if (dt > 0.0 && dt <= 1.0) {
            distance_since_lap_baseline_m_ += 0.5 * (speed_ + measured_speed) * dt;
          }
        }
        speed_ = measured_speed;
        speed_time_ = sample_time;
        distance_sample_time_ = sample_time;
      });
    go_sub_ = create_subscription<fs_msgs::msg::GoSignal>(
      "/fsds/signal/go", 10,
      [this](const fs_msgs::msg::GoSignal::SharedPtr) {go_time_ = now();});
    extra_info_sub_ = create_subscription<fs_msgs::msg::ExtraInfo>(
      "/fsds/testing_only/extra_info", 10,
      std::bind(&PurePursuit::extra_info_callback, this, std::placeholders::_1));
    finished_sub_ = create_subscription<fs_msgs::msg::FinishedSignal>(
      "/fsds/signal/finished", 10,
      [this](const fs_msgs::msg::FinishedSignal::SharedPtr) {
        if (!enabled_) {
          return;
        }
        enabled_ = false;
        stop_latched_ = true;
        set_status("FINISHED_BRAKING");
        publish_stop();
        RCLCPP_INFO(get_logger(), "FSDS finished signal received; autonomy stopped");
      });
    command_pub_ = create_publisher<fs_msgs::msg::ControlCommand>(
      "/fsds/control_command", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/status", 10);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/autonomy/enable",
      std::bind(&PurePursuit::set_enabled, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&PurePursuit::control_loop, this));

    set_status(enabled_ ? "WAITING_FOR_INPUTS" : "DISABLED");
    RCLCPP_INFO(
      get_logger(),
      "C++ %s controller ready (%s); launch pulse %.2f throttle for %.2f s, release %.2f m/s",
      controller_type_.c_str(), enabled_ ? "ARMED" : "DISABLED",
      launch_throttle_, launch_pulse_duration_,
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
    launch_complete_ = false;
    launch_started_.reset();
    last_control_time_.reset();
    speed_integral_ = 0.0;
    distance_since_lap_baseline_m_ = 0.0;
    distance_sample_time_.reset();
    if (request->data) {
      enabled_ = true;
      stop_latched_ = false;
      cone_count_at_arm_ = latest_cone_count_;
      lap_count_at_arm_ = latest_lap_count_;
      set_status("WAITING_FOR_INPUTS");
      response->message = "armed; waiting for fresh path, speed, GO and safety telemetry";
      RCLCPP_WARN(get_logger(), "AUTONOMY ARMED");
    } else {
      enabled_ = false;
      stop_latched_ = false;
      cone_count_at_arm_.reset();
      lap_count_at_arm_.reset();
      set_status("DISABLED");
      publish_stop();
      response->message = "disabled and braking";
      RCLCPP_WARN(get_logger(), "AUTONOMY DISABLED");
    }
    response->success = true;
  }

  void extra_info_callback(const fs_msgs::msg::ExtraInfo::SharedPtr message)
  {
    latest_cone_count_ = message->doo_counter;
    latest_lap_count_ = message->laps.size();
    extra_info_time_ = now();
    if (!enabled_) {
      return;
    }
    if (!cone_count_at_arm_.has_value()) {
      cone_count_at_arm_ = message->doo_counter;
    }
    if (!lap_count_at_arm_.has_value()) {
      lap_count_at_arm_ = message->laps.size();
    }
    if (stop_on_cone_contact_ && message->doo_counter > *cone_count_at_arm_) {
      enabled_ = false;
      stop_latched_ = true;
      set_status("CONE_CONTACT_BRAKING");
      publish_stop();
      RCLCPP_ERROR(
        get_logger(), "Cone counter increased from %u to %u; autonomy disarmed",
        *cone_count_at_arm_, message->doo_counter);
      return;
    }
    if (stop_after_laps_ > 0 &&
      message->laps.size() >= *lap_count_at_arm_ + static_cast<std::size_t>(stop_after_laps_))
    {
      if (distance_since_lap_baseline_m_ < minimum_lap_distance_m_) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring start-line lap event after only %.1f m (minimum %.1f m)",
          distance_since_lap_baseline_m_, minimum_lap_distance_m_);
        lap_count_at_arm_ = message->laps.size();
        distance_since_lap_baseline_m_ = 0.0;
        return;
      }
      enabled_ = false;
      stop_latched_ = true;
      set_status("LAP_TARGET_REACHED_BRAKING");
      publish_stop();
      RCLCPP_INFO(
        get_logger(), "Completed %d requested lap(s); autonomy stopped", stop_after_laps_);
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

  void control_loop()
  {
    if (!enabled_) {
      if (!stop_latched_) {
        set_status("DISABLED");
      }
      publish_stop();
      return;
    }
    if (stop_on_cone_contact_ &&
      (!cone_count_at_arm_.has_value() || !fresh(extra_info_time_, extra_info_timeout_)))
    {
      set_status("WAITING_FOR_SAFETY_TELEMETRY");
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
    double steering_angle = std::atan(wheelbase_ * curvature);
    if (controller_type_ == "stanley") {
      const auto stanley_angle = autonomy::stanley_steering_angle(
        path_, speed_, stanley_gain_, stanley_softening_speed_);
      if (!stanley_angle.has_value()) {
        set_status("NO_VALID_PATH_SEGMENT");
        publish_stop();
        return;
      }
      steering_angle = *stanley_angle;
    }
    // FSDS uses negative steering for left; positive curvature means target-left.
    const double steering = std::clamp(
      -steering_angle / max_steering_angle_, -1.0, 1.0);

    if (!launch_complete_) {
      if (!launch_started_.has_value()) {
        launch_started_ = now();
        set_status("LAUNCH_ASSIST");
      }
      if (speed_ >= launch_release_speed_) {
        launch_complete_ = true;
        launch_started_.reset();
        last_control_time_ = now();
        speed_integral_ = 0.0;
        RCLCPP_INFO(get_logger(), "Launch assist released at %.3f m/s", speed_);
      } else {
        const double launch_elapsed = (now() - *launch_started_).seconds();
        if (launch_elapsed > launch_timeout_) {
          enabled_ = false;
          stop_latched_ = true;
          set_status("LAUNCH_FAILED_BRAKING");
          RCLCPP_ERROR(
            get_logger(), "Launch assist timed out at %.3f m/s; disarmed", speed_);
          publish_stop();
          return;
        }
        if (launch_elapsed <= launch_pulse_duration_) {
          publish_drive(launch_throttle_, steering);
        } else {
          set_status("LAUNCH_SETTLING");
          publish_command(0.0, steering, 0.0);
        }
        return;
      }
    }

    if (speed_ >= max_speed_) {
      speed_integral_ = std::min(0.0, speed_integral_);
      last_control_time_ = now();
      set_status("OVERSPEED_BRAKING");
      publish_command(0.0, steering, overspeed_brake_);
      return;
    }

    const auto current_time = now();
    double dt = 0.02;
    if (last_control_time_.has_value()) {
      dt = std::clamp((current_time - *last_control_time_).seconds(), 0.001, 0.10);
    }
    last_control_time_ = current_time;

    const double corner_amount = std::min(
      1.0, std::abs(curvature) / full_corner_curvature_);
    const double desired_speed = target_speed_ +
      (corner_speed_ - target_speed_) * corner_amount;
    const double speed_error = desired_speed - speed_;
    if ((speed_error > 0.0 && speed_integral_ < 0.0) ||
      (speed_error < 0.0 && speed_integral_ > 0.0))
    {
      speed_integral_ = 0.0;
    }
    speed_integral_ = std::clamp(
      speed_integral_ + speed_error * dt,
      -speed_integral_limit_, speed_integral_limit_);
    const double effort = speed_kp_ * speed_error + speed_ki_ * speed_integral_;
    const double throttle = effort > 0.0 ?
      std::min(effort, max_tracking_throttle_) : 0.0;
    const double brake = effort < 0.0 ?
      std::min(-effort, max_tracking_brake_) : 0.0;

    set_status("TRACKING");
    publish_command(throttle, steering, brake);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed=%.2f target=%.2f throttle=%.2f brake=%.2f steering=%.2f curvature=%.3f",
      speed_, desired_speed, throttle, brake, steering, curvature);
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
  std::string controller_type_;
  bool launch_complete_{false};
  bool stop_on_cone_contact_;
  bool stop_latched_{false};
  int stop_after_laps_;
  double minimum_lap_distance_m_;
  double lookahead_distance_;
  double wheelbase_;
  double max_steering_angle_;
  double target_speed_;
  double corner_speed_;
  double max_speed_;
  double full_corner_curvature_;
  double speed_kp_;
  double speed_ki_;
  double speed_integral_limit_;
  double max_tracking_throttle_;
  double max_tracking_brake_;
  double overspeed_brake_;
  double launch_throttle_;
  double launch_release_speed_;
  double launch_pulse_duration_;
  double launch_timeout_;
  double path_timeout_;
  double speed_timeout_;
  double go_timeout_;
  double extra_info_timeout_;
  double stanley_gain_;
  double stanley_softening_speed_;
  double speed_{0.0};
  double speed_integral_{0.0};
  double distance_since_lap_baseline_m_{0.0};
  std::string status_;
  std::vector<autonomy::Point2> path_;
  std::optional<rclcpp::Time> path_time_;
  std::optional<rclcpp::Time> speed_time_;
  std::optional<rclcpp::Time> distance_sample_time_;
  std::optional<rclcpp::Time> go_time_;
  std::optional<rclcpp::Time> extra_info_time_;
  std::optional<rclcpp::Time> launch_started_;
  std::optional<rclcpp::Time> last_control_time_;
  std::optional<uint32_t> latest_cone_count_;
  std::optional<uint32_t> cone_count_at_arm_;
  std::optional<std::size_t> latest_lap_count_;
  std::optional<std::size_t> lap_count_at_arm_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr go_sub_;
  rclcpp::Subscription<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_sub_;
  rclcpp::Subscription<fs_msgs::msg::FinishedSignal>::SharedPtr finished_sub_;
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
