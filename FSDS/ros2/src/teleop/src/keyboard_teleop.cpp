#include <X11/Xlib.h>
#include <X11/keysym.h>

// Xlib defines None as a macro, which conflicts with rclcpp's enum value.
#ifdef None
#undef None
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "fs_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
volatile std::sig_atomic_t signal_requested = 0;

void signal_handler(int)
{
  signal_requested = 1;
}

double move_towards(double current, double target, double max_delta)
{
  if (current < target) {
    return std::min(current + max_delta, target);
  }
  return std::max(current - max_delta, target);
}
}  // namespace

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop"), display_(XOpenDisplay(nullptr))
  {
    if (display_ == nullptr) {
      throw std::runtime_error(
              "Could not open the X11 display. Check DISPLAY/XAUTHORITY and Distrobox X11 access.");
    }

    control_topic_ = declare_parameter<std::string>("control_topic", "/fsds/control_command");
    publish_rate_ = positive_parameter("publish_rate", 60.0);
    max_throttle_ = unit_parameter("max_throttle", 1.0);
    max_brake_ = unit_parameter("max_brake", 1.0);
    max_steering_ = unit_parameter("max_steering", 1.0);
    throttle_rise_rate_ = positive_parameter("throttle_rise_rate", 1.5);
    throttle_fall_rate_ = positive_parameter("throttle_fall_rate", 3.0);
    brake_rise_rate_ = positive_parameter("brake_rise_rate", 5.0);
    brake_fall_rate_ = positive_parameter("brake_fall_rate", 7.0);
    steering_rate_ = positive_parameter("steering_rate", 2.5);
    steering_return_rate_ = positive_parameter("steering_return_rate", 4.0);

    key_w_ = keycode(XK_w);
    key_up_ = keycode(XK_Up);
    key_s_ = keycode(XK_s);
    key_down_ = keycode(XK_Down);
    key_a_ = keycode(XK_a);
    key_left_ = keycode(XK_Left);
    key_d_ = keycode(XK_d);
    key_right_ = keycode(XK_Right);
    key_space_ = keycode(XK_space);
    key_enable_ = keycode(XK_e);
    key_quit_ = keycode(XK_q);
    key_escape_ = keycode(XK_Escape);

    publisher_ = create_publisher<fs_msgs::msg::ControlCommand>(control_topic_, 1);
    last_update_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&KeyboardTeleop::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Keyboard teleop ready (DISARMED). E: arm | W/S: throttle/brake | "
      "A/D: steer | Space: emergency brake | Q/Esc: quit");
  }

  ~KeyboardTeleop() override
  {
    if (display_ != nullptr) {
      XCloseDisplay(display_);
    }
  }

  bool exit_requested() const
  {
    return exit_requested_;
  }

  void publish_stop()
  {
    fs_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.throttle = 0.0;
    command.steering = 0.0;
    command.brake = 1.0;
    publisher_->publish(command);
  }

private:
  using KeyMap = char[32];

  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and greater than zero");
    }
    return value;
  }

  double unit_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::invalid_argument(name + " must be within [0, 1]");
    }
    return value;
  }

  KeyCode keycode(KeySym symbol) const
  {
    const KeyCode code = XKeysymToKeycode(display_, symbol);
    if (code == 0) {
      throw std::runtime_error("The current X11 keymap does not contain a required key");
    }
    return code;
  }

  static bool key_down(const KeyMap & keys, KeyCode code)
  {
    return (keys[code / 8] & (1 << (code % 8))) != 0;
  }

  static bool either_down(const KeyMap & keys, KeyCode first, KeyCode second)
  {
    return key_down(keys, first) || key_down(keys, second);
  }

  void update()
  {
    KeyMap keys{};
    XQueryKeymap(display_, keys);

    const bool enable_down = key_down(keys, key_enable_);
    if (enable_down && !enable_was_down_) {
      armed_ = !armed_;
      throttle_ = 0.0;
      brake_ = armed_ ? 0.0 : 1.0;
      RCLCPP_INFO(get_logger(), "Keyboard control %s", armed_ ? "ARMED" : "DISARMED");
    }
    enable_was_down_ = enable_down;

    const bool quit_down = key_down(keys, key_quit_) || key_down(keys, key_escape_);
    if (quit_down && !quit_was_down_) {
      armed_ = false;
      exit_requested_ = true;
    }
    quit_was_down_ = quit_down;

    const auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_update_).count();
    last_update_ = current_time;
    dt = std::clamp(dt, 0.0, 0.1);

    if (!armed_ || exit_requested_) {
      throttle_ = 0.0;
      brake_ = 1.0;
      steering_ = move_towards(steering_, 0.0, steering_return_rate_ * dt);
      publish_command();
      return;
    }

    const bool throttle_key = either_down(keys, key_w_, key_up_);
    const bool brake_key = either_down(keys, key_s_, key_down_);
    const bool left_key = either_down(keys, key_a_, key_left_);
    const bool right_key = either_down(keys, key_d_, key_right_);
    const bool emergency_brake = key_down(keys, key_space_);

    if (emergency_brake || brake_key) {
      throttle_ = move_towards(throttle_, 0.0, throttle_fall_rate_ * dt);
      const double target_brake = emergency_brake ? 1.0 : max_brake_;
      brake_ = emergency_brake ? 1.0 :
        move_towards(brake_, target_brake, brake_rise_rate_ * dt);
    } else {
      brake_ = move_towards(brake_, 0.0, brake_fall_rate_ * dt);
      const double target_throttle = throttle_key ? max_throttle_ : 0.0;
      const double throttle_rate = throttle_key ? throttle_rise_rate_ : throttle_fall_rate_;
      throttle_ = move_towards(throttle_, target_throttle, throttle_rate * dt);
    }

    double steering_target = 0.0;
    if (left_key != right_key) {
      steering_target = left_key ? -max_steering_ : max_steering_;
    }
    const double steering_change_rate =
      steering_target == 0.0 ? steering_return_rate_ : steering_rate_;
    steering_ = move_towards(steering_, steering_target, steering_change_rate * dt);

    publish_command();
  }

  void publish_command()
  {
    fs_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.throttle = std::clamp(throttle_, 0.0, 1.0);
    command.steering = std::clamp(steering_, -1.0, 1.0);
    command.brake = std::clamp(brake_, 0.0, 1.0);
    publisher_->publish(command);
  }

  Display * display_{nullptr};
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_update_;

  std::string control_topic_;
  double publish_rate_{60.0};
  double max_throttle_{1.0};
  double max_brake_{1.0};
  double max_steering_{1.0};
  double throttle_rise_rate_{1.5};
  double throttle_fall_rate_{3.0};
  double brake_rise_rate_{5.0};
  double brake_fall_rate_{7.0};
  double steering_rate_{2.5};
  double steering_return_rate_{4.0};
  double throttle_{0.0};
  double brake_{1.0};
  double steering_{0.0};

  bool armed_{false};
  bool enable_was_down_{false};
  bool quit_was_down_{false};
  bool exit_requested_{false};

  KeyCode key_w_{0};
  KeyCode key_up_{0};
  KeyCode key_s_{0};
  KeyCode key_down_{0};
  KeyCode key_a_{0};
  KeyCode key_left_{0};
  KeyCode key_d_{0};
  KeyCode key_right_{0};
  KeyCode key_space_{0};
  KeyCode key_enable_{0};
  KeyCode key_quit_{0};
  KeyCode key_escape_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::shared_ptr<KeyboardTeleop> node;
  try {
    node = std::make_shared<KeyboardTeleop>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_teleop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  while (rclcpp::ok() && !signal_requested && !node->exit_requested()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // Send several stop commands while DDS is still alive so FSDS cannot retain
  // a stale throttle command when the teleop process exits.
  for (int i = 0; i < 5; ++i) {
    node->publish_stop();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
