#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "fs_msgs/msg/extra_info.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace std::chrono_literals;

class SingleTestSession : public rclcpp::Node
{
public:
  SingleTestSession()
  : Node("single_test_session")
  {
    manifest_path_ = declare_parameter<std::string>("manifest_path", "");
    results_directory_ = declare_parameter<std::string>("results_directory", "results");
    const bool auto_start = declare_parameter<bool>("auto_start", false);
    if (manifest_path_.empty()) {
      throw std::invalid_argument("manifest_path must not be empty");
    }
    std::ifstream manifest_stream(manifest_path_);
    if (!manifest_stream) {
      throw std::invalid_argument("cannot open manifest: " + manifest_path_);
    }
    manifest_ = json::parse(manifest_stream);
    test_id_ = manifest_.at("test").at("id").get<std::string>();
    max_duration_s_ = manifest_.at("test").at("max_duration_s").get<double>();
    if (!std::isfinite(max_duration_s_) || max_duration_s_ <= 0.0) {
      throw std::invalid_argument("manifest max_duration_s must be positive");
    }

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/single_test/status", rclcpp::QoS(1).reliable().transient_local());
    speed_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/fsds/gss", rclcpp::SensorDataQoS(),
      std::bind(&SingleTestSession::speed_callback, this, std::placeholders::_1));
    extra_info_sub_ = create_subscription<fs_msgs::msg::ExtraInfo>(
      "/fsds/testing_only/extra_info", 10,
      std::bind(&SingleTestSession::extra_info_callback, this, std::placeholders::_1));
    finished_sub_ = create_subscription<fs_msgs::msg::FinishedSignal>(
      "/fsds/signal/finished", 10,
      [this](const fs_msgs::msg::FinishedSignal::SharedPtr) {
        if (running_) {
          finish("finished_signal");
        }
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/single_test/start",
      std::bind(
        &SingleTestSession::start_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/single_test/stop",
      std::bind(
        &SingleTestSession::stop_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(100ms, std::bind(&SingleTestSession::timer_callback, this));

    set_status("READY");
    RCLCPP_INFO(
      get_logger(), "Single Test '%s' ready; manual operation, %.1f s limit",
      test_id_.c_str(), max_duration_s_);
    if (auto_start) {
      start();
    }
  }

private:
  using SteadyClock = std::chrono::steady_clock;

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

  void start_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (running_) {
      response->success = false;
      response->message = "test is already running";
      return;
    }
    start();
    response->success = true;
    response->message = "test started";
  }

  void stop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!running_) {
      response->success = false;
      response->message = "test is not running";
      return;
    }
    const auto path = finish("manual_stop");
    response->success = true;
    response->message = "result written to " + path;
  }

  void start()
  {
    running_ = true;
    started_at_ = SteadyClock::now();
    last_speed_at_.reset();
    distance_m_ = 0.0;
    speed_sum_ = 0.0;
    max_speed_mps_ = 0.0;
    speed_samples_ = 0;
    down_or_out_cones_ = 0;
    lap_times_.clear();
    set_status("RUNNING");
  }

  void speed_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message)
  {
    if (!running_) {
      return;
    }
    const double speed = std::hypot(
      message->twist.twist.linear.x, message->twist.twist.linear.y);
    const auto current = SteadyClock::now();
    if (last_speed_at_) {
      const double delta = std::chrono::duration<double>(current - *last_speed_at_).count();
      if (delta >= 0.0 && delta <= 1.0) {
        distance_m_ += speed * delta;
      }
    }
    last_speed_at_ = current;
    speed_sum_ += speed;
    max_speed_mps_ = std::max(max_speed_mps_, speed);
    ++speed_samples_;
  }

  void extra_info_callback(const fs_msgs::msg::ExtraInfo::SharedPtr message)
  {
    if (!running_) {
      return;
    }
    down_or_out_cones_ = message->doo_counter;
    lap_times_.assign(message->laps.begin(), message->laps.end());
  }

  void timer_callback()
  {
    if (running_ && elapsed_s() >= max_duration_s_) {
      finish("time_limit");
    }
  }

  double elapsed_s() const
  {
    return std::chrono::duration<double>(SteadyClock::now() - started_at_).count();
  }

  std::string finish(const std::string & outcome)
  {
    const double duration_s = elapsed_s();
    running_ = false;
    set_status("WRITING_RESULT");
    const double average_speed = speed_samples_ > 0 ? speed_sum_ / speed_samples_ : 0.0;
    const auto unix_seconds = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const fs::path result_path = fs::absolute(results_directory_) /
      (test_id_ + "_" + std::to_string(unix_seconds) + ".json");
    fs::create_directories(result_path.parent_path());
    json result = {
      {"schema_version", 1},
      {"test_id", test_id_},
      {"outcome", outcome},
      {"completed_unix_s", unix_seconds},
      {"duration_s", duration_s},
      {"distance_m", distance_m_},
      {"average_speed_mps", average_speed},
      {"max_speed_mps", max_speed_mps_},
      {"speed_samples", speed_samples_},
      {"down_or_out_cones", down_or_out_cones_},
      {"lap_times_s", lap_times_},
      {"manifest", manifest_}
    };
    std::ofstream stream(result_path);
    if (!stream) {
      set_status("FAILED");
      throw std::runtime_error("cannot write result: " + result_path.string());
    }
    stream << std::setw(2) << result << '\n';
    last_result_path_ = result_path.string();
    set_status("COMPLETE");
    RCLCPP_INFO(get_logger(), "Result: %s", last_result_path_.c_str());
    return last_result_path_;
  }

  std::string manifest_path_;
  std::string results_directory_;
  std::string test_id_;
  std::string status_;
  std::string last_result_path_;
  double max_duration_s_{};
  bool running_{false};
  SteadyClock::time_point started_at_{};
  std::optional<SteadyClock::time_point> last_speed_at_;
  double distance_m_{0.0};
  double speed_sum_{0.0};
  double max_speed_mps_{0.0};
  std::size_t speed_samples_{0};
  uint32_t down_or_out_cones_{0};
  std::vector<float> lap_times_;
  json manifest_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_sub_;
  rclcpp::Subscription<fs_msgs::msg::FinishedSignal>::SharedPtr finished_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SingleTestSession>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("single_test_session"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
