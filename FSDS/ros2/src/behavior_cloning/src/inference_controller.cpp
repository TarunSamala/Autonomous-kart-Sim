#include "behavior_cloning/preprocessing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nlohmann/json.hpp>
#include <opencv2/dnn.hpp>
#include <rmw/qos_profiles.h>

#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/extra_info.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using json = nlohmann::json;

namespace
{

volatile std::sig_atomic_t signal_requested = 0;

void signal_handler(int)
{
  signal_requested = 1;
}

struct Prediction
{
  double steering{};
  double throttle{};
  double brake{};
};

double move_towards(const double current, const double target, const double maximum_delta)
{
  if (current < target) {
    return std::min(current + maximum_delta, target);
  }
  return std::max(current - maximum_delta, target);
}

}  // namespace

class InferenceController : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  InferenceController()
  : Node("behavior_cloning_inference")
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    metadata_path_ = declare_parameter<std::string>("metadata_path", "");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/behavior_cloning/predicted_command");
    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/fsds/front_rgb/image_color");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/fsds/front_depth/image_depth");
    const auto speed_topic = declare_parameter<std::string>("speed_topic", "/fsds/gss");
    const auto extra_info_topic = declare_parameter<std::string>(
      "extra_info_topic", "/fsds/testing_only/extra_info");
    enabled_ = declare_parameter<bool>("enabled", false);
    hold_brake_when_disabled_ = declare_parameter<bool>("hold_brake_when_disabled", false);
    require_safety_telemetry_ = declare_parameter<bool>("require_safety_telemetry", true);
    stop_on_cone_contact_ = declare_parameter<bool>("stop_on_cone_contact", true);
    stop_after_laps_ = declare_parameter<int>("stop_after_laps", 1);
    minimum_lap_distance_m_ = declare_parameter<double>("minimum_lap_distance_m", 100.0);
    prediction_timeout_s_ = declare_parameter<double>("prediction_timeout_s", 0.25);
    speed_timeout_s_ = declare_parameter<double>("speed_timeout_s", 0.50);
    safety_timeout_s_ = declare_parameter<double>("safety_timeout_s", 2.0);
    maximum_throttle_ = declare_parameter<double>("maximum_throttle", 0.35);
    maximum_brake_ = declare_parameter<double>("maximum_brake", 1.0);
    maximum_steering_ = declare_parameter<double>("maximum_steering", 1.0);
    steering_rate_ = declare_parameter<double>("steering_rate", 3.0);
    throttle_rate_ = declare_parameter<double>("throttle_rate", 2.0);
    brake_rate_ = declare_parameter<double>("brake_rate", 5.0);
    launch_throttle_ = declare_parameter<double>("launch_throttle", 0.45);
    launch_release_speed_ = declare_parameter<double>("launch_release_speed", 0.10);
    launch_pulse_duration_s_ = declare_parameter<double>("launch_pulse_duration_s", 0.10);
    launch_timeout_s_ = declare_parameter<double>("launch_timeout_s", 0.75);

    load_model_contract();
    validate_parameters();

    command_pub_ = create_publisher<fs_msgs::msg::ControlCommand>(command_topic_, 1);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/behavior_cloning/inference/status", rclcpp::QoS(1).reliable().transient_local());
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/behavior_cloning/enable",
      std::bind(
        &InferenceController::set_enabled, this,
        std::placeholders::_1, std::placeholders::_2));

    speed_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      speed_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message) {
        const auto current_time = now();
        const double measured_speed = std::hypot(
          message->twist.twist.linear.x, message->twist.twist.linear.y);
        if (enabled_ && speed_received_) {
          const double dt = (current_time - *speed_received_).seconds();
          if (dt > 0.0 && dt <= 1.0) {
            distance_since_arm_m_ += 0.5 * (speed_mps_ + measured_speed) * dt;
          }
        }
        speed_mps_ = measured_speed;
        speed_received_ = current_time;
      });
    extra_info_sub_ = create_subscription<fs_msgs::msg::ExtraInfo>(
      extra_info_topic, 10,
      std::bind(&InferenceController::extra_info_callback, this, std::placeholders::_1));

    if (modality_ == "rgbd") {
      rgb_filter_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
      depth_filter_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
      synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(30), rgb_filter_sub_, depth_filter_sub_);
      synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.04));
      synchronizer_->registerCallback(
        std::bind(
          &InferenceController::rgbd_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {
      rgb_sub_ = create_subscription<Image>(
        rgb_topic_, rclcpp::SensorDataQoS(),
        std::bind(&InferenceController::rgb_callback, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&InferenceController::control_loop, this));
    set_status(enabled_ ? "WAITING_FOR_INPUTS" : "DISABLED");
    if (enabled_) {
      arm_baselines();
    }
    RCLCPP_INFO(
      get_logger(), "%s behavior model loaded; output=%s; startup=%s",
      modality_.c_str(), command_topic_.c_str(), enabled_ ? "ENABLED" : "DISABLED");
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
  void load_model_contract()
  {
    if (model_path_.empty() || metadata_path_.empty()) {
      throw std::invalid_argument("model_path and metadata_path are required");
    }
    if (!std::filesystem::is_regular_file(model_path_) ||
      !std::filesystem::is_regular_file(metadata_path_))
    {
      throw std::invalid_argument("model or metadata file does not exist");
    }
    std::ifstream stream(metadata_path_);
    const json metadata = json::parse(stream);
    if (metadata.at("schema_version").get<int>() != 1) {
      throw std::invalid_argument("unsupported behavior model metadata schema");
    }
    modality_ = metadata.at("modality").get<std::string>();
    input_width_ = metadata.at("input_width").get<int>();
    input_height_ = metadata.at("input_height").get<int>();
    minimum_depth_m_ = metadata.at("minimum_depth_m").get<float>();
    maximum_depth_m_ = metadata.at("maximum_depth_m").get<float>();
    rgb_horizontal_fov_deg_ = metadata.at("rgb_horizontal_fov_deg").get<double>();
    depth_horizontal_fov_deg_ = metadata.at("depth_horizontal_fov_deg").get<double>();
    model_maximum_speed_mps_ = metadata.at("maximum_speed_mps").get<double>();
    if (modality_ != "rgb" && modality_ != "rgbd") {
      throw std::invalid_argument("model modality must be rgb or rgbd");
    }
    net_ = cv::dnn::readNetFromONNX(model_path_);
    if (net_.empty()) {
      throw std::runtime_error("OpenCV could not load the ONNX model");
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }

  void validate_parameters() const
  {
    const auto unit = [](const double value) {
        return std::isfinite(value) && value >= 0.0 && value <= 1.0;
      };
    if (input_width_ <= 0 || input_height_ <= 0 || minimum_depth_m_ <= 0.0F ||
      maximum_depth_m_ <= minimum_depth_m_ || model_maximum_speed_mps_ <= 0.0 ||
      prediction_timeout_s_ <= 0.0 || speed_timeout_s_ <= 0.0 || safety_timeout_s_ <= 0.0 ||
      steering_rate_ <= 0.0 || throttle_rate_ <= 0.0 || brake_rate_ <= 0.0 ||
      launch_release_speed_ <= 0.0 || launch_pulse_duration_s_ <= 0.0 ||
      launch_timeout_s_ <= launch_pulse_duration_s_ || stop_after_laps_ < 0 ||
      minimum_lap_distance_m_ < 0.0 || !unit(maximum_throttle_) ||
      !unit(maximum_brake_) || !unit(maximum_steering_) || !unit(launch_throttle_))
    {
      throw std::invalid_argument("invalid inference controller parameters");
    }
  }

  void set_status(const std::string & value)
  {
    if (status_ == value) {
      return;
    }
    status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "State: %s", value.c_str());
  }

  void arm_baselines()
  {
    cone_count_at_arm_ = latest_cone_count_;
    lap_count_at_arm_ = latest_lap_count_;
    distance_since_arm_m_ = 0.0;
    launch_started_.reset();
    launch_complete_ = false;
    previous_control_time_.reset();
    steering_command_ = 0.0;
    throttle_command_ = 0.0;
    brake_command_ = 0.0;
  }

  void set_enabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      const auto publisher_count = count_publishers(command_topic_);
      if (publisher_count != 1) {
        response->success = false;
        response->message =
          "command topic must have only this controller; found " +
          std::to_string(publisher_count) + " publishers";
        return;
      }
      if (!prediction_ || !fresh(prediction_time_, prediction_timeout_s_) ||
        !fresh(speed_received_, speed_timeout_s_) ||
        (require_safety_telemetry_ && !fresh(extra_info_received_, safety_timeout_s_)))
      {
        response->success = false;
        response->message = "cannot arm until model, speed and safety telemetry are fresh";
        return;
      }
      enabled_ = true;
      stop_latched_ = false;
      arm_baselines();
      set_status("ARMED");
      response->success = true;
      response->message = "behavior-cloning controller armed";
      RCLCPP_WARN(get_logger(), "BEHAVIOR CLONING ARMED on %s", command_topic_.c_str());
    } else {
      enabled_ = false;
      stop_latched_ = false;
      set_status("DISABLED");
      publish_stop();
      response->success = true;
      response->message = "controller disabled and braking";
    }
  }

  bool fresh(const std::optional<rclcpp::Time> & time, const double timeout) const
  {
    return time.has_value() && (now() - *time).seconds() <= timeout;
  }

  void extra_info_callback(const fs_msgs::msg::ExtraInfo::SharedPtr message)
  {
    latest_cone_count_ = message->doo_counter;
    latest_lap_count_ = message->laps.size();
    extra_info_received_ = now();
    if (!enabled_) {
      return;
    }
    if (!cone_count_at_arm_) {
      cone_count_at_arm_ = latest_cone_count_;
    }
    if (!lap_count_at_arm_) {
      lap_count_at_arm_ = latest_lap_count_;
    }
    if (stop_on_cone_contact_ && *latest_cone_count_ > *cone_count_at_arm_) {
      latch_stop("CONE_CONTACT_BRAKING");
      RCLCPP_ERROR(get_logger(), "Cone contact detected; behavior controller disarmed");
      return;
    }
    if (stop_after_laps_ > 0 &&
      *latest_lap_count_ >= *lap_count_at_arm_ + static_cast<std::size_t>(stop_after_laps_))
    {
      if (distance_since_arm_m_ < minimum_lap_distance_m_) {
        lap_count_at_arm_ = latest_lap_count_;
        distance_since_arm_m_ = 0.0;
        RCLCPP_WARN(
          get_logger(), "Ignoring short start-line event before %.1f m",
          minimum_lap_distance_m_);
        return;
      }
      latch_stop("LAP_TARGET_REACHED_BRAKING");
    }
  }

  cv::Mat depth_in_metres(const Image::ConstSharedPtr & message) const
  {
    if (message->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return cv_bridge::toCvShare(message, message->encoding)->image.clone();
    }
    if (message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      message->encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv::Mat metres;
      cv_bridge::toCvShare(message, message->encoding)->image.convertTo(
        metres, CV_32FC1, 0.001);
      return metres;
    }
    throw std::runtime_error("unsupported depth encoding: " + message->encoding);
  }

  void rgb_callback(const Image::ConstSharedPtr message)
  {
    run_inference(message, nullptr);
  }

  void rgbd_callback(
    const Image::ConstSharedPtr & rgb_message,
    const Image::ConstSharedPtr & depth_message)
  {
    try {
      const cv::Mat depth_m = depth_in_metres(depth_message);
      run_inference(rgb_message, &depth_m);
    } catch (const std::exception & error) {
      inference_failure(error.what());
    }
  }

  void run_inference(const Image::ConstSharedPtr & rgb_message, const cv::Mat * depth_m)
  {
    try {
      const cv::Mat bgr = cv_bridge::toCvCopy(
        rgb_message, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat image_blob = behavior_cloning::make_image_blob(
        bgr, depth_m, input_width_, input_height_, minimum_depth_m_, maximum_depth_m_,
        rgb_horizontal_fov_deg_, depth_horizontal_fov_deg_);
      cv::Mat state_blob(1, 1, CV_32F);
      state_blob.at<float>(0, 0) = static_cast<float>(
        std::clamp(speed_mps_ / model_maximum_speed_mps_, 0.0, 2.0));
      net_.setInput(image_blob, "image");
      net_.setInput(state_blob, "state");
      cv::Mat output = net_.forward();
      output = output.reshape(1, 1);
      if (output.total() < 3) {
        throw std::runtime_error("model output must contain steering, throttle and brake");
      }
      const float * values = output.ptr<float>();
      if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
        !std::isfinite(values[2]))
      {
        throw std::runtime_error("model produced a non-finite control");
      }
      prediction_ = Prediction{
        std::clamp(static_cast<double>(values[0]), -1.0, 1.0),
        std::clamp(static_cast<double>(values[1]), 0.0, 1.0),
        std::clamp(static_cast<double>(values[2]), 0.0, 1.0)};
      prediction_time_ = now();
    } catch (const std::exception & error) {
      inference_failure(error.what());
    }
  }

  void inference_failure(const std::string & reason)
  {
    prediction_.reset();
    if (enabled_) {
      latch_stop("INFERENCE_ERROR_BRAKING");
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Behavior model inference failed: %s", reason.c_str());
  }

  void latch_stop(const std::string & reason)
  {
    enabled_ = false;
    stop_latched_ = true;
    set_status(reason);
    publish_stop();
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

  void control_loop()
  {
    if (!enabled_) {
      if (!stop_latched_) {
        set_status("DISABLED");
      }
      if (hold_brake_when_disabled_) {
        publish_stop();
      }
      return;
    }
    if (count_publishers(command_topic_) != 1) {
      latch_stop("MULTIPLE_CONTROLLERS_BRAKING");
      RCLCPP_ERROR(get_logger(), "Another command publisher appeared; controller disarmed");
      return;
    }
    if (!prediction_ || !fresh(prediction_time_, prediction_timeout_s_) ||
      !fresh(speed_received_, speed_timeout_s_))
    {
      set_status("WAITING_FOR_FRESH_INPUTS");
      publish_stop();
      return;
    }
    if (require_safety_telemetry_ && !fresh(extra_info_received_, safety_timeout_s_)) {
      set_status("WAITING_FOR_SAFETY_TELEMETRY");
      publish_stop();
      return;
    }

    if (!launch_complete_) {
      if (speed_mps_ >= launch_release_speed_) {
        launch_complete_ = true;
        launch_started_.reset();
        previous_control_time_ = now();
        RCLCPP_INFO(get_logger(), "Launch assist released at %.3f m/s", speed_mps_);
      } else {
        if (!launch_started_) {
          launch_started_ = now();
          set_status("LAUNCH_ASSIST");
        }
        const double elapsed = (now() - *launch_started_).seconds();
        if (elapsed > launch_timeout_s_) {
          latch_stop("LAUNCH_FAILED_BRAKING");
          return;
        }
        if (elapsed <= launch_pulse_duration_s_) {
          publish_command(launch_throttle_, prediction_->steering, 0.0);
        } else {
          set_status("LAUNCH_SETTLING");
          publish_command(0.0, prediction_->steering, 0.0);
        }
        return;
      }
    }

    if (speed_mps_ >= model_maximum_speed_mps_) {
      set_status("OVERSPEED_BRAKING");
      publish_command(0.0, prediction_->steering, maximum_brake_);
      return;
    }

    const auto current_time = now();
    double dt = 0.02;
    if (previous_control_time_) {
      dt = std::clamp((current_time - *previous_control_time_).seconds(), 0.001, 0.1);
    }
    previous_control_time_ = current_time;

    double target_throttle = std::min(prediction_->throttle, maximum_throttle_);
    double target_brake = std::min(prediction_->brake, maximum_brake_);
    if (target_throttle >= target_brake) {
      target_throttle -= target_brake;
      target_brake = 0.0;
    } else {
      target_brake -= target_throttle;
      target_throttle = 0.0;
    }
    steering_command_ = move_towards(
      steering_command_, prediction_->steering * maximum_steering_, steering_rate_ * dt);
    throttle_command_ = move_towards(
      throttle_command_, target_throttle, throttle_rate_ * dt);
    brake_command_ = move_towards(brake_command_, target_brake, brake_rate_ * dt);
    set_status("TRACKING");
    publish_command(throttle_command_, steering_command_, brake_command_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed=%.2f steering=%.2f throttle=%.2f brake=%.2f",
      speed_mps_, steering_command_, throttle_command_, brake_command_);
  }

  std::string model_path_;
  std::string metadata_path_;
  std::string command_topic_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string modality_;
  std::string status_;
  int input_width_{0};
  int input_height_{0};
  bool enabled_{false};
  bool hold_brake_when_disabled_{false};
  bool require_safety_telemetry_{true};
  bool stop_on_cone_contact_{true};
  bool stop_latched_{false};
  bool launch_complete_{false};
  int stop_after_laps_{1};
  float minimum_depth_m_{0.2F};
  float maximum_depth_m_{4.0F};
  double rgb_horizontal_fov_deg_{80.9};
  double depth_horizontal_fov_deg_{73.8};
  double model_maximum_speed_mps_{1.5};
  double minimum_lap_distance_m_{100.0};
  double prediction_timeout_s_{0.25};
  double speed_timeout_s_{0.50};
  double safety_timeout_s_{2.0};
  double maximum_throttle_{0.35};
  double maximum_brake_{1.0};
  double maximum_steering_{1.0};
  double steering_rate_{3.0};
  double throttle_rate_{2.0};
  double brake_rate_{5.0};
  double launch_throttle_{0.45};
  double launch_release_speed_{0.10};
  double launch_pulse_duration_s_{0.10};
  double launch_timeout_s_{0.75};
  double speed_mps_{0.0};
  double distance_since_arm_m_{0.0};
  double steering_command_{0.0};
  double throttle_command_{0.0};
  double brake_command_{0.0};
  cv::dnn::Net net_;
  std::optional<Prediction> prediction_;
  std::optional<rclcpp::Time> prediction_time_;
  std::optional<rclcpp::Time> speed_received_;
  std::optional<rclcpp::Time> extra_info_received_;
  std::optional<rclcpp::Time> launch_started_;
  std::optional<rclcpp::Time> previous_control_time_;
  std::optional<std::uint32_t> latest_cone_count_;
  std::optional<std::uint32_t> cone_count_at_arm_;
  std::optional<std::size_t> latest_lap_count_;
  std::optional<std::size_t> lap_count_at_arm_;

  message_filters::Subscriber<Image> rgb_filter_sub_;
  message_filters::Subscriber<Image> depth_filter_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_sub_;
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
  std::shared_ptr<InferenceController> node;
  try {
    node = std::make_shared<InferenceController>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("behavior_cloning_inference"), "%s", error.what());
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
