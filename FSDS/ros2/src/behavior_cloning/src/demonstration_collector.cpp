#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rmw/qos_profiles.h>

#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/extra_info.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{

json camera_info_json(const sensor_msgs::msg::CameraInfo & message)
{
  return {
    {"frame_id", message.header.frame_id},
    {"width", message.width},
    {"height", message.height},
    {"distortion_model", message.distortion_model},
    {"d", message.d},
    {"k", std::vector<double>(message.k.begin(), message.k.end())},
    {"r", std::vector<double>(message.r.begin(), message.r.end())},
    {"p", std::vector<double>(message.p.begin(), message.p.end())}
  };
}

std::string padded_id(const std::uint64_t sample)
{
  std::ostringstream stream;
  stream << std::setfill('0') << std::setw(8) << sample;
  return stream.str();
}

}  // namespace

class DemonstrationCollector : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  DemonstrationCollector()
  : Node("demonstration_collector")
  {
    run_directory_ = declare_parameter<std::string>("run_directory", "");
    manifest_path_ = declare_parameter<std::string>("manifest_path", "");
    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/fsds/front_rgb/image_color");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/fsds/front_depth/image_depth");
    const auto rgb_info_topic = declare_parameter<std::string>(
      "rgb_camera_info_topic", "/fsds/front_rgb/camera_info");
    const auto depth_info_topic = declare_parameter<std::string>(
      "depth_camera_info_topic", "/fsds/front_depth/camera_info");
    control_topic_ = declare_parameter<std::string>(
      "control_topic", "/fsds/control_command");
    const auto speed_topic = declare_parameter<std::string>("speed_topic", "/fsds/gss");
    const auto extra_info_topic = declare_parameter<std::string>(
      "extra_info_topic", "/fsds/testing_only/extra_info");
    target_sample_hz_ = declare_parameter<double>("target_sample_hz", 10.0);
    sync_tolerance_s_ = declare_parameter<double>("sync_tolerance_s", 0.04);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.20);
    minimum_depth_m_ = declare_parameter<double>("minimum_depth_m", 0.2);
    maximum_depth_m_ = declare_parameter<double>("maximum_depth_m", 4.0);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 92);
    include_stationary_ = declare_parameter<bool>("include_stationary", false);
    recording_ = declare_parameter<bool>("auto_record", false);

    if (run_directory_.empty() || target_sample_hz_ <= 0.0 || sync_tolerance_s_ <= 0.0 ||
      input_timeout_s_ <= 0.0 || minimum_depth_m_ <= 0.0 ||
      maximum_depth_m_ <= minimum_depth_m_ || jpeg_quality_ < 1 || jpeg_quality_ > 100)
    {
      throw std::invalid_argument("invalid demonstration collector parameters");
    }

    initialize_dataset();

    control_sub_ = create_subscription<fs_msgs::msg::ControlCommand>(
      control_topic_, rclcpp::QoS(20).reliable(),
      [this](const fs_msgs::msg::ControlCommand::SharedPtr message) {
        latest_control_ = *message;
        control_received_ = now();
      });
    speed_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      speed_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message) {
        latest_speed_mps_ = std::hypot(
          message->twist.twist.linear.x, message->twist.twist.linear.y);
        speed_received_ = now();
      });
    extra_info_sub_ = create_subscription<fs_msgs::msg::ExtraInfo>(
      extra_info_topic, 10,
      [this](const fs_msgs::msg::ExtraInfo::SharedPtr message) {
        cone_count_ = message->doo_counter;
        lap_count_ = message->laps.size();
      });
    rgb_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      rgb_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        if (!rgb_camera_info_.has_value()) {
          rgb_camera_info_ = camera_info_json(*message);
          write_metadata();
        }
      });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        if (!depth_camera_info_.has_value()) {
          depth_camera_info_ = camera_info_json(*message);
          write_metadata();
        }
      });

    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(30), rgb_sub_, depth_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_s_));
    synchronizer_->registerCallback(
      std::bind(
        &DemonstrationCollector::image_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    record_service_ = create_service<std_srvs::srv::SetBool>(
      "/behavior_cloning/record",
      std::bind(
        &DemonstrationCollector::set_recording, this,
        std::placeholders::_1, std::placeholders::_2));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/behavior_cloning/collector/status", rclcpp::QoS(1).reliable().transient_local());
    set_status(recording_ ? "RECORDING" : "READY");

    RCLCPP_INFO(
      get_logger(), "RGB-D demonstration collector ready at %s (%s)",
      run_directory_.c_str(), recording_ ? "RECORDING" : "PAUSED");
  }

  ~DemonstrationCollector() override
  {
    recording_ = false;
    write_metadata();
  }

private:
  void initialize_dataset()
  {
    const fs::path root(run_directory_);
    if (fs::exists(root / "samples.csv")) {
      throw std::runtime_error(
              "run directory already contains samples.csv; use a new run directory");
    }
    fs::create_directories(root / "rgb");
    fs::create_directories(root / "depth");
    csv_.open(root / "samples.csv", std::ios::out | std::ios::trunc);
    if (!csv_) {
      throw std::runtime_error("cannot create dataset CSV in " + run_directory_);
    }
    csv_ << "sample_id,stamp_ns,rgb_file,depth_file,steering,throttle,brake,"
            "speed_mps,cone_count,lap_count\n";
    csv_ << std::setprecision(10);
    created_unix_s_ = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    write_metadata();
  }

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

  void set_recording(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      const auto publisher_count = count_publishers(control_topic_);
      if (publisher_count != 1) {
        response->success = false;
        response->message =
          "expected exactly one expert control publisher, found " +
          std::to_string(publisher_count);
        return;
      }
    }
    recording_ = request->data;
    last_sample_time_.reset();
    set_status(recording_ ? "RECORDING" : "PAUSED");
    write_metadata();
    response->success = true;
    response->message = recording_ ? "recording demonstrations" : "recording paused";
  }

  bool fresh(const std::optional<rclcpp::Time> & stamp) const
  {
    return stamp.has_value() && (now() - *stamp).seconds() <= input_timeout_s_;
  }

  cv::Mat depth_in_metres(const Image::ConstSharedPtr & message) const
  {
    if (message->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return cv_bridge::toCvShare(message, message->encoding)->image.clone();
    }
    if (message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      message->encoding == sensor_msgs::image_encodings::MONO16)
    {
      const cv::Mat raw = cv_bridge::toCvShare(message, message->encoding)->image;
      cv::Mat metres;
      raw.convertTo(metres, CV_32FC1, 0.001);
      return metres;
    }
    throw std::runtime_error("unsupported depth encoding: " + message->encoding);
  }

  cv::Mat depth_in_millimetres(const cv::Mat & metres) const
  {
    cv::Mat millimetres(metres.size(), CV_16UC1, cv::Scalar(0));
    for (int row = 0; row < metres.rows; ++row) {
      const auto * source = metres.ptr<float>(row);
      auto * destination = millimetres.ptr<std::uint16_t>(row);
      for (int column = 0; column < metres.cols; ++column) {
        const float value = source[column];
        if (std::isfinite(value) && value >= minimum_depth_m_ && value <= maximum_depth_m_) {
          destination[column] = static_cast<std::uint16_t>(
            std::clamp(std::lround(value * 1000.0F), 1L, 65535L));
        }
      }
    }
    return millimetres;
  }

  void image_callback(
    const Image::ConstSharedPtr & rgb_message,
    const Image::ConstSharedPtr & depth_message)
  {
    ++synchronized_pairs_;
    if (!recording_) {
      return;
    }
    if (count_publishers(control_topic_) != 1) {
      recording_ = false;
      set_status("INVALID_CONTROL_PUBLISHERS");
      write_metadata();
      RCLCPP_ERROR(
        get_logger(), "Recording stopped because the expert control publisher count changed");
      return;
    }
    if (!fresh(control_received_) || !fresh(speed_received_) || !latest_control_.has_value()) {
      ++dropped_stale_inputs_;
      set_status("WAITING_FOR_FRESH_LABELS");
      return;
    }

    const auto sample_time = now();
    if (last_sample_time_.has_value() &&
      (sample_time - *last_sample_time_).seconds() < 1.0 / target_sample_hz_)
    {
      return;
    }
    const auto & control = *latest_control_;
    if (!include_stationary_ && latest_speed_mps_ < 0.02 && control.throttle < 0.01 &&
      control.brake > 0.95)
    {
      ++dropped_stationary_;
      return;
    }

    try {
      const cv::Mat bgr = cv_bridge::toCvCopy(
        rgb_message, sensor_msgs::image_encodings::BGR8)->image;
      const cv::Mat depth_m = depth_in_metres(depth_message);
      if (bgr.size() != depth_m.size()) {
        throw std::runtime_error("RGB and depth resolutions differ");
      }

      const std::string id = padded_id(sample_count_);
      const fs::path rgb_relative = fs::path("rgb") / (id + ".jpg");
      const fs::path depth_relative = fs::path("depth") / (id + ".png");
      const fs::path rgb_final = fs::path(run_directory_) / rgb_relative;
      const fs::path depth_final = fs::path(run_directory_) / depth_relative;
      const fs::path rgb_temporary = rgb_final.string() + ".tmp.jpg";
      const fs::path depth_temporary = depth_final.string() + ".tmp.png";

      const std::vector<int> jpeg_parameters{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (!cv::imwrite(rgb_temporary.string(), bgr, jpeg_parameters)) {
        throw std::runtime_error("failed to encode RGB frame");
      }
      if (!cv::imwrite(depth_temporary.string(), depth_in_millimetres(depth_m))) {
        fs::remove(rgb_temporary);
        throw std::runtime_error("failed to encode depth frame");
      }
      fs::rename(rgb_temporary, rgb_final);
      fs::rename(depth_temporary, depth_final);

      const std::int64_t stamp_ns = rclcpp::Time(rgb_message->header.stamp).nanoseconds();
      csv_ << sample_count_ << ',' << stamp_ns << ',' << rgb_relative.generic_string() << ','
           << depth_relative.generic_string() << ',' << control.steering << ','
           << control.throttle << ',' << control.brake << ',' << latest_speed_mps_ << ','
           << cone_count_ << ',' << lap_count_ << '\n';
      ++sample_count_;
      last_sample_time_ = sample_time;
      set_status("RECORDING");
      if (sample_count_ % 100 == 0) {
        csv_.flush();
        write_metadata();
        RCLCPP_INFO(
          get_logger(), "Recorded %llu synchronized samples",
          static_cast<unsigned long long>(sample_count_));
      }
    } catch (const std::exception & error) {
      ++write_failures_;
      set_status("WRITE_ERROR");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to record sample: %s", error.what());
    }
  }

  void write_metadata() const
  {
    try {
      json metadata{
        {"schema_version", 1},
        {"created_unix_s", created_unix_s_},
        {"run_directory", fs::absolute(run_directory_).string()},
        {"manifest_path", manifest_path_.empty() ? json(nullptr) : json(manifest_path_)},
        {"topics", {
            {"rgb", rgb_topic_}, {"depth", depth_topic_},
            {"control", control_topic_}, {"speed", "/fsds/gss"}}},
        {"capture", {
            {"target_sample_hz", target_sample_hz_},
            {"sync_tolerance_s", sync_tolerance_s_},
            {"minimum_depth_m", minimum_depth_m_},
            {"maximum_depth_m", maximum_depth_m_},
            {"jpeg_quality", jpeg_quality_},
            {"include_stationary", include_stationary_}}},
        {"statistics", {
            {"samples", sample_count_},
            {"synchronized_pairs", synchronized_pairs_},
            {"dropped_stale_inputs", dropped_stale_inputs_},
            {"dropped_stationary", dropped_stationary_},
            {"write_failures", write_failures_}}},
        {"recording", recording_}
      };
      if (rgb_camera_info_) {
        metadata["rgb_camera_info"] = *rgb_camera_info_;
      }
      if (depth_camera_info_) {
        metadata["depth_camera_info"] = *depth_camera_info_;
      }
      const fs::path final_path = fs::path(run_directory_) / "metadata.json";
      const fs::path temporary_path = final_path.string() + ".tmp";
      std::ofstream output(temporary_path);
      output << std::setw(2) << metadata << '\n';
      output.close();
      fs::rename(temporary_path, final_path);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot update dataset metadata: %s", error.what());
    }
  }

  std::string run_directory_;
  std::string manifest_path_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string control_topic_;
  std::string status_;
  double target_sample_hz_{10.0};
  double sync_tolerance_s_{0.04};
  double input_timeout_s_{0.20};
  double minimum_depth_m_{0.2};
  double maximum_depth_m_{4.0};
  int jpeg_quality_{92};
  bool include_stationary_{false};
  bool recording_{false};
  std::int64_t created_unix_s_{0};
  std::uint64_t sample_count_{0};
  std::uint64_t synchronized_pairs_{0};
  std::uint64_t dropped_stale_inputs_{0};
  std::uint64_t dropped_stationary_{0};
  std::uint64_t write_failures_{0};
  double latest_speed_mps_{0.0};
  std::uint32_t cone_count_{0};
  std::size_t lap_count_{0};
  std::ofstream csv_;
  std::optional<fs_msgs::msg::ControlCommand> latest_control_;
  std::optional<rclcpp::Time> control_received_;
  std::optional<rclcpp::Time> speed_received_;
  std::optional<rclcpp::Time> last_sample_time_;
  std::optional<json> rgb_camera_info_;
  std::optional<json> depth_camera_info_;

  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DemonstrationCollector>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("demonstration_collector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
