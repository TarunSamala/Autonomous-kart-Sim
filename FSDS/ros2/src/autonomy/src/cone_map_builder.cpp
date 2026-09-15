#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{
struct Landmark
{
  double x{};
  double y{};
  std::size_t observations{1};
};

double squared_distance(const Landmark & landmark, const double x, const double y)
{
  const double dx = landmark.x - x;
  const double dy = landmark.y - y;
  return dx * dx + dy * dy;
}
}  // namespace

class ConeMapBuilder : public rclcpp::Node
{
public:
  ConeMapBuilder()
  : Node("cone_map_builder"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "slam/map");
    save_path_ = declare_parameter<std::string>("save_path", "");
    centerline_path_ = declare_parameter<std::string>("centerline_path", "");
    association_radius_ = declare_parameter<double>("association_radius", 0.45);
    const int minimum_observations = declare_parameter<int>("minimum_observations", 3);
    const int minimum_cones_per_side = declare_parameter<int>("minimum_cones_per_side", 4);
    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.20);
    maximum_centerline_offset_ = declare_parameter<double>("maximum_centerline_offset", 3.0);
    centerline_spacing_ = declare_parameter<double>("centerline_spacing", 0.30);
    const int minimum_centerline_points = declare_parameter<int>("minimum_centerline_points", 20);

    if (global_frame_.empty() || save_path_.empty() || centerline_path_.empty() ||
      association_radius_ <= 0.0 ||
      minimum_observations <= 0 || minimum_cones_per_side < 2 || publish_period_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid cone map builder parameters");
    }
    minimum_observations_ = static_cast<std::size_t>(minimum_observations);
    minimum_cones_per_side_ = static_cast<std::size_t>(minimum_cones_per_side);
    if (maximum_centerline_offset_ <= 0.0 || centerline_spacing_ <= 0.0 ||
      minimum_centerline_points < 3)
    {
      throw std::invalid_argument("invalid centreline builder parameters");
    }
    minimum_centerline_points_ = static_cast<std::size_t>(minimum_centerline_points);

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    left_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/left", sensor_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        observe(*message, left_cones_, "left cones");
      });
    right_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/right", sensor_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        observe(*message, right_cones_, "right cones");
      });
    midpoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/autonomy/delaunay_midpoints", sensor_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        observe(*message, midpoints_, "midpoints");
      });
    route_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/slam/recorded_route", rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        taught_route_ = *message;
      });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/autonomy/map/markers", rclcpp::QoS(1).reliable().transient_local());
    centerline_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/autonomy/map/centerline", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/cone_map/status", rclcpp::QoS(1).reliable().transient_local());
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/autonomy/cone_map/save",
      std::bind(&ConeMapBuilder::save, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/autonomy/cone_map/reset",
      std::bind(&ConeMapBuilder::reset, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(publish_period_s_));
    timer_ = create_wall_timer(period, std::bind(&ConeMapBuilder::publish, this));
    set_status("MAPPING");
    RCLCPP_INFO(
      get_logger(), "Persistent cone map: local detections -> %s; output=%s",
      global_frame_.c_str(), fs::absolute(save_path_).c_str());
  }

private:
  std::optional<tf2::Transform> global_from(const std::string & source_frame)
  {
    if (source_frame.empty()) {
      return std::nullopt;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, source_frame, tf2::TimePointZero);
      const auto & rotation = transform.transform.rotation;
      const auto & translation = transform.transform.translation;
      return tf2::Transform(
        tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        tf2::Vector3(translation.x, translation.y, translation.z));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cone-map TF %s <- %s unavailable: %s",
        global_frame_.c_str(), source_frame.c_str(), error.what());
      return std::nullopt;
    }
  }

  void associate(std::vector<Landmark> & landmarks, const double x, const double y)
  {
    const double radius_squared = association_radius_ * association_radius_;
    auto nearest = landmarks.end();
    double nearest_distance = radius_squared;
    for (auto candidate = landmarks.begin(); candidate != landmarks.end(); ++candidate) {
      const double distance = squared_distance(*candidate, x, y);
      if (distance <= nearest_distance) {
        nearest = candidate;
        nearest_distance = distance;
      }
    }
    if (nearest == landmarks.end()) {
      landmarks.push_back({x, y, 1});
      return;
    }
    const double count = static_cast<double>(nearest->observations);
    nearest->x = (nearest->x * count + x) / (count + 1.0);
    nearest->y = (nearest->y * count + y) / (count + 1.0);
    ++nearest->observations;
  }

  void observe(
    const geometry_msgs::msg::PoseArray & message, std::vector<Landmark> & landmarks,
    const char * label)
  {
    const auto transform = global_from(message.header.frame_id);
    if (!transform) {
      return;
    }
    for (const auto & pose : message.poses) {
      if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) {
        continue;
      }
      const tf2::Vector3 global = *transform *
        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
      associate(landmarks, global.x(), global.y());
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Mapped %s: %zu candidates (%zu confirmed)",
      label, landmarks.size(), confirmed_count(landmarks));
  }

  std::size_t confirmed_count(const std::vector<Landmark> & landmarks) const
  {
    return static_cast<std::size_t>(std::count_if(
      landmarks.begin(), landmarks.end(), [this](const Landmark & landmark) {
        return landmark.observations >= minimum_observations_;
      }));
  }

  visualization_msgs::msg::Marker marker(
    const std::string & marker_namespace, const int id, const int type) const
  {
    visualization_msgs::msg::Marker output;
    output.header.frame_id = global_frame_;
    output.header.stamp = now();
    output.ns = marker_namespace;
    output.id = id;
    output.type = type;
    output.action = visualization_msgs::msg::Marker::ADD;
    output.pose.orientation.w = 1.0;
    return output;
  }

  void append_cones(
    visualization_msgs::msg::MarkerArray & output, const std::vector<Landmark> & landmarks,
    const std::string & marker_namespace, const float red, const float green, const float blue)
  {
    int id = 0;
    for (const auto & landmark : landmarks) {
      if (landmark.observations < minimum_observations_) {
        continue;
      }
      auto value = marker(marker_namespace, id++, visualization_msgs::msg::Marker::CYLINDER);
      value.pose.position.x = landmark.x;
      value.pose.position.y = landmark.y;
      value.pose.position.z = 0.18;
      value.scale.x = 0.24;
      value.scale.y = 0.24;
      value.scale.z = 0.36;
      value.color.r = red;
      value.color.g = green;
      value.color.b = blue;
      value.color.a = 1.0F;
      output.markers.push_back(std::move(value));
    }
  }

  void publish()
  {
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    append_cones(output, left_cones_, "mapped_left", 0.05F, 0.25F, 1.0F);
    append_cones(output, right_cones_, "mapped_right", 1.0F, 0.75F, 0.0F);

    auto midpoint_marker = marker("mapped_midpoints", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
    midpoint_marker.scale.x = 0.16;
    midpoint_marker.scale.y = 0.16;
    midpoint_marker.scale.z = 0.16;
    midpoint_marker.color.r = 1.0F;
    midpoint_marker.color.g = 0.25F;
    midpoint_marker.color.b = 0.02F;
    midpoint_marker.color.a = 0.95F;
    for (const auto & midpoint : midpoints_) {
      if (midpoint.observations < minimum_observations_) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = midpoint.x;
      point.y = midpoint.y;
      point.z = 0.08;
      midpoint_marker.points.push_back(point);
    }
    output.markers.push_back(std::move(midpoint_marker));
    marker_pub_->publish(output);

    nav_msgs::msg::Path centerline_path;
    centerline_path.header.frame_id = global_frame_;
    centerline_path.header.stamp = now();
    for (const auto & midpoint : ordered_centerline()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = centerline_path.header;
      pose.pose.position.x = midpoint.x;
      pose.pose.position.y = midpoint.y;
      pose.pose.orientation.w = 1.0;
      centerline_path.poses.push_back(pose);
    }
    centerline_pub_->publish(centerline_path);
  }

  json serialize(const std::vector<Landmark> & landmarks) const
  {
    json output = json::array();
    for (const auto & landmark : landmarks) {
      if (landmark.observations >= minimum_observations_) {
        output.push_back({
          {"x", landmark.x}, {"y", landmark.y}, {"observations", landmark.observations}});
      }
    }
    return output;
  }

  std::vector<Landmark> ordered_centerline() const
  {
    if (taught_route_.poses.size() < 3) {
      return {};
    }
    struct OrderedPoint
    {
      std::size_t route_index{};
      Landmark landmark;
    };
    std::vector<OrderedPoint> ordered;
    const double maximum_offset_squared =
      maximum_centerline_offset_ * maximum_centerline_offset_;
    for (const auto & midpoint : midpoints_) {
      if (midpoint.observations < minimum_observations_) {
        continue;
      }
      std::size_t nearest_index = 0;
      double nearest_distance = std::numeric_limits<double>::max();
      for (std::size_t index = 0; index < taught_route_.poses.size(); ++index) {
        const auto & position = taught_route_.poses[index].pose.position;
        const double dx = midpoint.x - position.x;
        const double dy = midpoint.y - position.y;
        const double distance = dx * dx + dy * dy;
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest_index = index;
        }
      }
      if (nearest_distance <= maximum_offset_squared) {
        ordered.push_back({nearest_index, midpoint});
      }
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto & first, const auto & second) {
      return first.route_index < second.route_index;
    });

    std::vector<Landmark> output;
    for (const auto & candidate : ordered) {
      if (output.empty() ||
        std::hypot(candidate.landmark.x - output.back().x,
        candidate.landmark.y - output.back().y) >= centerline_spacing_)
      {
        output.push_back(candidate.landmark);
      }
    }
    return output;
  }

  json centerline_document(const std::vector<Landmark> & centerline) const
  {
    json poses = json::array();
    for (std::size_t index = 0; index < centerline.size(); ++index) {
      const Landmark & current = centerline[index];
      const Landmark & next = centerline[(index + 1) % centerline.size()];
      poses.push_back({current.x, current.y, std::atan2(next.y - current.y, next.x - current.x)});
    }
    return {
      {"schema_version", 1},
      {"frame_id", global_frame_},
      {"closed", true},
      {"sample_distance_m", centerline_spacing_},
      {"point_count", centerline.size()},
      {"save_reason", "persistent_delaunay_midpoints"},
      {"poses", std::move(poses)}
    };
  }

  static void write_temporary_json(const fs::path & path, const json & document)
  {
    std::ofstream stream(path);
    if (!stream) {
      throw std::runtime_error("cannot open temporary output file: " + path.string());
    }
    stream << std::setw(2) << document << '\n';
    if (!stream) {
      throw std::runtime_error("failed while writing output file: " + path.string());
    }
  }

  void save(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const std::size_t left_count = confirmed_count(left_cones_);
    const std::size_t right_count = confirmed_count(right_cones_);
    const std::size_t midpoint_count = confirmed_count(midpoints_);
    const auto centerline = ordered_centerline();
    if (left_count < minimum_cones_per_side_ || right_count < minimum_cones_per_side_ ||
      midpoint_count < 2 || centerline.size() < minimum_centerline_points_)
    {
      response->success = false;
      response->message = "map is incomplete: left=" + std::to_string(left_count) +
        " right=" + std::to_string(right_count) +
        " midpoints=" + std::to_string(midpoint_count) +
        " ordered_centerline=" + std::to_string(centerline.size());
      return;
    }

    const fs::path destination = fs::absolute(save_path_);
    const fs::path centerline_destination = fs::absolute(centerline_path_);
    fs::create_directories(destination.parent_path());
    fs::create_directories(centerline_destination.parent_path());
    const fs::path temporary = destination.string() + ".tmp";
    const fs::path centerline_temporary = centerline_destination.string() + ".tmp";
    const json document = {
      {"schema_version", 1},
      {"frame_id", global_frame_},
      {"association_radius_m", association_radius_},
      {"minimum_observations", minimum_observations_},
      {"left_cones", serialize(left_cones_)},
      {"right_cones", serialize(right_cones_)},
      {"delaunay_midpoints", serialize(midpoints_)}
    };
    try {
      write_temporary_json(temporary, document);
      write_temporary_json(centerline_temporary, centerline_document(centerline));
      fs::rename(temporary, destination);
      fs::rename(centerline_temporary, centerline_destination);
      set_status("SAVED");
      response->success = true;
      response->message = destination.string();
      RCLCPP_INFO(
        get_logger(),
        "Saved cone map: left=%zu right=%zu midpoints=%zu; centreline=%zu -> %s",
        left_count, right_count, midpoint_count, centerline.size(), destination.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  void reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    left_cones_.clear();
    right_cones_.clear();
    midpoints_.clear();
    set_status("MAPPING");
    publish();
    response->success = true;
    response->message = "persistent cone map cleared";
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

  std::string global_frame_;
  std::string save_path_;
  std::string centerline_path_;
  std::string status_;
  double association_radius_{};
  double publish_period_s_{};
  double maximum_centerline_offset_{};
  double centerline_spacing_{};
  std::size_t minimum_observations_{};
  std::size_t minimum_cones_per_side_{};
  std::size_t minimum_centerline_points_{};
  std::vector<Landmark> left_cones_;
  std::vector<Landmark> right_cones_;
  std::vector<Landmark> midpoints_;
  nav_msgs::msg::Path taught_route_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr right_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr midpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ConeMapBuilder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cone_map_builder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
