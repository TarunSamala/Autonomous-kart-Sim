#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
struct Point3 {double x; double y; double z;};
struct Cell {int x; int y; int z;};

bool operator==(const Cell & first, const Cell & second)
{
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

struct CellHash
{
  std::size_t operator()(const Cell & cell) const
  {
    std::size_t seed = std::hash<int>{}(cell.x);
    seed ^= std::hash<int>{}(cell.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(cell.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};
}  // namespace

class ConeDetector : public rclcpp::Node
{
public:
  ConeDetector()
  : Node("cone_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/fsds/lidar/Lidar1");
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    cluster_radius_ = declare_parameter<double>("cluster_radius", 0.25);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 180);
    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 15.0);
    min_z_ = declare_parameter<double>("min_z", -0.30);
    max_z_ = declare_parameter<double>("max_z", 0.30);
    max_cluster_extent_ = declare_parameter<double>("max_cluster_extent", 0.80);

    if (cluster_radius_ <= 0.0 || min_cluster_points_ <= 0 ||
      max_cluster_points_ < min_cluster_points_ || min_range_ < 0.0 ||
      max_range_ <= min_range_ || max_z_ <= min_z_)
    {
      throw std::invalid_argument("invalid cone detector parameters");
    }

    left_pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/left", 10);
    right_pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/autonomy/cones/right", 10);
    left_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/autonomy/debug/left_cones", 10);
    right_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/autonomy/debug/right_cones", 10);
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ConeDetector::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LiDAR cone detector listening on %s", lidar_topic_.c_str());
  }

private:
  Cell cell_for(const Point3 & point) const
  {
    return {
      static_cast<int>(std::floor(point.x / cluster_radius_)),
      static_cast<int>(std::floor(point.y / cluster_radius_)),
      static_cast<int>(std::floor(point.z / cluster_radius_))};
  }

  std::vector<Point3> read_filtered_points(
    const sensor_msgs::msg::PointCloud2 & message)
  {
    std::vector<Point3> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const Point3 point{*x, *y, *z};
        const double range = std::hypot(point.x, point.y);
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && point.x > 0.0 &&
          point.z > min_z_ && point.z < max_z_ &&
          range > min_range_ && range < max_range_)
        {
          points.push_back(point);
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid PointCloud2: %s", error.what());
    }
    return points;
  }

  std::vector<std::vector<Point3>> cluster(const std::vector<Point3> & points) const
  {
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells;
    for (std::size_t index = 0; index < points.size(); ++index) {
      cells[cell_for(points[index])].push_back(index);
    }

    std::vector<bool> used(points.size(), false);
    std::vector<std::vector<Point3>> clusters;
    for (std::size_t start = 0; start < points.size(); ++start) {
      if (used[start]) {continue;}
      used[start] = true;
      std::queue<std::size_t> pending;
      pending.push(start);
      std::vector<Point3> current;
      while (!pending.empty() &&
        current.size() <= static_cast<std::size_t>(max_cluster_points_))
      {
        const std::size_t index = pending.front();
        pending.pop();
        current.push_back(points[index]);
        const Cell origin = cell_for(points[index]);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const auto found = cells.find({origin.x + dx, origin.y + dy, origin.z + dz});
              if (found == cells.end()) {continue;}
              for (const auto neighbour : found->second) {
                if (used[neighbour]) {continue;}
                const double separation = std::sqrt(
                  std::pow(points[neighbour].x - points[index].x, 2) +
                  std::pow(points[neighbour].y - points[index].y, 2) +
                  std::pow(points[neighbour].z - points[index].z, 2));
                if (separation < cluster_radius_) {
                  used[neighbour] = true;
                  pending.push(neighbour);
                }
              }
            }
          }
        }
      }
      if (current.size() >= static_cast<std::size_t>(min_cluster_points_) &&
        current.size() <= static_cast<std::size_t>(max_cluster_points_))
      {
        clusters.push_back(std::move(current));
      }
    }
    return clusters;
  }

  bool centroid_if_cone(const std::vector<Point3> & points, Point3 & centroid) const
  {
    Point3 minimum{points.front()};
    Point3 maximum{points.front()};
    centroid = {0.0, 0.0, 0.0};
    for (const auto & point : points) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
    }
    if (maximum.x - minimum.x > max_cluster_extent_ ||
      maximum.y - minimum.y > max_cluster_extent_ ||
      maximum.z - minimum.z > max_cluster_extent_)
    {
      return false;
    }
    const double count = static_cast<double>(points.size());
    centroid.x /= count;
    centroid.y /= count;
    centroid.z /= count;
    return true;
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, message->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "LiDAR TF unavailable: %s", error.what());
      publish({}, {}, message->header.stamp);
      return;
    }
    const auto & rotation = transform.transform.rotation;
    const auto & translation = transform.transform.translation;
    const tf2::Transform base_from_sensor(
      tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
      tf2::Vector3(translation.x, translation.y, translation.z));

    const auto filtered_points = read_filtered_points(*message);
    const auto clusters = cluster(filtered_points);
    std::vector<Point3> left;
    std::vector<Point3> right;
    for (const auto & points : clusters) {
      Point3 centroid{};
      if (!centroid_if_cone(points, centroid)) {continue;}
      const tf2::Vector3 transformed = base_from_sensor *
        tf2::Vector3(centroid.x, centroid.y, centroid.z);
      Point3 base_point{transformed.x(), transformed.y(), transformed.z()};
      (base_point.y >= 0.0 ? left : right).push_back(base_point);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LiDAR points=%u filtered=%zu clusters=%zu cones=%zu (left=%zu right=%zu)",
      message->width * message->height, filtered_points.size(), clusters.size(),
      left.size() + right.size(), left.size(), right.size());
    publish(left, right, message->header.stamp);
  }

  void publish(
    const std::vector<Point3> & left, const std::vector<Point3> & right,
    const builtin_interfaces::msg::Time & stamp)
  {
    left_pose_pub_->publish(make_poses(left, stamp));
    right_pose_pub_->publish(make_poses(right, stamp));
    left_marker_pub_->publish(make_markers(left, stamp, "left", {0.0F, 0.2F, 1.0F}));
    right_marker_pub_->publish(make_markers(right, stamp, "right", {1.0F, 0.9F, 0.0F}));
  }

  geometry_msgs::msg::PoseArray make_poses(
    const std::vector<Point3> & points, const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::PoseArray message;
    message.header.frame_id = base_frame_;
    message.header.stamp = stamp;
    for (const auto & point : points) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = point.z;
      pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    return message;
  }

  visualization_msgs::msg::MarkerArray make_markers(
    const std::vector<Point3> & points, const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace, const std::array<float, 3> & color) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    for (std::size_t index = 0; index < points.size(); ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = stamp;
      marker.ns = marker_namespace;
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = points[index].x;
      marker.pose.position.y = points[index].y;
      marker.pose.position.z = 0.15;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.23;
      marker.scale.y = 0.23;
      marker.scale.z = 0.40;
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 1.0F;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 250000000U;
      array.markers.push_back(marker);
    }
    return array;
  }

  std::string lidar_topic_;
  std::string base_frame_;
  double cluster_radius_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_range_;
  double max_range_;
  double min_z_;
  double max_z_;
  double max_cluster_extent_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr left_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr right_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ConeDetector>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cone_detector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
