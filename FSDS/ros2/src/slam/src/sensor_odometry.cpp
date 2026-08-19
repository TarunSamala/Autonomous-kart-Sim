#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z) && std::isfinite(value.w) &&
         value.x * value.x + value.y * value.y + value.z * value.z +
         value.w * value.w > 1.0e-8;
}
}  // namespace

class SensorOdometry : public rclcpp::Node
{
public:
  SensorOdometry()
  : Node("sensor_odometry"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/fsds/imu");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/fsds/gss");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/localization/sensor_odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "sensor/odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "fsds/FSCar");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    velocity_timeout_s_ = declare_parameter<double>("velocity_timeout_s", 0.25);
    maximum_dt_s_ = declare_parameter<double>("maximum_dt_s", 0.20);
    maximum_speed_mps_ = declare_parameter<double>("maximum_speed_mps", 20.0);

    if (imu_topic_.empty() || velocity_topic_.empty() || odometry_topic_.empty() ||
      odom_frame_.empty() || base_frame_.empty() || velocity_timeout_s_ <= 0.0 ||
      maximum_dt_s_ <= 0.0 || maximum_speed_mps_ <= 0.0)
    {
      throw std::invalid_argument("invalid sensor odometry parameters");
    }

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      velocity_topic_, sensor_qos,
      std::bind(&SensorOdometry::velocity_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&SensorOdometry::imu_callback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/slam/sensor_odometry/reset",
      std::bind(
        &SensorOdometry::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Planar sensor odometry: %s + %s -> %s (%s -> %s)",
      imu_topic_.c_str(), velocity_topic_.c_str(), odometry_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message)
  {
    const double x = message->twist.twist.linear.x;
    const double y = message->twist.twist.linear.y;
    if (!std::isfinite(x) || !std::isfinite(y) || std::hypot(x, y) > maximum_speed_mps_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting invalid ground-speed sample");
      return;
    }
    body_velocity_x_ = x;
    body_velocity_y_ = y;
    velocity_stamp_ = rclcpp::Time(message->header.stamp);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (!finite_quaternion(message->orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting invalid IMU orientation");
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    const tf2::Quaternion measured_orientation(
      message->orientation.x, message->orientation.y,
      message->orientation.z, message->orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double measured_yaw = 0.0;
    tf2::Matrix3x3(measured_orientation).getRPY(roll, pitch, measured_yaw);

    if (!yaw_origin_) {
      yaw_origin_ = measured_yaw;
      previous_stamp_ = stamp;
      previous_world_velocity_.reset();
      publish(stamp, 0.0, message->angular_velocity.z);
      return;
    }

    const double yaw = wrap_angle(measured_yaw - *yaw_origin_);
    if (!previous_stamp_) {
      previous_stamp_ = stamp;
      publish(stamp, yaw, message->angular_velocity.z);
      return;
    }

    const double dt = (stamp - *previous_stamp_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      // FSDS may republish an IMU sample before Unreal has advanced the
      // sensor timestamp. It is not a new integration interval.
      return;
    }
    previous_stamp_ = stamp;
    if (dt > maximum_dt_s_) {
      previous_world_velocity_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rebasing after invalid IMU interval %.3f s", dt);
      publish(stamp, yaw, message->angular_velocity.z);
      return;
    }

    double body_x = 0.0;
    double body_y = 0.0;
    bool velocity_is_fresh = false;
    if (velocity_stamp_) {
      const double age = std::abs((stamp - *velocity_stamp_).seconds());
      velocity_is_fresh = std::isfinite(age) && age <= velocity_timeout_s_;
    }
    if (velocity_is_fresh) {
      body_x = body_velocity_x_;
      body_y = body_velocity_y_;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ground-speed sample is unavailable or stale; holding translation");
    }

    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const double world_x = cosine * body_x - sine * body_y;
    const double world_y = sine * body_x + cosine * body_y;
    if (previous_world_velocity_ && velocity_is_fresh) {
      position_x_ += 0.5 * (previous_world_velocity_->first + world_x) * dt;
      position_y_ += 0.5 * (previous_world_velocity_->second + world_y) * dt;
    } else if (velocity_is_fresh) {
      position_x_ += world_x * dt;
      position_y_ += world_y * dt;
    }
    if (velocity_is_fresh) {
      previous_world_velocity_ = std::make_pair(world_x, world_y);
    } else {
      previous_world_velocity_.reset();
    }
    publish(stamp, yaw, message->angular_velocity.z);
  }

  void publish(const rclcpp::Time & stamp, const double yaw, const double yaw_rate)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position_x_;
    odometry.pose.pose.position.y = position_y_;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = body_velocity_x_;
    odometry.twist.twist.linear.y = body_velocity_y_;
    odometry.twist.twist.angular.z = yaw_rate;
    odometry.pose.covariance[0] = 0.02;
    odometry.pose.covariance[7] = 0.02;
    odometry.pose.covariance[35] = 0.005;
    odometry.twist.covariance[0] = 0.10;
    odometry.twist.covariance[7] = 0.10;
    odometry.twist.covariance[35] = 0.01;
    odometry_pub_->publish(odometry);

    if (!publish_tf_) {return;}
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position_x_;
    transform.transform.translation.y = position_y_;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    position_x_ = 0.0;
    position_y_ = 0.0;
    yaw_origin_.reset();
    previous_stamp_.reset();
    previous_world_velocity_.reset();
    response->success = true;
    response->message = "sensor odometry reset; waiting for a new IMU origin";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::string imu_topic_;
  std::string velocity_topic_;
  std::string odometry_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{};
  double velocity_timeout_s_{};
  double maximum_dt_s_{};
  double maximum_speed_mps_{};
  double body_velocity_x_{};
  double body_velocity_y_{};
  double position_x_{};
  double position_y_{};
  std::optional<double> yaw_origin_;
  std::optional<rclcpp::Time> velocity_stamp_;
  std::optional<rclcpp::Time> previous_stamp_;
  std::optional<std::pair<double, double>> previous_world_velocity_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SensorOdometry>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sensor_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
