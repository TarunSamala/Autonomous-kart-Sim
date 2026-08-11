#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuReframer : public rclcpp::Node
{
public:
  ImuReframer()
  : Node("imu_reframer")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/fsds/imu");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/sensors/imu/camera");
    target_frame_ = declare_parameter<std::string>(
      "target_frame", "fsds/front_camera_imu");

    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr input) {
        auto output = *input;
        output.header.frame_id = target_frame_;
        publisher_->publish(output);
      });

    RCLCPP_WARN(
      get_logger(),
      "Simulation adapter active: camera IMU reuses the FSDS chassis IMU. "
      "Timing and orientation are valid for pipeline development, but camera "
      "lever-arm acceleration is not simulated.");
  }

private:
  std::string target_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuReframer>());
  rclcpp::shutdown();
  return 0;
}
