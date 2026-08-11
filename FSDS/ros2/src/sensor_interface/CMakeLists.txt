cmake_minimum_required(VERSION 3.8)
project(sensor_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_reframer src/imu_reframer.cpp)
target_compile_options(imu_reframer PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(imu_reframer rclcpp sensor_msgs)

install(TARGETS imu_reframer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
