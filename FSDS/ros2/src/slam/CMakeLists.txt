cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(route_recorder src/route_recorder.cpp)
target_compile_features(route_recorder PUBLIC cxx_std_17)
target_link_libraries(route_recorder nlohmann_json::nlohmann_json)
ament_target_dependencies(route_recorder
  fs_msgs geometry_msgs nav_msgs rclcpp std_msgs std_srvs tf2 tf2_ros)

add_executable(sensor_odometry src/sensor_odometry.cpp)
target_compile_features(sensor_odometry PUBLIC cxx_std_17)
ament_target_dependencies(sensor_odometry
  geometry_msgs nav_msgs rclcpp sensor_msgs std_srvs tf2 tf2_ros)

install(TARGETS route_recorder sensor_odometry RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
