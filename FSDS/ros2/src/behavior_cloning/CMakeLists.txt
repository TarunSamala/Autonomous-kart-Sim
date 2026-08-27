cmake_minimum_required(VERSION 3.8)
project(behavior_cloning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(demonstration_collector src/demonstration_collector.cpp)
target_compile_options(demonstration_collector PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(demonstration_collector
  cv_bridge fs_msgs geometry_msgs message_filters rclcpp sensor_msgs std_msgs std_srvs)
target_link_libraries(demonstration_collector
  nlohmann_json::nlohmann_json ${OpenCV_LIBS})
target_include_directories(demonstration_collector PRIVATE include ${OpenCV_INCLUDE_DIRS})

add_executable(inference_controller src/inference_controller.cpp)
target_compile_options(inference_controller PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(inference_controller
  cv_bridge fs_msgs geometry_msgs message_filters rclcpp sensor_msgs std_msgs std_srvs)
target_link_libraries(inference_controller
  nlohmann_json::nlohmann_json ${OpenCV_LIBS})
target_include_directories(inference_controller PRIVATE include ${OpenCV_INCLUDE_DIRS})

install(TARGETS demonstration_collector inference_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch training
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_preprocessing test/test_preprocessing.cpp)
  target_include_directories(test_preprocessing PRIVATE include)
  target_link_libraries(test_preprocessing ${OpenCV_LIBS})
  target_include_directories(test_preprocessing PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()

ament_package()
