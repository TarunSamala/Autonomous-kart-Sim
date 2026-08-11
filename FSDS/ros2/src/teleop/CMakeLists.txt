cmake_minimum_required(VERSION 3.8)
project(teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(X11 REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_compile_options(keyboard_teleop PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(keyboard_teleop PRIVATE ${X11_INCLUDE_DIR})
target_link_libraries(keyboard_teleop ${X11_LIBRARIES})
ament_target_dependencies(keyboard_teleop fs_msgs rclcpp)

install(TARGETS keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
