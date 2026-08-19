cmake_minimum_required(VERSION 3.8)
project(autonomy)

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
find_package(visualization_msgs REQUIRED)

add_library(autonomy_geometry src/geometry.cpp)
target_compile_features(autonomy_geometry PUBLIC cxx_std_17)
target_include_directories(autonomy_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(cone_detector src/cone_detector.cpp)
target_compile_features(cone_detector PUBLIC cxx_std_17)
target_include_directories(cone_detector PRIVATE include)
ament_target_dependencies(cone_detector
  geometry_msgs rclcpp sensor_msgs tf2 tf2_ros visualization_msgs)

add_executable(local_planner src/local_planner.cpp)
target_compile_features(local_planner PUBLIC cxx_std_17)
target_include_directories(local_planner PRIVATE include)
target_link_libraries(local_planner autonomy_geometry)
ament_target_dependencies(local_planner geometry_msgs nav_msgs rclcpp)

add_executable(track_reference_planner src/track_reference_planner.cpp)
target_compile_features(track_reference_planner PUBLIC cxx_std_17)
ament_target_dependencies(track_reference_planner
  fs_msgs geometry_msgs nav_msgs rclcpp std_msgs tf2)

add_executable(route_planner src/route_planner.cpp)
target_compile_features(route_planner PUBLIC cxx_std_17)
target_link_libraries(route_planner nlohmann_json::nlohmann_json)
ament_target_dependencies(route_planner
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_ros)

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_compile_features(pure_pursuit PUBLIC cxx_std_17)
target_include_directories(pure_pursuit PRIVATE include)
target_link_libraries(pure_pursuit autonomy_geometry)
ament_target_dependencies(pure_pursuit
  fs_msgs geometry_msgs nav_msgs rclcpp std_msgs std_srvs)

install(TARGETS autonomy_geometry cone_detector local_planner track_reference_planner route_planner pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_compile_features(test_geometry PUBLIC cxx_std_17)
  target_include_directories(test_geometry PRIVATE include)
  target_link_libraries(test_geometry autonomy_geometry)
endif()

ament_export_include_directories(include)
ament_export_libraries(autonomy_geometry)
ament_package()
