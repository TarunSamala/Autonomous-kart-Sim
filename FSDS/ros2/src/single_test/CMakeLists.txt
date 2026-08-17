cmake_minimum_required(VERSION 3.8)
project(single_test)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(single_test_config src/config.cpp)
target_compile_features(single_test_config PUBLIC cxx_std_17)
target_include_directories(single_test_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(single_test_config yaml-cpp)

add_executable(prepare_single_test src/prepare.cpp)
target_compile_features(prepare_single_test PUBLIC cxx_std_17)
target_link_libraries(prepare_single_test single_test_config nlohmann_json::nlohmann_json)

add_executable(single_test_session src/session.cpp)
target_compile_features(single_test_session PUBLIC cxx_std_17)
target_link_libraries(single_test_session nlohmann_json::nlohmann_json)
ament_target_dependencies(single_test_session
  fs_msgs geometry_msgs rclcpp std_msgs std_srvs)

install(TARGETS single_test_config prepare_single_test single_test_session
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_config test/test_config.cpp)
  target_compile_features(test_config PUBLIC cxx_std_17)
  target_link_libraries(test_config single_test_config yaml-cpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(single_test_config)
ament_package()
