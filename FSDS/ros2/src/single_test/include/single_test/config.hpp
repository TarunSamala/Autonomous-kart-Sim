#pragma once

#include <map>
#include <optional>
#include <string>

#include <yaml-cpp/yaml.h>

namespace single_test
{

struct TestConfig
{
  int schema_version{1};
  std::string id;
  std::string track;
  double max_duration_s{600.0};
  std::string operation{"manual"};
  std::string lidar_profile{"off"};
  std::string depth_profile{"off"};
  bool chassis_imu{true};
};

struct LidarProfile
{
  std::string label;
  std::string manufacturer;
  std::string model;
  std::string source_url;
  double range_m{};
  int number_of_lasers{};
  int points_per_scan{};
  int rotations_per_second{};
  double horizontal_fov_start_deg{};
  double horizontal_fov_end_deg{};
  double vertical_fov_lower_deg{};
  double vertical_fov_upper_deg{};
  std::string fidelity_note;
};

struct DepthProfile
{
  std::string label;
  std::string manufacturer;
  std::string model;
  std::string source_url;
  int width{};
  int height{};
  double frames_per_second{};
  double rgb_horizontal_fov_deg{};
  double depth_horizontal_fov_deg{};
  double min_depth_m{};
  double max_depth_m{};
  double baseline_mm{};
  bool integrated_imu{};
  std::string fidelity_note;
};

struct ProfileCatalog
{
  int schema_version{1};
  std::map<std::string, LidarProfile> lidar;
  std::map<std::string, DepthProfile> depth;
};

struct ResolvedTest
{
  TestConfig test;
  std::optional<LidarProfile> lidar;
  std::optional<DepthProfile> depth;
};

TestConfig parse_test_config(const YAML::Node & root);
ProfileCatalog parse_profile_catalog(const YAML::Node & root);
TestConfig load_test_config(const std::string & path);
ProfileCatalog load_profile_catalog(const std::string & path);
ResolvedTest resolve_test(const TestConfig & test, const ProfileCatalog & catalog);

}  // namespace single_test
