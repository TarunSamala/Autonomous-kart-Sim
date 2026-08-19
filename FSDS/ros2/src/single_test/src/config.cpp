#include "single_test/config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace single_test
{
namespace
{

template<typename T>
T required(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node || !node[key]) {
    throw std::invalid_argument(context + " is missing required field '" + key + "'");
  }
  return node[key].as<T>();
}

template<typename T>
T optional(
  const YAML::Node & node, const char * key, const T & default_value)
{
  return node && node[key] ? node[key].as<T>() : default_value;
}

bool valid_id(const std::string & value)
{
  return !value.empty() && std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isalnum(character) || character == '-' || character == '_';
    });
}

void positive(const double value, const std::string & field)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(field + " must be positive");
  }
}

}  // namespace

TestConfig parse_test_config(const YAML::Node & root)
{
  TestConfig result;
  result.schema_version = optional<int>(root, "schema_version", 1);
  if (result.schema_version != 1) {
    throw std::invalid_argument("unsupported test schema_version");
  }

  const auto test = root["test"];
  const auto sensors = root["sensors"];
  if (!test || !sensors) {
    throw std::invalid_argument("configuration requires 'test' and 'sensors' sections");
  }

  result.id = required<std::string>(test, "id", "test");
  result.track = required<std::string>(test, "track", "test");
  result.max_duration_s = optional<double>(test, "max_duration_s", 600.0);
  result.operation = optional<std::string>(test, "operation", "manual");
  result.lidar_profile = optional<std::string>(sensors, "lidar", "off");
  result.depth_profile = optional<std::string>(sensors, "depth_camera", "off");
  result.chassis_imu = optional<bool>(sensors, "chassis_imu", true);

  validate_test_config(result);
  return result;
}

void validate_test_config(const TestConfig & test)
{
  if (test.schema_version != 1) {
    throw std::invalid_argument("unsupported test schema_version");
  }
  if (!valid_id(test.id)) {
    throw std::invalid_argument("test.id may contain only letters, digits, '-' and '_'");
  }
  if (test.track.empty()) {
    throw std::invalid_argument("test.track must not be empty");
  }
  positive(test.max_duration_s, "test.max_duration_s");
  if (test.operation != "manual") {
    throw std::invalid_argument(
            "only operation 'manual' is supported while autonomy/navigation are excluded");
  }
  if (test.lidar_profile == "off" && test.depth_profile == "off") {
    throw std::invalid_argument("at least one of lidar or depth_camera must be enabled");
  }
}

ProfileCatalog parse_profile_catalog(const YAML::Node & root)
{
  ProfileCatalog result;
  result.schema_version = optional<int>(root, "schema_version", 1);
  if (result.schema_version != 1) {
    throw std::invalid_argument("unsupported profile schema_version");
  }

  for (const auto & entry : root["lidar"]) {
    const auto id = entry.first.as<std::string>();
    const auto node = entry.second;
    LidarProfile profile;
    profile.label = required<std::string>(node, "label", "lidar." + id);
    profile.manufacturer = required<std::string>(node, "manufacturer", "lidar." + id);
    profile.model = required<std::string>(node, "model", "lidar." + id);
    profile.source_url = optional<std::string>(node, "source_url", "");
    profile.range_m = required<double>(node, "range_m", "lidar." + id);
    profile.number_of_lasers = required<int>(node, "number_of_lasers", "lidar." + id);
    profile.points_per_scan = required<int>(node, "points_per_scan", "lidar." + id);
    profile.rotations_per_second = required<int>(node, "rotations_per_second", "lidar." + id);
    profile.horizontal_fov_start_deg = required<double>(
      node, "horizontal_fov_start_deg", "lidar." + id);
    profile.horizontal_fov_end_deg = required<double>(
      node, "horizontal_fov_end_deg", "lidar." + id);
    profile.vertical_fov_lower_deg = required<double>(
      node, "vertical_fov_lower_deg", "lidar." + id);
    profile.vertical_fov_upper_deg = required<double>(
      node, "vertical_fov_upper_deg", "lidar." + id);
    profile.fidelity_note = optional<std::string>(node, "fidelity_note", "");
    positive(profile.range_m, "lidar." + id + ".range_m");
    if (profile.number_of_lasers <= 0 || profile.points_per_scan <= 0 ||
      profile.rotations_per_second <= 0)
    {
      throw std::invalid_argument("lidar profile counts and rates must be positive: " + id);
    }
    result.lidar.emplace(id, std::move(profile));
  }

  for (const auto & entry : root["depth_camera"]) {
    const auto id = entry.first.as<std::string>();
    const auto node = entry.second;
    DepthProfile profile;
    profile.label = required<std::string>(node, "label", "depth_camera." + id);
    profile.manufacturer = required<std::string>(
      node, "manufacturer", "depth_camera." + id);
    profile.model = required<std::string>(node, "model", "depth_camera." + id);
    profile.source_url = optional<std::string>(node, "source_url", "");
    profile.width = required<int>(node, "width", "depth_camera." + id);
    profile.height = required<int>(node, "height", "depth_camera." + id);
    profile.frames_per_second = required<double>(
      node, "frames_per_second", "depth_camera." + id);
    profile.rgb_horizontal_fov_deg = required<double>(
      node, "rgb_horizontal_fov_deg", "depth_camera." + id);
    profile.depth_horizontal_fov_deg = required<double>(
      node, "depth_horizontal_fov_deg", "depth_camera." + id);
    profile.min_depth_m = required<double>(node, "min_depth_m", "depth_camera." + id);
    profile.max_depth_m = required<double>(node, "max_depth_m", "depth_camera." + id);
    if (node["baseline_mm"] && !node["baseline_mm"].IsNull()) {
      profile.baseline_mm = node["baseline_mm"].as<double>();
    }
    profile.integrated_imu = required<bool>(node, "integrated_imu", "depth_camera." + id);
    profile.fidelity_note = optional<std::string>(node, "fidelity_note", "");
    if (profile.width <= 0 || profile.height <= 0) {
      throw std::invalid_argument("depth profile dimensions must be positive: " + id);
    }
    positive(profile.frames_per_second, "depth_camera." + id + ".frames_per_second");
    positive(profile.min_depth_m, "depth_camera." + id + ".min_depth_m");
    positive(profile.max_depth_m, "depth_camera." + id + ".max_depth_m");
    if (profile.baseline_mm) {
      positive(*profile.baseline_mm, "depth_camera." + id + ".baseline_mm");
    }
    if (profile.max_depth_m <= profile.min_depth_m) {
      throw std::invalid_argument("depth profile max_depth_m must exceed min_depth_m: " + id);
    }
    result.depth.emplace(id, std::move(profile));
  }

  if (result.lidar.empty() || result.depth.empty()) {
    throw std::invalid_argument("profile catalog requires lidar and depth_camera profiles");
  }
  return result;
}

TestConfig load_test_config(const std::string & path)
{
  return parse_test_config(YAML::LoadFile(path));
}

ProfileCatalog load_profile_catalog(const std::string & path)
{
  return parse_profile_catalog(YAML::LoadFile(path));
}

ResolvedTest resolve_test(const TestConfig & test, const ProfileCatalog & catalog)
{
  ResolvedTest result;
  result.test = test;
  if (test.lidar_profile != "off") {
    const auto found = catalog.lidar.find(test.lidar_profile);
    if (found == catalog.lidar.end()) {
      throw std::invalid_argument("unknown lidar profile: " + test.lidar_profile);
    }
    result.lidar = found->second;
  }
  if (test.depth_profile != "off") {
    const auto found = catalog.depth.find(test.depth_profile);
    if (found == catalog.depth.end()) {
      throw std::invalid_argument("unknown depth_camera profile: " + test.depth_profile);
    }
    result.depth = found->second;
  }
  return result;
}

}  // namespace single_test
