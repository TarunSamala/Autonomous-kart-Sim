#include "single_test/config.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{

struct Arguments
{
  std::string config;
  std::string profiles;
  std::string base_settings;
  std::string output_settings;
  std::string output_manifest;
  std::optional<std::string> lidar_profile;
  std::optional<std::string> depth_profile;
  bool list_profiles{false};
};

std::string next_value(int & index, const int argc, char ** argv, const std::string & option)
{
  if (++index >= argc) {
    throw std::invalid_argument(option + " requires a value");
  }
  return argv[index];
}

Arguments parse_arguments(const int argc, char ** argv)
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--config") {
      result.config = next_value(index, argc, argv, option);
    } else if (option == "--profiles") {
      result.profiles = next_value(index, argc, argv, option);
    } else if (option == "--base-settings") {
      result.base_settings = next_value(index, argc, argv, option);
    } else if (option == "--output-settings") {
      result.output_settings = next_value(index, argc, argv, option);
    } else if (option == "--output-manifest") {
      result.output_manifest = next_value(index, argc, argv, option);
    } else if (option == "--lidar") {
      result.lidar_profile = next_value(index, argc, argv, option);
    } else if (option == "--depth-camera") {
      result.depth_profile = next_value(index, argc, argv, option);
    } else if (option == "--list-profiles") {
      result.list_profiles = true;
    } else if (option == "--help" || option == "-h") {
      std::cout <<
        "prepare_single_test --config TEST.yaml --profiles sensor_profiles.yaml "
        "--base-settings settings.json --output-settings generated.json "
        "--output-manifest manifest.json [--lidar PROFILE|off] "
        "[--depth-camera PROFILE|off]\n"
        "prepare_single_test --profiles sensor_profiles.yaml --list-profiles\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + option);
    }
  }
  if (result.profiles.empty()) {
    throw std::invalid_argument("--profiles is required");
  }
  if (!result.list_profiles &&
    (result.config.empty() || result.base_settings.empty() ||
    result.output_settings.empty() || result.output_manifest.empty()))
  {
    throw std::invalid_argument(
            "--config, --base-settings, --output-settings and --output-manifest are required");
  }
  return result;
}

json read_json(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open JSON file: " + path);
  }
  return json::parse(stream);
}

void write_json(const std::string & path, const json & value)
{
  const auto parent = fs::absolute(path).parent_path();
  fs::create_directories(parent);
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot write JSON file: " + path);
  }
  stream << std::setw(2) << value << '\n';
}

json lidar_manifest(
  const std::string & id, const single_test::LidarProfile & profile)
{
  return {
    {"profile", id}, {"label", profile.label},
    {"manufacturer", profile.manufacturer}, {"model", profile.model},
    {"source_url", profile.source_url},
    {"range_m", profile.range_m}, {"number_of_lasers", profile.number_of_lasers},
    {"points_per_scan", profile.points_per_scan},
    {"rotations_per_second", profile.rotations_per_second},
    {"horizontal_fov_deg", {
        profile.horizontal_fov_start_deg, profile.horizontal_fov_end_deg}},
    {"vertical_fov_deg", {
        profile.vertical_fov_lower_deg, profile.vertical_fov_upper_deg}},
    {"fidelity_note", profile.fidelity_note}
  };
}

json depth_manifest(
  const std::string & id, const single_test::DepthProfile & profile)
{
  return {
    {"profile", id}, {"label", profile.label},
    {"manufacturer", profile.manufacturer}, {"model", profile.model},
    {"source_url", profile.source_url},
    {"resolution", {profile.width, profile.height}},
    {"frames_per_second", profile.frames_per_second},
    {"rgb_horizontal_fov_deg", profile.rgb_horizontal_fov_deg},
    {"depth_horizontal_fov_deg", profile.depth_horizontal_fov_deg},
    {"depth_range_m", {profile.min_depth_m, profile.max_depth_m}},
    {"baseline_mm", profile.baseline_mm ? json(*profile.baseline_mm) : json(nullptr)},
    {"integrated_imu", profile.integrated_imu},
    {"fidelity_note", profile.fidelity_note}
  };
}

void apply_lidar(json & vehicle, const single_test::ResolvedTest & resolved)
{
  auto & sensors = vehicle.at("Sensors");
  auto & primary = sensors.at("Lidar1");
  primary["Enabled"] = resolved.lidar.has_value();
  if (sensors.contains("Lidar2")) {
    sensors["Lidar2"]["Enabled"] = false;
  }
  if (!resolved.lidar) {
    return;
  }
  const auto & profile = *resolved.lidar;
  primary["Range"] = profile.range_m;
  primary["NumberOfLasers"] = profile.number_of_lasers;
  primary["PointsPerScan"] = profile.points_per_scan;
  primary["RotationsPerSecond"] = profile.rotations_per_second;
  primary["HorizontalFOVStart"] = profile.horizontal_fov_start_deg;
  primary["HorizontalFOVEnd"] = profile.horizontal_fov_end_deg;
  primary["VerticalFOVLower"] = profile.vertical_fov_lower_deg;
  primary["VerticalFOVUpper"] = profile.vertical_fov_upper_deg;
  primary["DrawDebugPoints"] = false;
}

void apply_depth(json & vehicle, const single_test::ResolvedTest & resolved)
{
  auto & cameras = vehicle.at("Cameras");
  auto & rgb = cameras.at("front_rgb");
  auto & depth = cameras.at("front_depth");
  rgb["RosEnabled"] = resolved.depth.has_value();
  depth["RosEnabled"] = false;
  if (!resolved.depth) {
    rgb.erase("RosDepthCamera");
    return;
  }
  const auto & profile = *resolved.depth;
  rgb["RosDepthCamera"] = "front_depth";
  rgb["RosFramerate"] = profile.frames_per_second;
  depth["RosFramerate"] = profile.frames_per_second;
  rgb["CaptureSettings"][0]["Width"] = profile.width;
  rgb["CaptureSettings"][0]["Height"] = profile.height;
  rgb["CaptureSettings"][0]["FOV_Degrees"] = profile.rgb_horizontal_fov_deg;
  depth["CaptureSettings"][0]["Width"] = profile.width;
  depth["CaptureSettings"][0]["Height"] = profile.height;
  depth["CaptureSettings"][0]["FOV_Degrees"] = profile.depth_horizontal_fov_deg;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto catalog = single_test::load_profile_catalog(arguments.profiles);
    if (arguments.list_profiles) {
      std::cout << "LiDAR profiles:\n";
      for (const auto & [id, profile] : catalog.lidar) {
        std::cout << "  " << id << " - " << profile.label << '\n';
      }
      std::cout << "Depth camera profiles:\n";
      for (const auto & [id, profile] : catalog.depth) {
        std::cout << "  " << id << " - " << profile.label << '\n';
      }
      return 0;
    }

    auto test = single_test::load_test_config(arguments.config);
    if (arguments.lidar_profile) {
      test.lidar_profile = *arguments.lidar_profile;
    }
    if (arguments.depth_profile) {
      test.depth_profile = *arguments.depth_profile;
    }
    single_test::validate_test_config(test);
    const auto resolved = single_test::resolve_test(test, catalog);
    auto settings = read_json(arguments.base_settings);
    auto & vehicle = settings.at("Vehicles").at("FSCar");
    apply_lidar(vehicle, resolved);
    apply_depth(vehicle, resolved);
    vehicle.at("Sensors").at("Imu")["Enabled"] = test.chassis_imu;

    json manifest = {
      {"schema_version", 1},
      {"created_unix_s", std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch()).count()},
      {"test", {
          {"id", test.id}, {"track", test.track},
          {"max_duration_s", test.max_duration_s},
          {"operation", test.operation}}},
      {"sensors", {
          {"lidar", resolved.lidar ?
            lidar_manifest(test.lidar_profile, *resolved.lidar) : json(nullptr)},
          {"depth_camera", resolved.depth ?
            depth_manifest(test.depth_profile, *resolved.depth) : json(nullptr)},
          {"chassis_imu", test.chassis_imu}}},
      {"excluded", {"autonomy", "navigation"}},
      {"source", {
          {"test_config", fs::absolute(arguments.config).string()},
          {"profile_catalog", fs::absolute(arguments.profiles).string()},
          {"base_settings", fs::absolute(arguments.base_settings).string()}}}
    };

    write_json(arguments.output_settings, settings);
    write_json(arguments.output_manifest, manifest);
    std::cout << "Prepared Single Test '" << test.id << "'\n"
              << "  settings: " << fs::absolute(arguments.output_settings) << '\n'
              << "  manifest: " << fs::absolute(arguments.output_manifest) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "prepare_single_test: " << error.what() << '\n';
    return 1;
  }
}
