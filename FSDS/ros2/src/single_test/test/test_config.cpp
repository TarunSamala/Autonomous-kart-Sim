#include "single_test/config.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace
{

const char * profiles = R"(
schema_version: 1
lidar:
  a3m1:
    label: A3
    manufacturer: SLAMTEC
    model: A3M1
    range_m: 25
    number_of_lasers: 1
    points_per_scan: 1600
    rotations_per_second: 10
    horizontal_fov_start_deg: -180
    horizontal_fov_end_deg: 180
    vertical_fov_lower_deg: 0
    vertical_fov_upper_deg: 0
depth_camera:
  d455:
    label: D455
    manufacturer: Intel RealSense
    model: D455
    width: 848
    height: 480
    frames_per_second: 30
    rgb_horizontal_fov_deg: 86
    depth_horizontal_fov_deg: 86
    min_depth_m: 0.4
    max_depth_m: 6
    baseline_mm: 95
    integrated_imu: true
)";

TEST(SingleTestConfig, ResolvesCombinedSensors)
{
  const auto test = single_test::parse_test_config(YAML::Load(R"(
schema_version: 1
test: {id: mapping_01, track: TrainingMap, operation: manual}
sensors: {lidar: a3m1, depth_camera: d455, chassis_imu: true}
)"));
  const auto resolved = single_test::resolve_test(
    test, single_test::parse_profile_catalog(YAML::Load(profiles)));
  ASSERT_TRUE(resolved.lidar.has_value());
  ASSERT_TRUE(resolved.depth.has_value());
  EXPECT_EQ(resolved.lidar->number_of_lasers, 1);
  EXPECT_EQ(resolved.depth->width, 848);
}

TEST(SingleTestConfig, RejectsAutonomousOperation)
{
  EXPECT_THROW(single_test::parse_test_config(YAML::Load(R"(
test: {id: invalid, track: TrainingMap, operation: autonomous}
sensors: {lidar: a3m1, depth_camera: off}
)")), std::invalid_argument);
}

TEST(SingleTestConfig, RejectsUnknownProfile)
{
  const auto test = single_test::parse_test_config(YAML::Load(R"(
test: {id: invalid_profile, track: TrainingMap}
sensors: {lidar: unknown, depth_camera: off}
)"));
  EXPECT_THROW(single_test::resolve_test(
    test, single_test::parse_profile_catalog(YAML::Load(profiles))), std::invalid_argument);
}

TEST(SingleTestConfig, RequiresAtLeastOnePrimarySensor)
{
  EXPECT_THROW(single_test::parse_test_config(YAML::Load(R"(
test: {id: no_sensors, track: TrainingMap}
sensors: {lidar: off, depth_camera: off}
)")), std::invalid_argument);
}

}  // namespace
