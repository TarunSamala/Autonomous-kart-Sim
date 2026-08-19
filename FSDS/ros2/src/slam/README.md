# LiDAR SLAM

This package provides a sensor-derived SLAM Toolbox pipeline for FSDS:

```text
/fsds/lidar/Lidar1 (PointCloud2) -> /sensors/lidar/scan (LaserScan)
chassis IMU + GSS -> sensor_odometry -> sensor/odom -> fsds/FSCar
LaserScan + odometry -> SLAM Toolbox -> slam/map -> sensor/odom
```

Neither launch file consumes `/fsds/testing_only/odom` or
`/fsds/testing_only/track`. Those topics remain evaluation-only.

The deterministic chassis-sensor odometry is the default. Append
`use_vio:=true` to either launch command to benchmark RGB-D VIO as the motion
prior instead.

## Build and map

```zsh
colcon build --symlink-install --packages-up-to slam
source install/setup.zsh
ros2 launch slam fsds_slam_mapping.launch.xml
```

Validate the live chain:

```zsh
ros2 topic hz /sensors/lidar/scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo slam/map fsds/FSCar
```

After closing the loop, serialize the pose graph and save the occupancy map:

```zsh
mkdir -p ~/.ros/fsds_slam
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/.ros/fsds_slam/track'}"
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap "{name: {data: '$HOME/.ros/fsds_slam/track'}}"
```

## Localize in the saved graph

```zsh
ros2 launch slam fsds_slam_localization.launch.xml \
  map_file:=$HOME/.ros/fsds_slam/track
```

The serialized graph is required for SLAM Toolbox localization; the `.pgm`
and `.yaml` occupancy-map files alone are not sufficient.

## Follow a recorded route

The complete saved-map pipeline is:

```text
LiDAR + chassis IMU/GSS -> SLAM Toolbox localization
saved route + localized pose -> C++ route planner -> C++ Pure Pursuit
```

Launch it through the reproducible Single Test wrapper:

```zsh
scripts/single-test-autonomy
# In another terminal, after TRACKING_ROUTE appears:
scripts/single-test-arm
```

Decision nodes do not subscribe to `/fsds/testing_only/track` or
`/fsds/testing_only/odom`. `/fsds/testing_only/extra_info` is consumed only as
the benchmark/safety authority for lap completion and cone contact.
