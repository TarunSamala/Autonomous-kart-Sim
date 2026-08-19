# Single Test

This C++ package prepares and records one reproducible FSDS experiment. It does
not launch autonomy or navigation. The initial operation mode is deliberately
`manual`, allowing keyboard driving while sensor, mapping, localization, and
loop-closure behavior are evaluated independently.

## Contract

One test YAML selects a track and any valid LiDAR/depth-camera combination.
`prepare_single_test` validates that selection and writes:

1. A generated FSDS `settings.json`, consumed when the simulator starts.
2. An immutable manifest embedded into the final result.

Available profiles are defined in `config/sensor_profiles.yaml`. They are
hardware-inspired simulation contracts, not claims that Unreal reproduces all
optical, timing, material, sunlight, or USB behavior of the physical products.

## Prepare

From the ROS 2 workspace after building:

```zsh
ros2 run single_test prepare_single_test -- \
  --config src/single_test/config/example_single_test.yaml \
  --profiles src/single_test/config/sensor_profiles.yaml \
  --base-settings ../settings.json \
  --output-settings generated/single_test/settings.json \
  --output-manifest generated/single_test/manifest.json
```

For routine tests, select profiles directly from the command line:

```zsh
# LiDAR only
scripts/single-test-prepare --lidar a1m8 --depth-camera off

# Depth camera only
scripts/single-test-prepare --lidar off --depth-camera d455

# Combined sensors
scripts/single-test-prepare --lidar a3m1 --depth-camera d435
```

Valid LiDAR profiles are `a1m8`, `a3m1`, and `fsds_16`. Valid depth profiles
are `hp60c`, `d415`, `d435`, and `d455`. A custom test YAML and output
directory can still be passed as the first two positional arguments.

Never overwrite the repository's base `FSDS/settings.json` for an experiment.
Restart FSDS with the generated file through its `-settings` argument; sensor
changes are read only when the simulator starts:

```zsh
<path-to-release>/FSDS.sh \
  -settings "$PWD/generated/single_test/settings.json"
```

Then start the bridge with the same generated file:

```zsh
scripts/single-test-bridge
```

The bridge reads `FSDS_SETTINGS_PATH`, creates only the selected camera topics,
and derives its LiDAR polling period from the enabled profile's scan frequency.

## Visualize sensors

After FSDS and `scripts/single-test-bridge` are running:

```zsh
scripts/single-test-visualize
```

This opens RViz2 with Best Effort sensor QoS and displays the LiDAR cloud,
front RGB image, metric depth image, and a depth-derived point cloud. The
point-cloud converter uses depth calibration instead of assuming RGB/depth
registration, so it works for D415 and D435 profiles with different fields of
view. Disable parts of the launch when needed:

```zsh
scripts/single-test-visualize point_cloud:=false
scripts/single-test-visualize rviz:=false
```

If a selected sensor is `off`, its preconfigured RViz display simply reports
that no topic is available; it does not affect the enabled sensor.

Expected topics by selection:

| Selection | Topics |
|---|---|
| LiDAR on | `/fsds/lidar/Lidar1` |
| Depth camera on | `/fsds/front_rgb/image_color`, `/fsds/front_depth/image_depth`, both `camera_info` topics |
| Visualization point cloud | `/fsds/front_depth/points` |
| Chassis IMU on | `/fsds/imu` |

Useful terminal checks are:

```zsh
ros2 topic list | sort
ros2 topic hz /fsds/lidar/Lidar1
ros2 topic hz /fsds/front_depth/image_depth
ros2 topic info --verbose /fsds/front_depth/points
```

## Record a manual test

```zsh
ros2 run single_test single_test_session --ros-args \
  -p manifest_path:=/absolute/path/to/manifest.json \
  -p results_directory:=/absolute/path/to/results

ros2 service call /single_test/start std_srvs/srv/Trigger '{}'
# Drive manually, and later:
ros2 service call /single_test/stop std_srvs/srv/Trigger '{}'
```

The result includes duration, integrated distance, average/max speed, lap
times, down/out cone count, completion reason, and the exact manifest.

## Next localization milestone

After this preparation layer is validated, add a mapping session type around
the existing RTAB-Map baseline:

1. Build a map while manually driving and save its database.
2. Verify loop closures and graph corrections against FSDS ground truth.
3. Restart in localization-only mode using the saved map.
4. Record translational/yaw error, relocalization time, loop-closure count, and
   tracking-loss intervals in the Single Test result.
