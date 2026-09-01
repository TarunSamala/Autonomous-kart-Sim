# Single Test

This C++ package prepares and records one reproducible FSDS experiment. It
supports `manual`, `mapping`, `localization`, and `autonomy` operations. Each
result embeds the selected sensor profiles, algorithms, map/route artifacts,
lap metrics, and cone-contact count.

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

The `scripts/single-test-prepare` wrapper writes the generated contract under
`generated/single_test/`, preserves the original active settings once as
`FSDS/settings.json.pre-single-test`, and applies the generated settings to the
canonical `FSDS/settings.json`. Sensor changes are read only when the simulator
starts, so restart the locally built editor with:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim
FSDS/scripts/fsds-editor
```

Then start the bridge. By default, both the simulator launcher and this bridge
wrapper read the canonical active `FSDS/settings.json`, preventing drift
between two settings copies:

```zsh
scripts/single-test-bridge
```

The bridge exports `FSDS_SETTINGS_PATH` to that canonical file, creates only
the selected camera topics, and derives its LiDAR polling period from the
enabled profile's scan frequency. Passing an explicit settings path to
`scripts/single-test-bridge` remains available for advanced isolated runs, but
the simulator must be launched with the same explicit path.

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
An optional `run_metadata_path` parameter embeds additional immutable JSON such
as behavior-model training and validation metadata. Setting
`stop_on_cone_contact:=true` ends and writes the run on the first new cone
contact.

## Run the saved-map autonomous benchmark

Prepare the immutable A3M1 + D455, SLAM Toolbox, recorded-route, Pure Pursuit
contract without overwriting the active simulator settings:

```zsh
ros2 run single_test prepare_single_test -- \
  --config src/single_test/config/slam_toolbox_trackdrive.yaml \
  --profiles src/single_test/config/sensor_profiles.yaml \
  --base-settings ../settings.json \
  --output-settings generated/single_test/slam_toolbox_trackdrive/settings.json \
  --output-manifest generated/single_test/slam_toolbox_trackdrive/manifest.json
```

With FSDS and the bridge running, launch localization, route following, and the
benchmark recorder in one terminal:

```zsh
scripts/single-test-autonomy
```

Wait for `route_planner` to report `TRACKING_ROUTE`, then arm both the recorder
and controller from another terminal:

```zsh
scripts/single-test-arm
```

The controller starts disabled, brakes on stale/missing inputs, stops on the
first new cone contact, ignores the short initial start-line event, and stops
after one full lap. Results are written under `results/`.
