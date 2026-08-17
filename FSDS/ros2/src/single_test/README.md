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

Never overwrite the repository's base `FSDS/settings.json` for an experiment.
Launch FSDS with the generated file through its `-settings` argument. Start the
bridge with the same generated file:

```zsh
scripts/single-test-bridge
```

The bridge reads `FSDS_SETTINGS_PATH`, creates only the selected camera topics,
and derives its LiDAR polling period from the enabled profile's scan frequency.

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
