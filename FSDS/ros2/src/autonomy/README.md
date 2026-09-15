# Autonomy

This package is an entirely C++ LiDAR autonomy baseline for FSDS:

```text
/fsds/lidar/Lidar1
  -> cone_detector
  -> /autonomy/cones/left + /autonomy/cones/right
  -> local_planner
  -> /autonomy/local_path
  -> pure_pursuit
  -> /fsds/control_command
```

## AMZ-style teach and repeat

The `amz_style` experiment is a two-stage workflow:

1. Manually drive one or two complete laps while SLAM localizes the kart.
2. Accumulate left/right cone landmarks and raw Delaunay gate midpoints in
   `slam/map` instead of discarding detections as they leave sensor range.
3. Accept loop return only after the configured travel distance and heading
   agreement are satisfied near the starting pose.
4. Save the SLAM pose graph, observed cone map, manual route, and ordered
   Delaunay centreline.
5. Stop keyboard command publication and launch saved-map localization.
6. Keep control disabled until the readiness gate passes and the operator
   confirms arming.

Prepare the sensor profile before starting FSDS, then run the integrated flow:

```zsh
FSDS/ros2/scripts/single-test prepare amz_style
# Start FSDS, then:
FSDS/ros2/scripts/single-test teach-run --laps 1
```

Press `E` to arm keyboard control, then use `W/A/S/D`. RViz displays the live
triangulation, persistent cone landmarks, accumulated Delaunay centreline, and
manual route. On accepted loop return the teach process brakes, saves the map,
and transitions to autonomous replay. The replay follows
`generated/maps/TrainingMap/centerline_delaunay.json`; the manual route is kept
as evidence and ordering context, not used as the commanded path.

`--auto-arm` is available for unattended simulation runs, but the default
confirmation gate is recommended during development.

## Reactive Delaunay planner

The `delaunay_reactive` profile uses an AMZ-inspired local strategy without
requiring VIO, VSLAM, a saved map, or FSDS testing-only track geometry:

1. Combine the currently detected left and right cone positions.
2. Build a Bowyer-Watson Delaunay triangulation.
3. Keep opposite-boundary Delaunay edges whose lengths are valid track widths.
4. Treat the valid edge midpoints as centerline gates.
5. Connect gates that share a Delaunay triangle.
6. Grow possible centerlines with a bounded beam search.
7. Rank paths by heading smoothness, gate-width consistency, point spacing,
   sensor-horizon coverage, start alignment, and boundary-color ordering.
8. Publish the lowest-cost path to the existing Pure Pursuit controller.

If triangulation cannot produce a safe two-point path, the planner falls back
to paired cones and then to a conservative single-boundary offset. An empty
path is published if no method is safe enough; the controller then brakes on
its existing stale/path checks.

Debug output is published on `/autonomy/debug/delaunay_planner`: dark lines are
the triangulation, pale blue lines are valid cross-track gates, green lines are
beam-search candidates, and the thick red line is the selected candidate. The
magenta `/autonomy/local_path` is the smoothed path actually consumed by the
controller.

Raw valid cross-track gate midpoints are published on
`/autonomy/delaunay_midpoints`. During teaching, `cone_map_builder` transforms
and associates them in `slam/map`, publishes `/autonomy/map/centerline`, and
saves the ordered centreline after loop return.

Launch perception and planning disabled, with the dedicated top-down view:

```zsh
ros2 launch autonomy fsds_autonomy.launch.xml enabled:=false rviz:=true
```

Before arming, confirm that the selected red/magenta path stays between the
detected boundaries through straights and corners:

```zsh
ros2 topic hz /autonomy/local_path
ros2 topic echo /autonomy/local_path --once
ros2 topic hz /autonomy/debug/delaunay_planner
```

The controller starts disabled, requires fresh LiDAR-derived path and GSS data,
and requires the FSDS GO signal. Missing or stale input commands full brake.
Do not run keyboard teleop simultaneously because it publishes to the same
control topic.

## LiDAR profile

The active `FSDS/settings.json` must have `Lidar1` enabled before FSDS starts.
For the first functional test, prepare the `fsds_16` development profile; it
configures `Lidar1` at its existing mount with:

- 16 vertical layers spanning -20 through +5 degrees
- 4096 firings per scan
- a 180 degree forward horizontal field of view
- 10 scans per second

The bridge publishes the cloud in `fsds/Lidar1`. The detector uses the bridge's
static TF to transform cone centroids into `fsds/FSCar`; it does not duplicate
the sensor mounting transform in autonomy code.

## Static-friction launch assist

FSDS uses Unreal PhysX for the vehicle model and does not document a universal
minimum throttle. The controller therefore applies a bounded launch assist:

1. Apply `launch_throttle` while measured GSS speed is below
   `launch_release_speed`.
2. Stop the boost immediately when release speed is reached and continue with
   normal path-following throttle.
3. If speed is not reached within `launch_timeout`, command full brake and
   disarm. It never retries automatically.

Defaults are deliberately conservative: a 0.55 throttle pulse for 0.10 seconds,
a 0.10 m/s release speed, and a 0.75 second failure timeout. Tune these values
in `config/autonomy.yaml` from recorded GSS and command data rather than
changing Unreal friction blindly.

## Build

Inside `kart-ros2-humble`:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to autonomy
source install/setup.zsh
```

## Prepare FSDS

Before launching FSDS, apply the Phase 1 sensor contract from the repository
root. This enables the dense development LiDAR and keeps the D455 stream
available for visualization and later algorithms:

```zsh
FSDS/ros2/scripts/single-test-prepare \
  FSDS/ros2/src/single_test/config/delaunay_phase1.yaml
```

The command validates and replaces `FSDS/settings.json`, preserving its first
pre-test version as `FSDS/settings.json.pre-single-test`. Restart FSDS after
every sensor-profile change because AirSim reads sensor settings at startup.

## Validate without movement

Start FSDS and the bridge, then launch disabled:

```zsh
ros2 launch autonomy fsds_autonomy.launch.xml
```

Check the full data contract before arming:

```zsh
ros2 topic hz /fsds/lidar/Lidar1
ros2 topic echo /autonomy/cones/left --once
ros2 topic echo /autonomy/cones/right --once
ros2 topic echo /autonomy/local_path --once
ros2 topic echo /autonomy/status
```

In RViz, use `fsds/FSCar` as the fixed frame. Add the two cone `PoseArray`
topics, `/autonomy/local_path` as `Path`, and the two debug cone marker topics.

## First controlled movement

Keep the simulator reset position clear and arm from a separate terminal:

```zsh
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: true}'
```

Disarm immediately at any time:

```zsh
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: false}'
```

Observe launch behavior:

```zsh
ros2 topic echo /autonomy/status
ros2 topic echo /fsds/gss
```

Expected status progression is `WAITING_FOR_INPUTS -> LAUNCH_ASSIST -> TRACKING`.
`LAUNCH_FAILED_BRAKING` means the bounded boost could not overcome the initial
vehicle resistance and the controller has already disarmed itself.
After launch, throttle tapers toward zero at the configured `target_speed`.
`OVERSPEED_BRAKING` applies the configured brake whenever `max_speed` is reached.
