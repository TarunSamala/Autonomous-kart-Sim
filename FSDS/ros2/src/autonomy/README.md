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

The controller starts disabled, requires fresh LiDAR-derived path and GSS data,
and requires the FSDS GO signal. Missing or stale input commands full brake.
Do not run keyboard teleop simultaneously because it publishes to the same
control topic.

## LiDAR profile

`FSDS/settings.json` configures `Lidar1` at `(0.45, 0.0, 0.55)` m with:

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

Defaults are deliberately conservative: 0.45 throttle, 0.50 m/s release speed,
and 0.80 seconds maximum. Tune these values in `config/autonomy.yaml` from
recorded GSS and command data rather than changing Unreal friction blindly.

## Build

Inside `kart-ros2-humble`:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to autonomy
source install/setup.zsh
```

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
