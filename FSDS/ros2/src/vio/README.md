# VIO

This package establishes the simulator-side visual-inertial odometry contract.
It launches synchronized RGB-D odometry and an IMU adapter, publishing:

```text
/localization/vio/odometry
```

## Important simulation limitation

FSDS v2.2 exposes one chassis IMU and does not support positioning another IMU
at the camera through `settings.json`. The adapter republishes that measurement
as `/sensors/imu/camera` with the camera-IMU frame. This is useful for timing,
TF, synchronization and pipeline development, but it does not simulate
camera lever-arm acceleration. The physical D456 must use its own IMU stream.

RTAB-Map's RGB-D odometry uses the IMU for initialization and gravity alignment;
it is the validation baseline, not the final tightly coupled VIO backend.

## D455 simulation profile

The FSDS camera pair uses 848x480 at 30 Hz with an 87 degree horizontal FOV.
At this aspect ratio, the calculated vertical FOV is approximately 57 degrees.
VIO accepts feature depths from 0.4 m through 6.0 m, matching the D455's optimal
working range. The raw FSDS depth topic remains unclipped for diagnostics and
other algorithms.

FSDS renders depth directly, so the 95 mm physical stereo baseline does not
participate in its depth calculation. FSDS v2.2 also cannot model the D455's
separate camera-mounted IMU; see the limitation above.

## Run

Start FSDS and its ROS bridge, then:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch vio fsds_vio.launch.py
```

If the repository shortcut is installed, the same launch can be started from
either the host or inside Distrobox with:

```zsh
vio-start
```

The registered colored point cloud is enabled by default at
`/fsds/front_depth/points`. Disable it when it is not needed:

```zsh
vio-start point_cloud:=false
```

Keep `publish_tf:=false` while the bridge publishes the simulator ground-truth
transform to `fsds/FSCar`. Ground truth is for evaluation only and must never be
used as odometry input to this node.

Check output:

```zsh
ros2 topic hz /localization/vio/odometry
ros2 topic echo /localization/vio/odometry --once
```
