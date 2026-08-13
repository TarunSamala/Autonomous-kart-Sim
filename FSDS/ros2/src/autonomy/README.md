# Autonomy

This ROS 2 package provides the first local FSDS autonomy loop:

```text
/fsds/lidar/Lidar1
  -> cone_detector
  -> /autonomy/cones/left + /autonomy/cones/right
  -> local_planner
  -> /autonomy/local_path
  -> pure_pursuit
  -> /fsds/control_command
```

The launch starts with autonomy disabled and continuously commands full brake.
It also stops if the path, lidar perception, or odometry becomes stale. Do not
run keyboard teleop simultaneously because both publish control commands.

## Build

Inside `kart-ros2-humble`:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to autonomy
source install/setup.zsh
```

## Run safely

Start FSDS and the ROS bridge, then:

```zsh
ros2 launch autonomy fsds_autonomy.launch.py
```

Verify perception and planning before enabling movement:

```zsh
ros2 topic hz /fsds/lidar/Lidar1
ros2 topic echo /autonomy/cones/left --once
ros2 topic echo /autonomy/cones/right --once
ros2 topic echo /autonomy/local_path --once
```

In RViz, set the fixed frame to `fsds/FSCar`. Display the two cone topics as
`PoseArray` and the local path as `Path`.

Enable:

```zsh
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: true}'
```

Disable immediately:

```zsh
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: false}'
```
