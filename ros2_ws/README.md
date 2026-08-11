# ROS 2 workspace

This is the main autonomy workspace. Package names intentionally omit a project
prefix so that the software graph stays concise.

## Package plan

| Package | State | Responsibility |
| --- | --- | --- |
| `interfaces` | Planned | Shared messages and services |
| `sensor_interface` | Planned | Simulator/RealSense and vehicle sensor adapters |
| `teleop` | Implemented | Manual keyboard control |
| `vio` | Planned | Visual-inertial odometry |
| `vslam` | Planned | Mapping, loop closure and global correction |
| `state_estimator` | Planned | VIO, wheel odometry and chassis IMU fusion |
| `perception` | Planned | Cones, obstacles and free-space estimation |
| `control` | Planned | Path tracking and command arbitration |

Generated `build`, `install`, and `log` directories are not source and must not
be committed.

## Build

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/ros2_ws
source /opt/ros/humble/setup.zsh
source ../FSDS/ros2/install/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Source order

Every new terminal should source ROS, the FSDS bridge overlay, and then this
workspace in that order.
