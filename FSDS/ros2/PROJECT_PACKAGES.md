# Project ROS 2 packages

Project packages live beside the FSDS bridge in `FSDS/ros2/src`. Package names
intentionally omit a project prefix so that the software graph stays concise.

## Package plan

| Package | State | Responsibility |
| --- | --- | --- |
| `single_test` | Implemented baseline | Sensor profile preparation, lifecycle and result recording |
| `interfaces` | Planned | Shared messages and services |
| `sensor_interface` | Implemented | Simulator/RealSense and vehicle sensor adapters |
| `teleop` | Implemented | Manual keyboard control |
| `vio` | Baseline | Synchronized RGB-D odometry with IMU initialization |
| `vslam` | Baseline | RGB-D mapping, loop closure and global correction |
| `state_estimator` | Planned | VIO, wheel odometry and chassis IMU fusion |
| `perception` | Planned | Cones, obstacles and free-space estimation |
| `control` | Planned | Path tracking and command arbitration |
| `autonomy` | Baseline | C++ LiDAR cone detection, local planning and Pure Pursuit |

Generated `build`, `install`, `log`, `generated`, and `results` directories are
not source and must not be committed.

## Build

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Source order

Every new terminal should source ROS and then this workspace:

```zsh
source /opt/ros/humble/setup.zsh
source /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2/install/setup.zsh
```
