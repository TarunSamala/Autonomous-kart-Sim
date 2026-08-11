# Autonomous-kart-Sim
A ROS 2-based simulation, benchmarking, and validation platform for autonomous go-karts, featuring configurable vehicle dynamics, LiDAR, IMU and RGB-D sensors, track generation, SLAM, path planning, perception, control algorithms, fault injection, and automated performance scoring.

## Repository layout

```text
FSDS/                         Simulator source
FSDS/ros2/src/                ROS 2 bridge and project packages
```

The autonomy workspace uses concise ROS package names without a project prefix:

```text
interfaces
sensor_interface
teleop
vio
vslam
state_estimator
perception
control
```

Only `teleop` is migrated so far, under `FSDS/ros2/src/teleop`. The remaining
packages will be introduced there as their interfaces and implementations are defined. The older
`fsds-autonomous-driving` repository is an archive and is not part of this
workspace.
