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
single_test
sensor_interface
teleop
vio
vslam
state_estimator
perception
control
autonomy
```

Implemented baselines currently include Single Test preparation/recording,
sensor adapters, keyboard teleoperation, RGB-D odometry, RTAB-Map VSLAM, and a
C++ LiDAR/Pure-Pursuit autonomy pipeline. Planned packages will be introduced as
their common interfaces are defined. The older `fsds-autonomous-driving`
repository is an archive and is not part of this workspace.
