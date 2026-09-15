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
behavior_cloning
vio
vslam
state_estimator
perception
control
autonomy
```

Implemented baselines currently include Single Test preparation/recording,
sensor adapters, keyboard teleoperation, HP60C RGB/RGB-D behavior-cloning data
and C++ inference, RGB-D odometry, RTAB-Map VSLAM, and a C++
LiDAR autonomy pipeline with Pure Pursuit and Stanley path tracking. Planned packages will be introduced as
their common interfaces are defined. The older `fsds-autonomous-driving`
repository is an archive and is not part of this workspace.

## Canonical FSDS checkout

This repository's `FSDS/` directory is the only FSDS source and settings
checkout used by the project. Do not recreate
`~/Formula-Student-Driverless-Simulator` or link another FSDS clone into the
home directory. The active simulator settings are always:

```text
Autonomous-kart-Sim/FSDS/settings.json
```

Launch the locally built Unreal project with the repository-owned launcher so
AirSim receives that path explicitly:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim
FSDS/scripts/fsds-editor
```

Set `UE4_EDITOR` only if Unreal Engine is not in the sibling
`Github/UnrealEngine` directory. `FSDS_SETTINGS_PATH` may select a generated
test settings file; the simulator launcher and ROS bridge must use the same
value.
