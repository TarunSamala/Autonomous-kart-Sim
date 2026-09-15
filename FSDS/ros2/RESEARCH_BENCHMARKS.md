# Autonomy research benchmarks

Single Test exposes four experiment families and five executable profiles
through one preparation, launch, arm, stop, and result contract. All
controllers start disabled. Every result contains the immutable manifest used
for the run.

| Experiment | Perception/localization | Planner/controller | Intended claim |
| --- | --- | --- | --- |
| `amz_style` | LiDAR cone clusters, no prior map | Delaunay beam search + Pure Pursuit | Deployable first-lap local autonomy baseline |
| `behavior_cloning` | RGB-D and speed | Held-out learned policy | End-to-end imitation-learning comparison |
| `slam` | LiDAR odometry + saved SLAM Toolbox pose graph | Recorded route + Pure Pursuit | Sensor-derived mapping/localization track drive |
| `slam_stanley` | Same LiDAR odometry, pose graph and route as `slam` | Recorded route + Stanley | Controlled path-tracker comparison |
| `general_autonomy` | FSDS testing-only track and odometry | Oracle reference + Pure Pursuit | Controller upper-bound/debug baseline only |

The `general_autonomy` manifest sets
`research.uses_privileged_simulation_data: true`. Never report it as a
deployable autonomy result or compare it without that qualification.

## Reproducible sequence

The shortest workflow uses the repository-local `single-test` command. It
enters Distrobox automatically, starts the bridge and experiment stack in one
terminal, waits for readiness, and asks for confirmation immediately before
the vehicle can move.

From the repository root, before launching FSDS:

```zsh
FSDS/ros2/scripts/single-test prepare amz_style
```

Valid values are `amz_style`, `behavior_cloning`, `slam`, `slam_stanley`, and
`general_autonomy`. The command validates the profile, writes a generated
settings file and research manifest under
`FSDS/ros2/generated/single_test/<experiment>/`, and applies the settings to
the repository-local `FSDS/settings.json`. Restart FSDS after preparation.

After FSDS is running:

```zsh
FSDS/ros2/scripts/single-test launch amz_style
```

The launcher starts the bridge when needed, starts the selected stack disabled,
and waits until its services exist. Inspect FSDS/RViz when it prints `READY`,
then press Enter in that same terminal to arm and begin timing. `Ctrl+C`
disarms and cleans up processes owned by the launcher.

Useful checks and manual controls are still available:

```zsh
FSDS/ros2/scripts/single-test check
FSDS/ros2/scripts/single-test arm amz_style
FSDS/ros2/scripts/single-test stop
```

Behavior cloning additionally requires a validated model at
`FSDS/ros2/generated/behavior_cloning/models/rgbd/model.onnx` and
`model.json`. SLAM additionally requires the saved pose graph and route listed
in its manifest. The launch scripts fail before commanding the vehicle when
those artifacts are missing.

## Manual teach, then autonomous repeat

The mapped profiles support a manual first lap. Before starting FSDS, apply
the SLAM sensor contract:

```zsh
FSDS/ros2/scripts/single-test prepare slam
```

Start FSDS, then run one command from the repository root:

```zsh
FSDS/ros2/scripts/single-test teach
```

The command starts or reuses the bridge, starts SLAM Toolbox and the C++ route
recorder, then starts C++ keyboard control. Click the FSDS window, press `E`
to arm, and drive one complete lap with `W/A/S/D`. At lap completion it stops
teleop and serializes the optimized pose graph. Existing map artifacts are
only replaced after typing `REPLACE`, and are archived with a timestamped
backup suffix.

Restart FSDS at the start position and compare both controllers against the
same saved map and route, one at a time:

```zsh
FSDS/ros2/scripts/single-test launch slam
FSDS/ros2/scripts/single-test launch slam_stanley
```

Both remain disabled until the launcher reports `READY` and you confirm
arming.

A previously completed but very slow autonomous SLAM lap proves only that the
pipeline can reach a terminal state. It is not a performance reference. New
mapped-autonomy results must be compared against the current manual lap and
repeated clean autonomous trials using the newly taught map and closed route.

The in-simulator menu can launch C++ keyboard control after the ROS bridge is
running. Select `START MANUAL DRIVE`, close the menu with `F2`, press `E` to
arm, and use `W/A/S/D`; Space is emergency brake and `Q` exits. This manual
button controls the vehicle only. Use `single-test teach` when the drive must
also record the SLAM graph and route.

## Research reporting rules

- Use separate runs; do not average adjacent samples as independent trials.
- Record configuration, model/map checksum, random seed, simulator version,
  track, completion reason, lap time, cone contacts, distance, and speed.
- Run repeated trials for every configuration and report failures, not only
  successful laps.
- Compare methods under the same speed limit and stopping rules.
- Compare `slam` and `slam_stanley` with the exact same map and route hashes.
- Treat simulator-oracle data and testing-only topics as privileged inputs.
- Behavior-cloning train/validation splits must be split by complete run.
- A SLAM mapping run and localization evaluation run must be separate.
