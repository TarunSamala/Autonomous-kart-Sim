# Autonomy research benchmarks

Single Test exposes four experiment families through one preparation, launch,
arm, stop, and result contract. All controllers start disabled. Every result
contains the immutable manifest used for the run.

| Experiment | Perception/localization | Planner/controller | Intended claim |
| --- | --- | --- | --- |
| `amz_style` | LiDAR cone clusters, no prior map | Delaunay beam search + Pure Pursuit | Deployable first-lap local autonomy baseline |
| `behavior_cloning` | RGB-D and speed | Held-out learned policy | End-to-end imitation-learning comparison |
| `slam` | LiDAR odometry + saved SLAM Toolbox pose graph | Recorded route + Pure Pursuit | Sensor-derived mapping/localization track drive |
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

Valid values are `amz_style`, `behavior_cloning`, `slam`, and
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

## Research reporting rules

- Use separate runs; do not average adjacent samples as independent trials.
- Record configuration, model/map checksum, random seed, simulator version,
  track, completion reason, lap time, cone contacts, distance, and speed.
- Run repeated trials for every configuration and report failures, not only
  successful laps.
- Compare methods under the same speed limit and stopping rules.
- Treat simulator-oracle data and testing-only topics as privileged inputs.
- Behavior-cloning train/validation splits must be split by complete run.
- A SLAM mapping run and localization evaluation run must be separate.
