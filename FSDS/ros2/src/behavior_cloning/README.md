# Behavior Cloning

This package provides a reproducible FSDS demonstration-learning pipeline for
the simulated YDLIDAR HP60C RGB-D profile. It supports two models from the same
recordings:

- `rgb`: RGB image plus measured speed.
- `rgbd`: RGB, geometrically aligned metric depth, and measured speed.

The network predicts steering, throttle, and brake. Deployment uses a C++
OpenCV-DNN node with stale-input braking, output/rate limits, launch assist,
cone-contact stop, lap stop, and a disabled-by-default vehicle mode.

Behavior cloning does not guarantee autonomy from one clean lap. Small model
errors move the vehicle into camera views absent from the demonstrations. Use
multiple independent laps, recovery demonstrations, run-level validation, and
shadow-mode comparison before closed-loop testing.

## 1. Build

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to behavior_cloning
source install/setup.zsh
```

## 2. Prepare the HP60C simulation contract

Run this before starting FSDS:

```zsh
scripts/behavior-cloning-prepare
```

It applies a 640x480, 20 Hz RGB plus metric-depth configuration with the HP60C
80.9 degree RGB FOV, 73.8 degree depth FOV, and 0.2-4.0 m working range. FSDS
reproduces the data contract, not the physical projector/noise/lighting
characteristics of the real device.

Launch the packaged simulator from the repository root:

```zsh
./fsds-v2.2.0-linux/FSDS.sh \
  -settings "$PWD/FSDS/settings.json"
```

Then launch the matching bridge:

```zsh
FSDS/ros2/scripts/behavior-cloning-bridge
```

## 3. Collect one demonstration run

Each launch creates a new immutable run directory containing JPEG RGB frames,
16-bit millimetre depth PNGs, calibration metadata, and synchronized labels.

```zsh
FSDS/ros2/scripts/behavior-cloning-collect
```

In the FSDS window, press `E` to arm keyboard control. In another terminal,
start capture immediately before driving:

```zsh
FSDS/ros2/scripts/behavior-cloning-record start
```

Drive a smooth, cone-free lap. Stop capture after stopping the car:

```zsh
FSDS/ros2/scripts/behavior-cloning-record stop
```

Press `E` to disarm teleop, then stop the collection launch with `Ctrl+C`.
Repeat this process for separate runs. Never concatenate laps into one run:
run-level separation is what makes held-out evaluation meaningful.

Recommended initial dataset:

- At least 12 clean full-lap runs; 20-30 is preferable.
- Both smooth centerline laps and deliberate recovery runs.
- Recovery demonstrations beginning 0.5-1.0 m left/right of center with small
  heading errors, corrected smoothly back to the lane.
- Several speed profiles and braking examples.
- No keyboard tapping oscillation, cone contact, reverse driving, or another
  publisher on `/fsds/control_command`.

Data rate is about 10 synchronized samples/s. Keep the validation runs as
entire runs; never randomly split adjacent frames.

## 4. Validate data

```zsh
FSDS/ros2/scripts/behavior-cloning-validate
```

This checks timestamps, label ranges, every RGB/depth file, matching
resolutions, 16-bit depth encoding, and collector write failures.

## 5. Create the training environment

Training uses PyTorch because it is the maintained training/export toolchain.
Vehicle execution remains C++ through OpenCV DNN.

```zsh
distrobox enter kart-ros2-humble
cd /home/satvara/Github/Autonomous-kart-Sim/FSDS/ros2
python3 -m venv --system-site-packages .venv-behavior-cloning
.venv-behavior-cloning/bin/pip install \
  --index-url https://download.pytorch.org/whl/cpu 'torch>=2.2,<3'
.venv-behavior-cloning/bin/pip install \
  -r src/behavior_cloning/training/requirements.txt
```

## 6. Train both comparison models

```zsh
scripts/behavior-cloning-train rgb
scripts/behavior-cloning-train rgbd
```

Each output directory contains:

- `model.pt`: best PyTorch weights.
- `model.onnx`: deployable model.
- `model.json`: immutable preprocessing, split, and validation metrics.

Training refuses a single-run random split. It balances steering/brake samples,
uses mild appearance augmentation, exports ONNX, and verifies that the result
loads through the same OpenCV DNN backend used by the C++ controller.

Offline loss is necessary but not sufficient. Prefer the model with lower
held-out steering error and no systematic corner bias, then validate it in
shadow mode.

## 7. Shadow-mode validation

Start teleop collection as usual, then run the model without connecting it to
the vehicle:

```zsh
scripts/behavior-cloning-shadow \
  generated/behavior_cloning/models/rgbd
```

The expert remains on `/fsds/control_command`; predictions appear only on:

```text
/behavior_cloning/predicted_command
```

Compare the two streams with PlotJuggler or record them in a rosbag. Repeat
poorly predicted turns as recovery demonstrations and retrain. This iterative
expert-correction process is the practical route toward stable closed-loop
behavior.

## 8. Supervised closed-loop test

Stop keyboard teleop and every other control publisher. Start drive mode:

```zsh
scripts/behavior-cloning-drive \
  generated/behavior_cloning/models/rgbd
```

Confirm the node reports `DISABLED`, then verify exactly one publisher:

```zsh
ros2 topic info /fsds/control_command -v
```

Keep the simulator reset control and terminal available. Arm only after fresh
predictions and safety telemetry are visible:

```zsh
scripts/behavior-cloning-arm
```

Monitor:

```zsh
ros2 topic echo /behavior_cloning/inference/status
ros2 topic echo /fsds/testing_only/extra_info
```

First tests should use a reduced speed and active human supervision. A model is
accepted for benchmark use only after multiple complete held-out laps with zero
cone contacts. A single successful lap is not evidence of generalization.

## 9. Timed sub-60-second benchmark

Prepare the immutable 60-second HP60C benchmark contract before starting FSDS:

```zsh
scripts/behavior-cloning-benchmark-prepare
```

Restart FSDS and connect the behavior-cloning bridge. Stop keyboard teleop and
all other controllers, then launch a disabled benchmark at a conservative
throttle limit:

```zsh
scripts/behavior-cloning-benchmark \
  generated/behavior_cloning/models/rgbd 0.20
```

Start the recorder and controller together from a second terminal:

```zsh
scripts/behavior-cloning-benchmark-start
```

The run ends on a complete lap, the first cone contact, or the 60-second time
limit. An emergency/manual stop is available with:

```zsh
scripts/behavior-cloning-benchmark-stop
```

Results are stored under `results/behavior_cloning/<model-directory>/`. Each
JSON result includes duration, FSDS lap time, distance, average/max speed, cone
contacts, the immutable sensor/algorithm manifest, and the complete model
metadata. Increase the throttle limit progressively (`0.20`, `0.25`, `0.30`,
then `0.35`) only after repeatable cone-free laps.

Use at least ten attempts for a comparison. The recommended acceptance target
is a median lap below 60 seconds, zero cone contacts or off-track events, zero
safety interventions, and at least nine completed laps out of ten.
