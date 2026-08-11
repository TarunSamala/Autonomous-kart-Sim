# Keyboard teleop

Game-style keyboard control for FSDS through `/fsds/control_command`. The node
polls X11 directly, so held keys and combinations such as `W+A` work even when
the terminal is not focused.

## Controls

| Key | Action |
| --- | --- |
| `E` | Arm/disarm keyboard control |
| `W` / Up | Throttle |
| `S` / Down | Brake |
| `A` / Left | Steer left |
| `D` / Right | Steer right |
| Space | Immediate full brake |
| `Q` / Escape | Full brake and quit |

The node starts disarmed and publishes full brake. Steering, throttle and brake
are ramped to avoid abrupt keyboard commands. FSDS `ControlCommand` has no
reverse-gear or handbrake field, so those controls are not available here.

Do not run this node at the same time as an autonomous controller that also
publishes `/fsds/control_command`; without a command multiplexer, the publishers
will compete.

## Build and run

Build this package from the main ROS 2 workspace:

```zsh
cd /home/satvara/Github/Autonomous-kart-Sim/ros2_ws
source /opt/ros/humble/setup.zsh
source ../FSDS/ros2/install/setup.zsh
colcon build --packages-select teleop --symlink-install
source install/setup.zsh
ros2 run teleop keyboard_teleop
```

If CMake cannot find X11, install its development package inside Distrobox:

```zsh
sudo apt install libx11-dev
```

Example gentler setup:

```zsh
ros2 run teleop keyboard_teleop --ros-args \
  -p max_throttle:=0.6 \
  -p max_steering:=0.8 \
  -p steering_rate:=1.8
```

Available parameters are `control_topic`, `publish_rate`, `max_throttle`,
`max_brake`, `max_steering`, `throttle_rise_rate`, `throttle_fall_rate`,
`brake_rise_rate`, `brake_fall_rate`, `steering_rate`, and
`steering_return_rate`.
