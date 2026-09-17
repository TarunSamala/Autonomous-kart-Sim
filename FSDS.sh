#!/usr/bin/env zsh

set -e

launcher_path="${0:A}"
launcher_dir="${launcher_path:h}"

# This launcher works both from the repository root and when copied into the
# packaged LinuxNoEditor directory by FSDS/scripts/fsds-package.
if [[ -f "$launcher_dir/FSDS/settings.json" ]]; then
  repository_root="$launcher_dir"
  default_package_root="$repository_root/FSDS/packaged/LinuxNoEditor"
elif [[ -f "$launcher_dir/../../../FSDS/settings.json" ]]; then
  repository_root="${launcher_dir}/../../.."
  repository_root="${repository_root:A}"
  default_package_root="$launcher_dir"
else
  print -u2 "FSDS: cannot locate Autonomous-kart-Sim from $launcher_dir"
  exit 1
fi

fsds_root="$repository_root/FSDS"
package_root="${FSDS_PACKAGE_ROOT:-$default_package_root}"
game_binary="$package_root/FSOnline/Binaries/Linux/Blocks"
settings_file="${FSDS_SETTINGS_PATH:-$fsds_root/settings.json}"

if [[ ! -x "$game_binary" ]]; then
  print -u2 "FSDS: packaged simulator not found: $game_binary"
  print -u2 "Build it once with: FSDS/scripts/fsds-package"
  exit 1
fi
if [[ ! -f "$settings_file" ]]; then
  print -u2 "FSDS: settings file not found: $settings_file"
  exit 1
fi

# The packaged Unreal process inherits these paths. They let every benchmark
# menu button reach the canonical ROS workspace without depending on where the
# packaged binary happens to live.
export FSDS_BRIDGE_START_SCRIPT="$fsds_root/ros2/scripts/bridge-start"
export FSDS_MANUAL_DRIVE_SCRIPT="$fsds_root/ros2/scripts/teleop-start"
export FSDS_SINGLE_TEST_PREPARE_SCRIPT="$fsds_root/ros2/scripts/single-test-prepare"
export FSDS_RVIZ_START_SCRIPT="$fsds_root/ros2/scripts/single-test-visualize"

exec "$game_binary" FSOnline -settings "${settings_file:A}" "$@"
