#!/usr/bin/env bash
# Default map: the actual OmniLRS Lunaryard generator, with Lumos + stereo ROS 2.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
task_package="$task_root/assets/USD_Assets/lumos1_rockerless_URDF_description/lumos1_rockerless_URDF_description"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$task_root"
export OMNILRS_ROOT="$task_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$task_root/log/gazebo_lunaryard/ros}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$task_root/build/gazebo_lunaryard/matplotlib}"
mkdir -p "$ROS_LOG_DIR" "$MPLCONFIGDIR"
colcon --log-base "$task_root/log/gazebo_lunaryard" build \
  --base-paths "$task_package" --packages-select lumos1_rockerless_URDF_description \
  --build-base "$task_root/build/gazebo_ros" --install-base "$task_root/install/gazebo_ros" \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
set +u
source "$task_root/install/gazebo_ros/setup.bash"
set -u
exec ros2 launch lumos1_rockerless_URDF_description lunaryard.launch.py "$@"
