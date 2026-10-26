#!/usr/bin/env bash
# Run from any directory, independently of the Isaac Sim / Pixi environment.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
task_package="$task_root/assets/USD_Assets/lumos1_rockerless_URDF_description/lumos1_rockerless_URDF_description"
# ROS setup scripts are not safe with nounset enabled.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$task_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$task_root/log/gazebo_site20/ros}"
mkdir -p "$ROS_LOG_DIR"
colcon --log-base "$task_root/log/gazebo_site20" build \
  --base-paths "$task_package" --packages-select lumos1_rockerless_URDF_description \
  --build-base "$task_root/build/gazebo_ros" --install-base "$task_root/install/gazebo_ros" \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
set +u
source "$task_root/install/gazebo_ros/setup.bash"
set -u
exec ros2 launch lumos1_rockerless_URDF_description site20.launch.py \
  "dem:=$task_root/assets/Terrains/SouthPole/Site20_final_adj_5mpp_surf/dem.npy" \
  "output:=$task_root/build/gazebo_site20" "$@"
