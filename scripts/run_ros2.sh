#!/usr/bin/env bash
# Run the simulator and keyboard control with the same bundled ROS 2 libraries.
set -euo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"

# A separate domain avoids discovery traffic from the system ROS installation.
task_domain="${OMNILRS_ROS_DOMAIN_ID:-42}"
task_mode="${1:-sim}"
if [[ $# -gt 0 ]]; then shift; fi

case "$task_mode" in
    sim)
        task_args=(python run.py mode=ROS2 "mode.ROS_DOMAIN_ID=$task_domain"
            "++environment.robots_settings.parameters.domain_id=$task_domain")
        ;;
    teleop)
        task_args=(python scripts/teleop_husky.py)
        ;;
    record)
        task_args=(python scripts/record_forward.py)
        ;;
    pv)
        task_args=(python scripts/record_cansat_pv.py)
        ;;
    *)
        echo "Usage: bash scripts/run_ros2.sh [sim|teleop|record|pv] [additional arguments]" >&2
        exit 2
        ;;
esac

exec env -u LD_LIBRARY_PATH -u PYTHONPATH -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    ROS_DOMAIN_ID="$task_domain" \
    pixi run -e ros2 "${task_args[@]}" "$@"
