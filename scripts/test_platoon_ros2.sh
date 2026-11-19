#!/usr/bin/env bash
# Test and build verification helper for platoon_controller (Ubuntu / ROS 2).
set -euo pipefail

if ! command -v colcon >/dev/null 2>&1; then
  echo "Error: colcon not found. Run this inside Ubuntu with ROS 2 sourced." >&2
  exit 1
fi

echo "==> Building platoon_controller..."
colcon build --symlink-install --packages-select platoon_controller

echo "==> Sourcing install/setup.bash..."
# shellcheck disable=SC1091
set +u
source install/setup.bash
set -u

echo "==> Running colcon test..."
colcon test --packages-select platoon_controller
colcon test-result --verbose

echo ""
echo "==> Build & tests complete. To run Gazebo follower verification:"
echo "  Default namespace:"
echo "    ros2 launch platoon_controller platoon_follower.launch.py use_sim_time:=true"
echo "  Namespaced 2nd robot (e.g. tb3_2):"
echo "    ros2 launch platoon_controller platoon_follower.launch.py namespace:=tb3_2 use_sim_time:=true"
