#!/usr/bin/env bash
set -eo pipefail

cd "$HOME/quad_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "ROS nodes:"
nodes="$(ros2 node list 2>/dev/null || true)"
printf '%s\n' "$nodes"

echo
echo "ROS topics:"
topics="$(ros2 topic list 2>/dev/null || true)"
printf '%s\n' "$topics"

echo
if printf '%s\n' "$nodes" | grep -Eq '(^|/)controller_manager$'; then
  echo "ros2_control controllers:"
  ros2 control list_controllers || true
  echo
  echo "ros2_control hardware interfaces:"
  ros2 control list_hardware_interfaces || true
else
  echo "No /controller_manager node found; skipping ros2_control inspection."
fi

echo
if printf '%s\n' "$topics" | grep -qx '/joint_states'; then
  echo "One /joint_states sample:"
  timeout 5 ros2 topic echo /joint_states --once || true
else
  echo "No /joint_states topic found."
fi
