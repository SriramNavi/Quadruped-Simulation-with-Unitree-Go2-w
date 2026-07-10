#!/usr/bin/env bash
set -eo pipefail

# The cloned Go2 repository does not provide a standalone RViz-only launch.
# Its documented visualization path is the Gazebo simulation launch with RViz enabled.

cd "$HOME/quad_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

has_rviz_arg=false
for arg in "$@"; do
  if [[ "$arg" == rviz:=* ]]; then
    has_rviz_arg=true
    break
  fi
done

if [[ "$has_rviz_arg" == true ]]; then
  exec ros2 launch unitree_go2_sim unitree_go2_launch.py "$@"
else
  exec ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=true "$@"
fi
