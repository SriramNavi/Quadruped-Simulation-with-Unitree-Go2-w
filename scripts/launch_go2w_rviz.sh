#!/usr/bin/env bash
set -eo pipefail

cd "$HOME/quad_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 launch unitree_go2w_description view_go2w.launch.py rviz:=true gui:=true "$@"
