#!/usr/bin/env bash
set -eo pipefail

cd "$HOME/quad_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 launch quadruped_bringup go2w_reference.launch.py rviz:=false gazebo:=true manual_mode:=true use_reference_controller:=false "$@"
