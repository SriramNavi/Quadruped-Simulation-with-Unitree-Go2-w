#!/usr/bin/env bash
set -eo pipefail

cd "$HOME/quad_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch quadruped_bringup go2w_teleop.launch.py
