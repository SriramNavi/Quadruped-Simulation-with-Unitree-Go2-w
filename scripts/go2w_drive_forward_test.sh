#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/quad_ws/install/setup.bash" ]; then
  source "$HOME/quad_ws/install/setup.bash"
fi

TOPIC="/go2w_wheel_velocity_controller/commands"
MSG_TYPE="std_msgs/msg/Float64MultiArray"
FORWARD="{data: [5.0, 5.0, 5.0, 5.0]}"
STOP="{data: [0.0, 0.0, 0.0, 0.0]}"

ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "$FORWARD"
sleep 2
ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "$STOP"
