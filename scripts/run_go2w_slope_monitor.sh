#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RAMP_ANGLE="${1:-15}"
LANE_Y="${2:--6.0}"
TRIAL_ID="${3:-trial_01}"
TARGET_SPEED="${4:-0.25}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  ramp_angle_deg:="${RAMP_ANGLE}" \
  lane_y:="${LANE_Y}" \
  trial_id:="${TRIAL_ID}" \
  target_speed_mps:="${TARGET_SPEED}"
