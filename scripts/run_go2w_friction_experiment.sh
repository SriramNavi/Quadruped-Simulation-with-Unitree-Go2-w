#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRESET="${1:-dry_concrete}"
OUTPUT_DIRECTORY="${2:-}"
TEST_DURATION="${3:-60.0}"
INITIAL_DELAY="${4:-3.0}"
COMMAND_SPEED="${5:-0.25}"
RUN_ID="${6:-run_$(date +%Y%m%d_%H%M%S)}"
GUI="${GUI:-false}"
EXTRA_LAUNCH_ARGS="${EXTRA_LAUNCH_ARGS:-}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if [[ -n "${OUTPUT_DIRECTORY}" && -e "${OUTPUT_DIRECTORY}" ]]; then
  echo "Output directory already exists: ${OUTPUT_DIRECTORY}" >&2
  exit 1
fi

FRICTION_OVERRIDES=()
if [[ -n "${EXTRA_LAUNCH_ARGS}" ]]; then
  read -r -a FRICTION_OVERRIDES <<< "${EXTRA_LAUNCH_ARGS}"
fi

if ! ros2 run quadruped_control go2w_friction_presets show \
  "${PRESET}" >/dev/null; then
  echo "Invalid friction preset: ${PRESET}" >&2
  exit 1
fi

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid ros2 launch quadruped_bringup go2w_reference.launch.py \
  rviz:=false gui:="${GUI}" friction_preset:="${PRESET}" \
  "${FRICTION_OVERRIDES[@]}" &
PIDS+=("$!")

deadline=$((SECONDS + 90))
until ros2 control list_controllers 2>/dev/null \
  | grep -q 'go2w_wheel_velocity_controller.*active'; do
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for active Go2-W wheel controller" >&2
    exit 1
  fi
  sleep 1
done

CONTROLLER_CONFIG="$(ros2 pkg prefix quadruped_control)/share/quadruped_control/config/go2w_drive_controller.yaml"
setsid ros2 run quadruped_control go2w_cmd_vel_to_wheels \
  --ros-args --params-file "${CONTROLLER_CONFIG}" &
PIDS+=("$!")

deadline=$((SECONDS + 30))
until ros2 topic list 2>/dev/null | grep -qx '/joint_states'; do
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for /joint_states" >&2
    exit 1
  fi
  sleep 1
done

deadline=$((SECONDS + 30))
until timeout 2s ros2 topic echo /imu/data --once >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for a valid /imu/data message" >&2
    exit 1
  fi
  sleep 1
done

MONITOR_ARGUMENTS=(
  ramp_angle_deg:=15
  lane_y:=-6.0
  friction_preset:="${PRESET}"
  trial_id:="${RUN_ID}"
  target_speed_mps:="${COMMAND_SPEED}"
  trial_timeout_sec:="${TEST_DURATION}"
  countdown_sec:="${INITIAL_DELAY}"
  auto_drive_enabled:=true
  auto_start:=true
  shutdown_after_result:=true
  "${FRICTION_OVERRIDES[@]}"
)
if [[ -n "${OUTPUT_DIRECTORY}" ]]; then
  MONITOR_ARGUMENTS+=(log_directory:="${OUTPUT_DIRECTORY}")
fi

setsid ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  "${MONITOR_ARGUMENTS[@]}" &
MONITOR_PID="$!"
PIDS+=("${MONITOR_PID}")
wait "${MONITOR_PID}"
