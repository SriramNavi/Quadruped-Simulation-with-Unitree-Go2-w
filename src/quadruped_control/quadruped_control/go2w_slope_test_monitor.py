"""Quantitative passive and automated Go2-W slope trial monitoring."""

from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

from geometry_msgs.msg import PoseArray, Twist, Vector3Stamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Imu, JointState

from std_msgs.msg import Bool, Float64MultiArray, String


NAN = float('nan')

JOINT_NAMES = (
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'FL_foot_joint',
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'FR_foot_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
    'RL_foot_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
    'RR_foot_joint',
)


@dataclass(frozen=True)
class RampGeometry:
    """Exact geometry read from go2w_slope_test.sdf."""

    lane_y: float
    ramp_center_x: float
    ramp_center_z: float
    ramp_pitch_rad: float
    ramp_length: float
    ramp_width: float
    ramp_thickness: float
    ramp_entry_x: float
    platform_center_x: float
    platform_center_z: float
    platform_length: float
    platform_width: float
    platform_thickness: float
    platform_entry_x: float
    platform_top_height: float
    success_margin: float = 0.25

    @property
    def success_min_x(self) -> float:
        """Return the inner platform start, away from the ramp-facing edge."""
        return self.platform_entry_x + self.success_margin

    @property
    def success_max_x(self) -> float:
        """Return the inner platform end, away from the open far edge."""
        platform_end = self.platform_center_x + self.platform_length / 2.0
        return platform_end - self.success_margin

    @property
    def platform_end_x(self) -> float:
        """Return the open far edge of the platform."""
        return self.platform_center_x + self.platform_length / 2.0


# Values below are copied from the generated SDF, not inferred from nominal angles.
RAMP_GEOMETRY = {
    15: RampGeometry(
        lane_y=-6.0,
        ramp_center_x=3.9303437084,
        ramp_center_z=0.5890920632,
        ramp_pitch_rad=-0.2617993878,
        ramp_length=5.0,
        ramp_width=2.5,
        ramp_thickness=0.12,
        ramp_entry_x=1.5,
        platform_center_x=7.3096291314,
        platform_center_z=1.2340952255,
        platform_length=2.0,
        platform_width=2.5,
        platform_thickness=0.12,
        platform_entry_x=6.3096291314,
        platform_top_height=1.2940952255,
    ),
    30: RampGeometry(
        lane_y=-2.0,
        ramp_center_x=3.6950635095,
        ramp_center_z=1.1980384758,
        ramp_pitch_rad=-0.5235987756,
        ramp_length=5.0,
        ramp_width=2.5,
        ramp_thickness=0.12,
        ramp_entry_x=1.5,
        platform_center_x=6.8101270189,
        platform_center_z=2.4400000000,
        platform_length=2.0,
        platform_width=2.5,
        platform_thickness=0.12,
        platform_entry_x=5.8101270189,
        platform_top_height=2.5000000000,
    ),
    35: RampGeometry(
        lane_y=2.0,
        ramp_center_x=3.5822946969,
        ramp_center_z=1.3847919682,
        ramp_pitch_rad=-0.6108652382,
        ramp_length=5.0,
        ramp_width=2.5,
        ramp_thickness=0.12,
        ramp_entry_x=1.5,
        platform_center_x=6.5757602214,
        platform_center_z=2.8078821818,
        platform_length=2.0,
        platform_width=2.5,
        platform_thickness=0.12,
        platform_entry_x=5.5757602214,
        platform_top_height=2.8678821818,
    ),
    45: RampGeometry(
        lane_y=6.0,
        ramp_center_x=3.3101933598,
        ramp_center_z=1.7253405461,
        ramp_pitch_rad=-0.7853981634,
        ramp_length=5.0,
        ramp_width=2.5,
        ramp_thickness=0.12,
        ramp_entry_x=1.5,
        platform_center_x=6.0155339059,
        platform_center_z=3.4755339059,
        platform_length=2.0,
        platform_width=2.5,
        platform_thickness=0.12,
        platform_entry_x=5.0155339059,
        platform_top_height=3.5355339059,
    ),
}

TERMINAL_STATES = {'TOPPLED', 'SUCCESS', 'TIMEOUT', 'ABORTED'}
AUTOMATION_PHASES = {
    'INITIALIZING', 'PRECHECK', 'COUNTDOWN', 'ACCELERATING', 'CRUISING',
    'DECELERATING', 'STOPPING', 'COMPLETE',
}


@dataclass(frozen=True)
class PrecheckConfig:
    """Configuration used by the pure automated-start validation helper."""

    expected_x_m: float
    expected_y_m: float
    expected_yaw_deg: float
    position_tolerance_m: float
    yaw_tolerance_deg: float
    stationary_speed_threshold_mps: float
    upright_roll_tolerance_deg: float
    upright_pitch_tolerance_deg: float
    require_cmd_vel_subscriber: bool = True
    require_start_pose: bool = True
    require_upright_start: bool = True
    require_stationary_start: bool = True


@dataclass(frozen=True)
class PrecheckSample:
    """One robot-state sample used for automated-start validation."""

    odometry_valid: bool
    joint_states_valid: bool
    odometry_fresh: bool
    joint_states_fresh: bool
    cmd_vel_subscriber_count: int
    emergency_stop_active: bool
    base_x_m: float = NAN
    base_y_m: float = NAN
    yaw_deg: float = NAN
    roll_deg: float = NAN
    pitch_deg: float = NAN
    planar_speed_mps: float = NAN


def finite_values(*values: float) -> bool:
    """Return true when every value can be interpreted as finite."""
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def ros_timeout_elapsed(
    start_time_sec: float | None,
    now_sec: float,
) -> tuple[float | None, float]:
    """Start and measure a timeout only while ROS time is positive and monotonic."""
    if not math.isfinite(now_sec) or now_sec <= 0.0:
        return start_time_sec, 0.0
    if start_time_sec is None or now_sec < start_time_sec:
        return now_sec, 0.0
    return start_time_sec, now_sec - start_time_sec


def pose_matches_expected_spawn(
    position: Sequence[float],
    expected_x_m: float,
    expected_y_m: float,
    tolerance_m: float,
) -> bool:
    """Validate an indexed Gazebo pose before accepting it as the robot model."""
    return (
        finite_values(*position, expected_x_m, expected_y_m, tolerance_m)
        and tolerance_m >= 0.0
        and math.hypot(
            float(position[0]) - expected_x_m,
            float(position[1]) - expected_y_m,
        ) <= tolerance_m
    )


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def angular_error_deg(actual_deg: float, expected_deg: float) -> float:
    """Return the signed shortest angular error in degrees."""
    if not finite_values(actual_deg, expected_deg):
        return NAN
    return math.degrees(normalize_angle(math.radians(actual_deg - expected_deg)))


def rate_limited_speed(
    current_mps: float,
    target_mps: float,
    rate_mps2: float,
    elapsed_sec: float,
) -> float:
    """Move a scalar speed toward a target without exceeding rate * elapsed time."""
    if not finite_values(current_mps, target_mps, rate_mps2, elapsed_sec):
        raise ValueError('velocity profile inputs must be finite')
    if rate_mps2 <= 0.0:
        raise ValueError('velocity profile rate must be greater than zero')
    if elapsed_sec <= 0.0:
        return current_mps
    maximum_change = rate_mps2 * elapsed_sec
    difference = target_mps - current_mps
    if abs(difference) <= maximum_change:
        return target_mps
    return current_mps + math.copysign(maximum_change, difference)


def calculate_deceleration_start_x(
    geometry: RampGeometry,
    target_speed_mps: float,
    deceleration_mps2: float,
    stop_distance_before_platform_end_m: float,
) -> float:
    """Return the earlier of platform entry and the braking-distance limit."""
    if deceleration_mps2 <= 0.0:
        raise ValueError('deceleration_mps2 must be greater than zero')
    stopping_distance = target_speed_mps ** 2 / (2.0 * deceleration_mps2)
    latest_start = (
        geometry.platform_end_x
        - stop_distance_before_platform_end_m
        - stopping_distance
    )
    return min(geometry.success_min_x, latest_start)


def precheck_blockers(
    sample: PrecheckSample,
    config: PrecheckConfig,
) -> tuple[str, ...]:
    """Return every independently active automated-start blocker."""
    blockers = []
    if not sample.odometry_valid:
        blockers.append('WAITING_FOR_ODOMETRY')
    elif not sample.odometry_fresh:
        blockers.append('ODOMETRY_STALE')
    if not sample.joint_states_valid:
        blockers.append('WAITING_FOR_JOINT_STATES')
    elif not sample.joint_states_fresh:
        blockers.append('JOINT_STATES_STALE')
    if sample.emergency_stop_active:
        blockers.append('EMERGENCY_STOP_ACTIVE')
    if config.require_cmd_vel_subscriber and sample.cmd_vel_subscriber_count < 1:
        blockers.append('CMD_VEL_NO_SUBSCRIBER')
    odometry_ready = sample.odometry_valid and sample.odometry_fresh
    if odometry_ready and config.require_start_pose:
        if (
            not finite_values(sample.base_x_m, sample.base_y_m)
            or abs(sample.base_x_m - config.expected_x_m) > config.position_tolerance_m
            or abs(sample.base_y_m - config.expected_y_m) > config.position_tolerance_m
        ):
            blockers.append('START_POSITION_INVALID')
        elif (
            not math.isfinite(
                angular_error_deg(sample.yaw_deg, config.expected_yaw_deg)
            )
            or abs(angular_error_deg(sample.yaw_deg, config.expected_yaw_deg))
            > config.yaw_tolerance_deg
        ):
            blockers.append('START_HEADING_INVALID')
    if odometry_ready and config.require_upright_start and (
        not finite_values(sample.roll_deg, sample.pitch_deg)
        or abs(sample.roll_deg) > config.upright_roll_tolerance_deg
        or abs(sample.pitch_deg) > config.upright_pitch_tolerance_deg
    ):
        blockers.append('ROBOT_NOT_UPRIGHT')
    if odometry_ready and config.require_stationary_start and (
        not math.isfinite(sample.planar_speed_mps)
        or sample.planar_speed_mps > config.stationary_speed_threshold_mps
    ):
        blockers.append('ROBOT_NOT_STATIONARY')
    return tuple(blockers)


def evaluate_precheck(
    sample: PrecheckSample,
    config: PrecheckConfig,
) -> tuple[bool, str]:
    """Evaluate automated-start conditions in deterministic safety order."""
    blockers = precheck_blockers(sample, config)
    return not blockers, blockers[0] if blockers else ''


def automation_safety_abort_reason(
    *,
    odometry_fresh: bool,
    joint_states_fresh: bool,
    cmd_vel_subscriber_available: bool,
    require_cmd_vel_subscriber: bool,
    lane_error_m: float,
    heading_error_deg: float,
    max_lane_error_m: float,
    max_heading_error_deg: float,
    emergency_stop_active: bool = False,
) -> str:
    """Return the first active runtime automation safety violation."""
    if emergency_stop_active:
        return 'EMERGENCY_STOP'
    if not odometry_fresh:
        return 'ODOMETRY_TIMEOUT'
    if not joint_states_fresh:
        return 'JOINT_STATES_TIMEOUT'
    if require_cmd_vel_subscriber and not cmd_vel_subscriber_available:
        return 'CMD_VEL_NO_SUBSCRIBER'
    if not math.isfinite(lane_error_m) or abs(lane_error_m) > max_lane_error_m:
        return 'LANE_DEVIATION'
    if (
        not math.isfinite(heading_error_deg)
        or abs(heading_error_deg) > max_heading_error_deg
    ):
        return 'HEADING_DEVIATION'
    return ''


def command_speed_for_state(
    requested_speed_mps: float,
    result_state: str,
    topple_condition_active: bool = False,
) -> float:
    """Apply the invariant that terminal or toppling states command zero."""
    if topple_condition_active or result_state in TERMINAL_STATES:
        return 0.0
    return max(0.0, float(requested_speed_mps))


def motion_command_publication_enabled(auto_drive_enabled: bool) -> bool:
    """Return whether this monitor instance may publish Twist commands."""
    return bool(auto_drive_enabled)


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    """Normalize a finite quaternion, rejecting zero-norm input."""
    if not finite_values(x, y, z, w):
        raise ValueError('quaternion contains a non-finite component')
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm is zero')
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float]:
    """Convert a quaternion to normalized roll, pitch, and yaw."""
    x, y, z, w = normalize_quaternion(x, y, z, w)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    )
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)


def body_up_vector_vertical_component(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """Return the world-Z component of the body's unit up vector."""
    x, y, z, w = normalize_quaternion(x, y, z, w)
    del z, w
    return 1.0 - 2.0 * (x * x + y * y)


def safe_sequence_value(values: Sequence[float], index: int) -> float:
    """Read a sequence value as a finite float or return NaN."""
    if index >= len(values):
        return NAN
    try:
        value = float(values[index])
    except (TypeError, ValueError):
        return NAN
    return value if math.isfinite(value) else NAN


def extract_joint_values(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    efforts: Sequence[float],
    required_names: Iterable[str] = JOINT_NAMES,
) -> dict[str, tuple[float, float, float]]:
    """Extract named joint position, velocity, and effort without order assumptions."""
    indices = {name: index for index, name in enumerate(names)}
    extracted = {}
    for name in required_names:
        index = indices.get(name)
        if index is None:
            extracted[name] = (NAN, NAN, NAN)
            continue
        extracted[name] = (
            safe_sequence_value(positions, index),
            safe_sequence_value(velocities, index),
            safe_sequence_value(efforts, index),
        )
    return extracted


def calculate_rate(current: float, previous: float, elapsed_sec: float) -> float:
    """Calculate a rate from the real elapsed simulation time."""
    if not finite_values(current, previous, elapsed_sec) or elapsed_sec <= 0.0:
        return NAN
    return (current - previous) / elapsed_sec


def quaternion_multiply(
    first: Sequence[float],
    second: Sequence[float],
) -> tuple[float, float, float, float]:
    """Multiply quaternions in x, y, z, w order."""
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_world_vector_to_body(
    vector: Sequence[float],
    orientation: Sequence[float],
) -> tuple[float, float, float]:
    """Rotate a world-frame vector into the body frame."""
    x, y, z, w = normalize_quaternion(*orientation)
    conjugate = (-x, -y, -z, w)
    pure = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    result = quaternion_multiply(quaternion_multiply(conjugate, pure), (x, y, z, w))
    return result[0], result[1], result[2]


def pose_rates(
    position: Sequence[float],
    orientation: Sequence[float],
    previous_position: Sequence[float],
    previous_orientation: Sequence[float],
    elapsed_sec: float,
) -> tuple[float, float, float, float, float, float]:
    """Derive body-frame linear and angular rates from two world poses."""
    if elapsed_sec <= 0.0 or not finite_values(elapsed_sec):
        return (NAN,) * 6
    world_velocity = tuple(
        calculate_rate(position[index], previous_position[index], elapsed_sec)
        for index in range(3)
    )
    linear = rotate_world_vector_to_body(world_velocity, orientation)
    previous = normalize_quaternion(*previous_orientation)
    current = normalize_quaternion(*orientation)
    delta = quaternion_multiply(
        (-previous[0], -previous[1], -previous[2], previous[3]),
        current,
    )
    delta = normalize_quaternion(*delta)
    if delta[3] < 0.0:
        delta = tuple(-value for value in delta)
    vector_norm = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
    if vector_norm <= 1.0e-12:
        angular = (0.0, 0.0, 0.0)
    else:
        angle = 2.0 * math.atan2(vector_norm, max(delta[3], 0.0))
        scale = angle / (vector_norm * elapsed_sec)
        angular = (delta[0] * scale, delta[1] * scale, delta[2] * scale)
    return linear + angular


def warning_threshold_active(
    roll_deg: float,
    pitch_deg: float,
    warning_roll_deg: float,
    warning_pitch_deg: float,
) -> bool:
    """Return true if either warning orientation threshold is met."""
    return (
        finite_values(roll_deg, pitch_deg)
        and (
            abs(roll_deg) >= warning_roll_deg
            or abs(pitch_deg) >= warning_pitch_deg
        )
    )


def active_topple_conditions(
    roll_deg: float,
    pitch_deg: float,
    base_z: float,
    body_up_alignment: float,
    topple_roll_deg: float,
    topple_pitch_deg: float,
    minimum_base_height_m: float,
    minimum_body_up_alignment: float,
) -> tuple[str, ...]:
    """Return active, independently measured topple conditions."""
    conditions = []
    if math.isfinite(roll_deg) and abs(roll_deg) >= topple_roll_deg:
        conditions.append('ROLL_LIMIT')
    if math.isfinite(pitch_deg) and abs(pitch_deg) >= topple_pitch_deg:
        conditions.append('PITCH_LIMIT')
    if math.isfinite(base_z) and base_z <= minimum_base_height_m:
        conditions.append('LOW_BASE')
    if math.isfinite(body_up_alignment) and body_up_alignment < minimum_body_up_alignment:
        conditions.append('BODY_INVERTED')
    return tuple(conditions)


class TrialStateMachine:
    """Deterministic slope-trial state and dwell-time logic."""

    def __init__(
        self,
        geometry: RampGeometry,
        lane_y: float,
        warning_roll_deg: float = 30.0,
        warning_pitch_deg: float = 45.0,
        topple_roll_deg: float = 55.0,
        topple_pitch_deg: float = 65.0,
        minimum_base_height_m: float = 0.12,
        minimum_body_up_alignment: float = 0.35,
        topple_hold_sec: float = 0.25,
        success_hold_sec: float = 1.0,
        lane_tolerance_m: float = 1.0,
        slide_back_distance_m: float = 0.75,
        trial_timeout_sec: float = 60.0,
    ) -> None:
        """Configure the trial geometry, thresholds, and state memory."""
        self.geometry = geometry
        self.lane_y = lane_y
        self.warning_roll_deg = warning_roll_deg
        self.warning_pitch_deg = warning_pitch_deg
        self.topple_roll_deg = topple_roll_deg
        self.topple_pitch_deg = topple_pitch_deg
        self.minimum_base_height_m = minimum_base_height_m
        self.minimum_body_up_alignment = minimum_body_up_alignment
        self.topple_hold_sec = topple_hold_sec
        self.success_hold_sec = success_hold_sec
        self.lane_tolerance_m = lane_tolerance_m
        self.slide_back_distance_m = slide_back_distance_m
        self.trial_timeout_sec = trial_timeout_sec

        self.state = 'WAITING_FOR_DATA'
        self.start_time: float | None = None
        self.result_time: float | None = None
        self.topple_since: float | None = None
        self.success_since: float | None = None
        self.time_to_platform_sec: float | None = None
        self.topple_trigger = ''
        self.warning_active = False
        self.topple_condition_active = False
        self.success_region_active = False
        self.maximum_forward_x = -math.inf
        self.entered_ramp = False
        self.slide_back_detected = False

    def elapsed(self, now_sec: float) -> float:
        """Return trial elapsed time, excluding the pre-start ready period."""
        if self.start_time is None:
            return 0.0
        end = self.result_time if self.result_time is not None else now_sec
        return max(0.0, end - self.start_time)

    def abort(self, now_sec: float) -> None:
        """End an unfinished trial as aborted."""
        if self.state not in TERMINAL_STATES:
            self.state = 'ABORTED'
            self.result_time = now_sec

    def start(self, now_sec: float) -> None:
        """Start an explicitly armed automated trial."""
        if self.state in {'WAITING_FOR_DATA', 'READY'}:
            self.start_time = now_sec
            self.state = 'RUNNING'

    def step(
        self,
        now_sec: float,
        data_ready: bool,
        moving_toward_ramp: bool,
        roll_deg: float = NAN,
        pitch_deg: float = NAN,
        base_z: float = NAN,
        body_up_alignment: float = NAN,
        base_x: float = NAN,
        base_y: float = NAN,
        emergency_stop: bool = False,
        planar_speed_mps: float = NAN,
        require_stationary_success: bool = False,
        stationary_speed_threshold_mps: float = 0.03,
    ) -> str:
        """Advance the state machine using one timestamped sample."""
        if self.state in TERMINAL_STATES:
            return self.state
        if not data_ready:
            if self.start_time is None:
                self.state = 'WAITING_FOR_DATA'
            return self.state
        if self.state == 'WAITING_FOR_DATA':
            self.state = 'READY'
        if self.state == 'READY':
            if not moving_toward_ramp:
                return self.state
            self.start_time = now_sec
            self.state = 'RUNNING'

        if emergency_stop:
            self.abort(now_sec)
            return self.state

        if math.isfinite(base_x):
            self.maximum_forward_x = max(self.maximum_forward_x, base_x)
            if base_x >= self.geometry.ramp_entry_x:
                self.entered_ramp = True
            if (
                self.entered_ramp
                and self.maximum_forward_x - base_x > self.slide_back_distance_m
            ):
                self.slide_back_detected = True

        conditions = active_topple_conditions(
            roll_deg,
            pitch_deg,
            base_z,
            body_up_alignment,
            self.topple_roll_deg,
            self.topple_pitch_deg,
            self.minimum_base_height_m,
            self.minimum_body_up_alignment,
        )
        self.topple_condition_active = bool(conditions)
        self.warning_active = warning_threshold_active(
            roll_deg,
            pitch_deg,
            self.warning_roll_deg,
            self.warning_pitch_deg,
        )
        self.success_region_active = (
            not conditions
            and finite_values(base_x, base_y, base_z, roll_deg, pitch_deg)
            and self.geometry.success_min_x <= base_x <= self.geometry.success_max_x
            and abs(base_y - self.lane_y) <= self.lane_tolerance_m
            and base_z >= self.geometry.platform_top_height
            and abs(roll_deg) < self.topple_roll_deg
            and abs(pitch_deg) < self.topple_pitch_deg
        )
        if (
            self.success_region_active
            and self.start_time is not None
            and self.time_to_platform_sec is None
        ):
            self.time_to_platform_sec = max(0.0, now_sec - self.start_time)

        if conditions:
            self.success_since = None
            if self.topple_since is None:
                self.topple_since = now_sec
            if now_sec - self.topple_since >= self.topple_hold_sec:
                self.state = 'TOPPLED'
                self.result_time = now_sec
                self.topple_trigger = conditions[0] if len(conditions) == 1 else 'MULTIPLE'
                return self.state
            self.state = 'TOPPLING'
        else:
            self.topple_since = None
            success_ready = (
                self.success_region_active
                and (
                    not require_stationary_success
                    or (
                        math.isfinite(planar_speed_mps)
                        and planar_speed_mps <= stationary_speed_threshold_mps
                    )
                )
            )
            if success_ready:
                if self.success_since is None:
                    self.success_since = now_sec
                if now_sec - self.success_since >= self.success_hold_sec:
                    self.state = 'SUCCESS'
                    self.result_time = now_sec
                    return self.state
            else:
                self.success_since = None
            self.state = 'WARNING' if self.warning_active else 'RUNNING'

        if self.start_time is not None and now_sec - self.start_time >= self.trial_timeout_sec:
            self.state = 'TIMEOUT'
            self.result_time = now_sec
        return self.state


class GroundTruthOdometryAdapter(Node):
    """Convert Gazebo's timestamped model PoseArray into ground-truth odometry."""

    def __init__(self) -> None:
        """Configure the source pose selector and normalized odometry output."""
        super().__init__('go2w_ground_truth_adapter')
        self.declare_parameter('pose_array_topic', '/go2w/ground_truth/pose_array')
        self.declare_parameter('odometry_topic', '/go2w/ground_truth/odom')
        self.declare_parameter('model_pose_index', 0)
        self.declare_parameter('expected_spawn_x_m', -3.0)
        self.declare_parameter('expected_spawn_y_m', -6.0)
        self.declare_parameter('spawn_position_tolerance_m', 0.75)
        pose_topic = self.get_parameter('pose_array_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        self.pose_index = int(self.get_parameter('model_pose_index').value)
        self.expected_spawn_x_m = float(
            self.get_parameter('expected_spawn_x_m').value
        )
        self.expected_spawn_y_m = float(
            self.get_parameter('expected_spawn_y_m').value
        )
        self.spawn_position_tolerance_m = float(
            self.get_parameter('spawn_position_tolerance_m').value
        )
        if self.pose_index < 0:
            raise ValueError('model_pose_index must be non-negative')
        if self.spawn_position_tolerance_m < 0.0:
            raise ValueError('spawn_position_tolerance_m must not be negative')
        self.publisher = self.create_publisher(Odometry, odometry_topic, 10)
        self.subscription = self.create_subscription(
            PoseArray,
            pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.previous_position: tuple[float, float, float] | None = None
        self.previous_orientation: tuple[float, float, float, float] | None = None
        self.previous_time_sec: float | None = None
        self.pose_selection_validated = False
        self.get_logger().info(
            f'Ground truth adapter: {pose_topic}[{self.pose_index}] -> {odometry_topic}; '
            f'expected spawn=({self.expected_spawn_x_m:.3f}, '
            f'{self.expected_spawn_y_m:.3f})'
        )

    def _pose_callback(self, message: PoseArray) -> None:
        if self.pose_index >= len(message.poses):
            self.get_logger().error(
                f'PoseArray contains {len(message.poses)} poses; '
                f'index {self.pose_index} is unavailable',
                throttle_duration_sec=5.0,
            )
            return
        pose = message.poses[self.pose_index]
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        try:
            orientation = normalize_quaternion(*orientation)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        if not finite_values(*position):
            self.get_logger().warning(
                'Ground-truth position is invalid', throttle_duration_sec=5.0,
            )
            return
        if not self.pose_selection_validated:
            if not pose_matches_expected_spawn(
                position,
                self.expected_spawn_x_m,
                self.expected_spawn_y_m,
                self.spawn_position_tolerance_m,
            ):
                self.get_logger().error(
                    f'PoseArray index {self.pose_index} is at '
                    f'({position[0]:.3f}, {position[1]:.3f}), not near the '
                    f'configured spawn ({self.expected_spawn_x_m:.3f}, '
                    f'{self.expected_spawn_y_m:.3f}); odometry suppressed',
                    throttle_duration_sec=5.0,
                )
                return
            self.pose_selection_validated = True
            self.get_logger().info(
                f'Validated PoseArray index {self.pose_index} at the robot spawn'
            )
        time_sec = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        if (
            self.previous_position is None
            or self.previous_orientation is None
            or self.previous_time_sec is None
            or time_sec <= self.previous_time_sec
        ):
            self.previous_position = position
            self.previous_orientation = orientation
            self.previous_time_sec = time_sec
            return
        rates = pose_rates(
            position,
            orientation,
            self.previous_position,
            self.previous_orientation,
            time_sec - self.previous_time_sec,
        )
        if not finite_values(*rates):
            self.get_logger().warning(
                'Ground-truth pose rates are invalid; odometry suppressed',
                throttle_duration_sec=5.0,
            )
            return
        odometry = Odometry()
        odometry.header = message.header
        odometry.header.frame_id = 'world'
        odometry.child_frame_id = 'base'
        odometry.pose.pose = pose
        odometry.twist.twist.linear.x = rates[0]
        odometry.twist.twist.linear.y = rates[1]
        odometry.twist.twist.linear.z = rates[2]
        odometry.twist.twist.angular.x = rates[3]
        odometry.twist.twist.angular.y = rates[4]
        odometry.twist.twist.angular.z = rates[5]
        self.publisher.publish(odometry)
        self.previous_position = position
        self.previous_orientation = orientation
        self.previous_time_sec = time_sec


def csv_columns() -> list[str]:
    """Return the stable per-trial CSV schema."""
    columns = [
        'sim_time_sec', 'wall_time_iso', 'trial_id', 'ramp_angle_deg',
        'target_speed_mps', 'state', 'base_x_m', 'base_y_m', 'base_z_m',
        'roll_deg', 'pitch_deg', 'yaw_deg', 'body_up_alignment',
        'linear_vx_mps', 'linear_vy_mps', 'linear_vz_mps',
        'angular_vx_radps', 'angular_vy_radps', 'angular_vz_radps',
        'planar_speed_mps', 'cmd_linear_x_mps', 'cmd_angular_z_radps',
        'wheel_cmd_fl_radps', 'wheel_cmd_fr_radps', 'wheel_cmd_rl_radps',
        'wheel_cmd_rr_radps', 'emergency_stop', 'imu_roll_deg',
        'imu_pitch_deg', 'imu_yaw_deg', 'imu_angular_x_radps',
        'imu_angular_y_radps', 'imu_angular_z_radps', 'imu_accel_x_mps2',
        'imu_accel_y_mps2', 'imu_accel_z_mps2',
    ]
    for name in JOINT_NAMES:
        columns.extend((
            f'{name}_position_rad',
            f'{name}_velocity_radps',
            f'{name}_effort_nm',
        ))
    columns.extend((
        'warning_active',
        'topple_condition_active',
        'slide_back_detected',
        'success_region_active',
        'topple_trigger',
        'auto_drive_enabled',
        'automation_phase',
        'precheck_passed',
        'precheck_failure_reason',
        'countdown_remaining_sec',
        'generated_cmd_linear_x_mps',
        'generated_cmd_angular_z_radps',
        'acceleration_mps2',
        'deceleration_mps2',
        'odometry_fresh',
        'joint_states_fresh',
        'cmd_vel_subscriber_count',
        'lane_error_m',
        'heading_error_deg',
        'start_pose_valid',
        'upright_start_valid',
        'stationary_start_valid',
        'abort_reason',
        'deceleration_start_x_m',
        'platform_success_start_x_m',
        'platform_success_end_x_m',
    ))
    return columns


class Go2WSlopeTestMonitor(Node):
    """Read state and command telemetry, classify a trial, and write CSV/JSON."""

    def __init__(self) -> None:
        """Load parameters, open the trial log, and connect ROS interfaces."""
        super().__init__('go2w_slope_test_monitor')
        self._declare_parameters()
        self._load_parameters()
        self._create_log()

        self.odometry: Odometry | None = None
        self.joints = {name: (NAN, NAN, NAN) for name in JOINT_NAMES}
        self.cmd_linear = NAN
        self.cmd_angular = NAN
        self.wheel_commands = [NAN] * 4
        self.emergency_stop: bool | None = None
        self.imu_values = [NAN] * 9
        self.odometry_valid = False
        self.joints_valid = False
        self.last_odometry_receive_sec: float | None = None
        self.last_joint_states_receive_sec: float | None = None
        self.initial_base_x: float | None = None
        self.seen_cmd = False
        self.seen_wheel_commands = False
        self.seen_emergency_stop = False
        self.seen_imu = False
        self.seen_odometry_twist = False
        self.seen_joint_effort = {name: False for name in JOINT_NAMES}

        self.maximum_roll_deg = NAN
        self.maximum_pitch_deg = NAN
        self.minimum_base_height_m = NAN
        self.maximum_commanded_speed_mps = NAN
        self.maximum_measured_speed_mps = NAN
        self.maximum_generated_command_mps = NAN
        self.cruise_command_sum_mps = 0.0
        self.cruise_command_count = 0
        self.current_commanded_speed_mps = 0.0
        self.target_commanded_speed_mps = self.target_speed_mps
        self.generated_cmd_linear_x_mps = 0.0 if self.auto_drive_enabled else NAN
        self.generated_cmd_angular_z_radps = 0.0 if self.auto_drive_enabled else NAN
        self.automation_phase = (
            'INITIALIZING' if self.auto_drive_enabled else 'PASSIVE'
        )
        self.automation_phase_at_result = ''
        self.precheck_passed = False
        self.precheck_failure_reason = ''
        self.precheck_active_blockers: tuple[str, ...] = ()
        self.last_precheck_blockers: tuple[str, ...] = ()
        self.countdown_remaining_sec = NAN
        self.countdown_start_time: float | None = None
        self.last_countdown_value: int | None = None
        self.actual_start_pose: dict[str, float] | None = None
        self.command_subscriber_count_at_start: int | None = None
        self.abort_reason = ''
        self.state_data_timeouts: list[str] = []
        self.runner_estop_asserted = False
        self.automation_halted_for_topple = False
        self.stop_start_time: float | None = None
        self.cmd_subscriber_missing_since: float | None = None
        self.last_command_update_time: float | None = None
        self.precheck_start_time: float | None = None
        self.start_pose_valid = False
        self.upright_start_valid = False
        self.stationary_start_valid = False
        self.last_display_time = -math.inf
        self.last_display_state = ''
        self.rows_since_flush = 0
        self.closed = False

        self._create_ros_interfaces()
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        self.last_command_update_time = now_sec if now_sec > 0.0 else None
        if self.auto_drive_enabled:
            self._publish_zero()
            self.automation_timer = self.create_timer(
                1.0 / self.command_publish_rate_hz,
                self._automation_tick,
            )
        else:
            self.automation_timer = None
        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self._sample)
        self.get_logger().info(
            f'[{self.trial_id}] monitor ready; '
            f'auto_drive={self.auto_drive_enabled}; CSV: {self.csv_path}'
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'joint_states_topic': '/joint_states',
            'cmd_vel_topic': '/cmd_vel',
            'wheel_command_topic': '/go2w_wheel_velocity_controller/commands',
            'emergency_stop_topic': '/go2w/emergency_stop',
            'odometry_topic': '/go2w/ground_truth/odom',
            'imu_topic': '/imu/data',
            'ramp_angle_deg': 15.0,
            'lane_y': NAN,
            'trial_id': 'trial_01',
            'target_speed_mps': 0.25,
            'operator_note': '',
            'log_directory': '~/quad_ws/logs/slope_tests',
            'sample_rate_hz': 20.0,
            'trial_timeout_sec': 60.0,
            'result_hold_sec': 2.0,
            'auto_stop_logging_on_result': True,
            'warning_roll_deg': 30.0,
            'warning_pitch_deg': 45.0,
            'topple_roll_deg': 55.0,
            'topple_pitch_deg': 65.0,
            'minimum_base_height_m': 0.12,
            'minimum_body_up_alignment': 0.35,
            'topple_hold_sec': 0.25,
            'success_hold_sec': 1.0,
            'lane_tolerance_m': 1.0,
            'slide_back_distance_m': 0.75,
            'auto_drive_enabled': False,
            'auto_start': True,
            'countdown_sec': 3.0,
            'shutdown_after_result': False,
            'stop_hold_sec': 1.0,
            'require_cmd_vel_subscriber': True,
            'require_start_pose': True,
            'require_upright_start': True,
            'require_stationary_start': True,
            'acceleration_mps2': 0.20,
            'deceleration_mps2': 0.35,
            'emergency_deceleration_mps2': 1.5,
            'command_publish_rate_hz': 20.0,
            'command_timeout_guard_sec': 0.5,
            'stop_on_warning': False,
            'decelerate_on_platform_entry': True,
            'stop_distance_before_platform_end_m': 0.50,
            'expected_start_x_m': -3.0,
            'expected_start_y_m': NAN,
            'expected_start_yaw_deg': 0.0,
            'start_position_tolerance_m': 0.35,
            'start_yaw_tolerance_deg': 5.0,
            'stationary_speed_threshold_mps': 0.03,
            'upright_roll_tolerance_deg': 5.0,
            'upright_pitch_tolerance_deg': 5.0,
            'commanded_angular_velocity_radps': 0.0,
            'max_lane_error_before_abort_m': 0.75,
            'max_heading_error_before_abort_deg': 15.0,
            'precheck_timeout_sec': 30.0,
            'odometry_timeout_sec': 0.5,
            'joint_states_timeout_sec': 0.5,
            'cmd_vel_subscriber_grace_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _value(self, name: str):
        return self.get_parameter(name).value

    def _load_parameters(self) -> None:
        angle = float(self._value('ramp_angle_deg'))
        rounded_angle = int(round(angle))
        if rounded_angle not in RAMP_GEOMETRY or abs(angle - rounded_angle) > 1.0e-6:
            supported = ', '.join(str(value) for value in sorted(RAMP_GEOMETRY))
            raise ValueError(f'unsupported ramp_angle_deg={angle}; supported: {supported}')
        self.ramp_angle_deg = rounded_angle
        self.geometry = RAMP_GEOMETRY[rounded_angle]
        configured_lane = float(self._value('lane_y'))
        self.lane_y = configured_lane if math.isfinite(configured_lane) else self.geometry.lane_y
        self.trial_id = str(self._value('trial_id'))
        self.target_speed_mps = float(self._value('target_speed_mps'))
        self.operator_note = str(self._value('operator_note'))
        self.log_directory = Path(str(self._value('log_directory'))).expanduser()
        self.sample_rate_hz = float(self._value('sample_rate_hz'))
        self.result_hold_sec = float(self._value('result_hold_sec'))
        self.auto_stop = bool(self._value('auto_stop_logging_on_result'))
        self.auto_drive_enabled = bool(self._value('auto_drive_enabled'))
        self.auto_start = bool(self._value('auto_start'))
        self.countdown_sec = float(self._value('countdown_sec'))
        self.shutdown_after_result = bool(self._value('shutdown_after_result'))
        self.stop_hold_sec = float(self._value('stop_hold_sec'))
        self.acceleration_mps2 = float(self._value('acceleration_mps2'))
        self.deceleration_mps2 = float(self._value('deceleration_mps2'))
        self.emergency_deceleration_mps2 = float(
            self._value('emergency_deceleration_mps2')
        )
        self.command_publish_rate_hz = float(
            self._value('command_publish_rate_hz')
        )
        self.command_timeout_guard_sec = float(
            self._value('command_timeout_guard_sec')
        )
        self.stop_on_warning = bool(self._value('stop_on_warning'))
        self.decelerate_on_platform_entry = bool(
            self._value('decelerate_on_platform_entry')
        )
        self.stop_distance_before_platform_end_m = float(
            self._value('stop_distance_before_platform_end_m')
        )
        expected_y = float(self._value('expected_start_y_m'))
        self.precheck_config = PrecheckConfig(
            expected_x_m=float(self._value('expected_start_x_m')),
            expected_y_m=expected_y if math.isfinite(expected_y) else self.lane_y,
            expected_yaw_deg=float(self._value('expected_start_yaw_deg')),
            position_tolerance_m=float(self._value('start_position_tolerance_m')),
            yaw_tolerance_deg=float(self._value('start_yaw_tolerance_deg')),
            stationary_speed_threshold_mps=float(
                self._value('stationary_speed_threshold_mps')
            ),
            upright_roll_tolerance_deg=float(
                self._value('upright_roll_tolerance_deg')
            ),
            upright_pitch_tolerance_deg=float(
                self._value('upright_pitch_tolerance_deg')
            ),
            require_cmd_vel_subscriber=bool(
                self._value('require_cmd_vel_subscriber')
            ),
            require_start_pose=bool(self._value('require_start_pose')),
            require_upright_start=bool(self._value('require_upright_start')),
            require_stationary_start=bool(
                self._value('require_stationary_start')
            ),
        )
        self.max_lane_error_before_abort_m = float(
            self._value('max_lane_error_before_abort_m')
        )
        self.max_heading_error_before_abort_deg = float(
            self._value('max_heading_error_before_abort_deg')
        )
        self.precheck_timeout_sec = float(self._value('precheck_timeout_sec'))
        self.odometry_timeout_sec = float(self._value('odometry_timeout_sec'))
        self.joint_states_timeout_sec = float(
            self._value('joint_states_timeout_sec')
        )
        self.cmd_vel_subscriber_grace_sec = float(
            self._value('cmd_vel_subscriber_grace_sec')
        )
        commanded_angular = float(self._value('commanded_angular_velocity_radps'))
        if abs(commanded_angular) > 1.0e-12:
            raise ValueError(
                'commanded_angular_velocity_radps must be zero for straight tests'
            )
        if self.sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be greater than zero')
        if self.result_hold_sec < 0.0:
            raise ValueError('result_hold_sec must not be negative')
        positive_values = {
            'target_speed_mps': self.target_speed_mps,
            'acceleration_mps2': self.acceleration_mps2,
            'deceleration_mps2': self.deceleration_mps2,
            'emergency_deceleration_mps2': self.emergency_deceleration_mps2,
            'command_publish_rate_hz': self.command_publish_rate_hz,
            'command_timeout_guard_sec': self.command_timeout_guard_sec,
            'precheck_timeout_sec': self.precheck_timeout_sec,
            'odometry_timeout_sec': self.odometry_timeout_sec,
            'joint_states_timeout_sec': self.joint_states_timeout_sec,
        }
        invalid = [name for name, value in positive_values.items() if value <= 0.0]
        if invalid:
            raise ValueError(f'{", ".join(invalid)} must be greater than zero')
        nonnegative_values = {
            'countdown_sec': self.countdown_sec,
            'stop_hold_sec': self.stop_hold_sec,
            'stop_distance_before_platform_end_m': (
                self.stop_distance_before_platform_end_m
            ),
            'cmd_vel_subscriber_grace_sec': self.cmd_vel_subscriber_grace_sec,
        }
        invalid = [name for name, value in nonnegative_values.items() if value < 0.0]
        if invalid:
            raise ValueError(f'{", ".join(invalid)} must not be negative')

        machine_values = {
            name: float(self._value(name))
            for name in (
                'warning_roll_deg', 'warning_pitch_deg', 'topple_roll_deg',
                'topple_pitch_deg', 'minimum_base_height_m',
                'minimum_body_up_alignment', 'topple_hold_sec',
                'success_hold_sec', 'lane_tolerance_m',
                'slide_back_distance_m', 'trial_timeout_sec',
            )
        }
        if machine_values['trial_timeout_sec'] <= 0.0:
            raise ValueError('trial_timeout_sec must be greater than zero')
        self.machine = TrialStateMachine(
            self.geometry,
            self.lane_y,
            **machine_values,
        )
        self.thresholds = machine_values
        self.deceleration_start_x_m = calculate_deceleration_start_x(
            self.geometry,
            self.target_speed_mps,
            self.deceleration_mps2,
            self.stop_distance_before_platform_end_m,
        )
        self.topics = {
            name: str(self._value(name))
            for name in (
                'joint_states_topic', 'cmd_vel_topic', 'wheel_command_topic',
                'emergency_stop_topic', 'odometry_topic', 'imu_topic',
            )
        }

    def _create_log(self) -> None:
        self.log_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone()
        safe_trial_id = re.sub(r'[^A-Za-z0-9_.-]+', '_', self.trial_id).strip('_')
        safe_trial_id = safe_trial_id or 'trial'
        stem = (
            f'{timestamp:%Y%m%d_%H%M%S}_go2w_'
            f'{self.ramp_angle_deg}deg_{safe_trial_id}'
        )
        self.csv_path = (self.log_directory / f'{stem}.csv').resolve()
        self.summary_path = (self.log_directory / f'{stem}_summary.json').resolve()
        self.start_timestamp = timestamp.isoformat()
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=csv_columns())
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _create_ros_interfaces(self) -> None:
        self.create_subscription(
            JointState, self.topics['joint_states_topic'], self._joint_callback,
            qos_profile_sensor_data,
        )
        self.cmd_subscription = self.create_subscription(
            Twist, self.topics['cmd_vel_topic'], self._cmd_callback, 10,
        )
        self.create_subscription(
            Float64MultiArray, self.topics['wheel_command_topic'],
            self._wheel_callback, 10,
        )
        self.create_subscription(
            Bool, self.topics['emergency_stop_topic'], self._emergency_callback, 10,
        )
        self.create_subscription(
            Odometry, self.topics['odometry_topic'], self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, self.topics['imu_topic'], self._imu_callback,
            qos_profile_sensor_data,
        )
        self.state_publisher = self.create_publisher(String, '/go2w/slope_test/state', 10)
        self.toppled_publisher = self.create_publisher(Bool, '/go2w/slope_test/toppled', 10)
        self.orientation_publisher = self.create_publisher(
            Vector3Stamped, '/go2w/slope_test/orientation_deg', 10,
        )
        self.status_publisher = self.create_publisher(String, '/go2w/slope_test/status', 10)
        self.command_publisher = self.create_publisher(
            Twist, self.topics['cmd_vel_topic'], 10,
        )
        self.emergency_stop_publisher = self.create_publisher(
            Bool, self.topics['emergency_stop_topic'], 10,
        )

    def _joint_callback(self, message: JointState) -> None:
        self.last_joint_states_receive_sec = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        self.joints = extract_joint_values(
            message.name, message.position, message.velocity, message.effort,
        )
        self.joints_valid = all(
            finite_values(values[0], values[1]) for values in self.joints.values()
        )
        for name, values in self.joints.items():
            if math.isfinite(values[2]):
                self.seen_joint_effort[name] = True

    def _cmd_callback(self, message: Twist) -> None:
        self.cmd_linear = float(message.linear.x)
        self.cmd_angular = float(message.angular.z)
        self.seen_cmd = finite_values(self.cmd_linear, self.cmd_angular)

    def _wheel_callback(self, message: Float64MultiArray) -> None:
        self.wheel_commands = [safe_sequence_value(message.data, index) for index in range(4)]
        self.seen_wheel_commands = finite_values(*self.wheel_commands)

    def _emergency_callback(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        self.seen_emergency_stop = True

    def _odometry_callback(self, message: Odometry) -> None:
        self.last_odometry_receive_sec = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        pose = message.pose.pose
        quaternion = pose.orientation
        try:
            normalize_quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
            )
        except ValueError:
            self.odometry_valid = False
            return
        self.odometry_valid = finite_values(
            pose.position.x, pose.position.y, pose.position.z,
        )
        if self.odometry_valid:
            self.odometry = message
        twist = message.twist.twist
        if finite_values(
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ):
            self.seen_odometry_twist = True

    def _imu_callback(self, message: Imu) -> None:
        orientation = message.orientation
        try:
            roll, pitch, yaw = quaternion_to_rpy(
                orientation.x, orientation.y, orientation.z, orientation.w,
            )
            rpy = [math.degrees(value) for value in (roll, pitch, yaw)]
        except ValueError:
            rpy = [NAN] * 3
        self.imu_values = rpy + [
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]
        self.seen_imu = finite_values(*self.imu_values)

    @staticmethod
    def _is_fresh(last_receive_sec: float | None, timeout_sec: float, now_sec: float) -> bool:
        return (
            last_receive_sec is not None
            and now_sec >= last_receive_sec
            and now_sec - last_receive_sec <= timeout_sec
        )

    def _odometry_fresh(self, now_sec: float) -> bool:
        return self._is_fresh(
            self.last_odometry_receive_sec, self.odometry_timeout_sec, now_sec,
        )

    def _joint_states_fresh(self, now_sec: float) -> bool:
        return self._is_fresh(
            self.last_joint_states_receive_sec,
            self.joint_states_timeout_sec,
            now_sec,
        )

    def _cmd_vel_subscriber_count(self) -> int:
        # This node observes /cmd_vel itself; exclude that subscription so the
        # precheck verifies that an external command consumer exists.
        return max(0, self.command_publisher.get_subscription_count() - 1)

    def _publish_drive_command(self, speed_mps: float) -> None:
        if not motion_command_publication_enabled(self.auto_drive_enabled):
            return
        command = Twist()
        command.linear.x = command_speed_for_state(
            speed_mps,
            self.machine.state,
            self.machine.topple_condition_active,
        )
        command.angular.z = 0.0
        self.command_publisher.publish(command)
        self.generated_cmd_linear_x_mps = command.linear.x
        self.generated_cmd_angular_z_radps = 0.0
        self.maximum_generated_command_mps = self._updated_extreme(
            self.maximum_generated_command_mps,
            command.linear.x,
        )

    def _publish_zero(self) -> None:
        if not self.auto_drive_enabled:
            return
        self.current_commanded_speed_mps = 0.0
        self._publish_drive_command(0.0)

    def _publish_emergency_stop(self, active: bool) -> None:
        if not self.auto_drive_enabled:
            return
        self.runner_estop_asserted = bool(active)
        message = Bool()
        message.data = bool(active)
        self.emergency_stop_publisher.publish(message)

    def _precheck_sample(
        self,
        now_sec: float,
        base: Sequence[float],
    ) -> PrecheckSample:
        return PrecheckSample(
            odometry_valid=self.odometry_valid,
            joint_states_valid=self.joints_valid,
            odometry_fresh=self._odometry_fresh(now_sec),
            joint_states_fresh=self._joint_states_fresh(now_sec),
            cmd_vel_subscriber_count=self._cmd_vel_subscriber_count(),
            emergency_stop_active=(
                self.emergency_stop is True and not self.runner_estop_asserted
            ),
            base_x_m=base[0],
            base_y_m=base[1],
            yaw_deg=base[5],
            roll_deg=base[3],
            pitch_deg=base[4],
            planar_speed_mps=base[13],
        )

    def _update_precheck_flags(self, base: Sequence[float]) -> None:
        self.start_pose_valid = (
            finite_values(base[0], base[1], base[5])
            and abs(base[0] - self.precheck_config.expected_x_m)
            <= self.precheck_config.position_tolerance_m
            and abs(base[1] - self.precheck_config.expected_y_m)
            <= self.precheck_config.position_tolerance_m
            and abs(angular_error_deg(base[5], self.precheck_config.expected_yaw_deg))
            <= self.precheck_config.yaw_tolerance_deg
        )
        self.upright_start_valid = (
            finite_values(base[3], base[4])
            and abs(base[3]) <= self.precheck_config.upright_roll_tolerance_deg
            and abs(base[4]) <= self.precheck_config.upright_pitch_tolerance_deg
        )
        self.stationary_start_valid = (
            math.isfinite(base[13])
            and base[13] <= self.precheck_config.stationary_speed_threshold_mps
        )

    def _set_abort(
        self,
        reason: str,
        now_sec: float,
        *,
        state_data_timeout: bool = False,
    ) -> None:
        if self.machine.state in TERMINAL_STATES:
            return
        self.abort_reason = reason
        if state_data_timeout and reason not in self.state_data_timeouts:
            self.state_data_timeouts.append(reason)
        self.automation_phase_at_result = self.automation_phase
        self._publish_zero()
        self._publish_emergency_stop(True)
        self.machine.abort(now_sec)
        self._begin_stopping(now_sec, assert_estop=True)

    def _begin_stopping(self, now_sec: float, *, assert_estop: bool) -> None:
        if not self.automation_phase_at_result and self.machine.state in TERMINAL_STATES:
            self.automation_phase_at_result = self.automation_phase
        if self.stop_start_time is None:
            self.stop_start_time = now_sec
        self.stop_assert_estop = getattr(self, 'stop_assert_estop', False) or assert_estop
        self.automation_phase = 'STOPPING'
        self._publish_zero()
        if self.stop_assert_estop:
            self._publish_emergency_stop(True)

    def _active_safety_reason(
        self,
        now_sec: float,
        base: Sequence[float],
    ) -> tuple[str, bool]:
        subscriber_available = self._cmd_vel_subscriber_count() >= 1
        if self.precheck_config.require_cmd_vel_subscriber:
            if not subscriber_available:
                if self.cmd_subscriber_missing_since is None:
                    self.cmd_subscriber_missing_since = now_sec
                if (
                    now_sec - self.cmd_subscriber_missing_since
                    < self.cmd_vel_subscriber_grace_sec
                ):
                    subscriber_available = True
            else:
                self.cmd_subscriber_missing_since = None
        lane_error = base[1] - self.lane_y if math.isfinite(base[1]) else NAN
        heading_error = angular_error_deg(
            base[5], self.precheck_config.expected_yaw_deg,
        )
        reason = automation_safety_abort_reason(
            odometry_fresh=(
                self.odometry_valid and self._odometry_fresh(now_sec)
            ),
            joint_states_fresh=(
                self.joints_valid and self._joint_states_fresh(now_sec)
            ),
            cmd_vel_subscriber_available=subscriber_available,
            require_cmd_vel_subscriber=self.precheck_config.require_cmd_vel_subscriber,
            lane_error_m=lane_error,
            heading_error_deg=heading_error,
            max_lane_error_m=self.max_lane_error_before_abort_m,
            max_heading_error_deg=self.max_heading_error_before_abort_deg,
            emergency_stop_active=(
                self.emergency_stop is True and not self.runner_estop_asserted
            ),
        )
        return reason, reason in {'ODOMETRY_TIMEOUT', 'JOINT_STATES_TIMEOUT'}

    def _run_precheck(self, now_sec: float, base: Sequence[float]) -> None:
        self._publish_zero()
        self.automation_phase = 'PRECHECK'
        self.precheck_start_time, precheck_elapsed = ros_timeout_elapsed(
            self.precheck_start_time, now_sec,
        )
        self._update_precheck_flags(base)
        sample = self._precheck_sample(now_sec, base)
        blockers = precheck_blockers(sample, self.precheck_config)
        self._set_precheck_status(blockers)
        passed = not blockers
        if not passed:
            if precheck_elapsed >= self.precheck_timeout_sec:
                self._set_abort('PRECHECK_TIMEOUT', now_sec)
            return
        if not self.auto_start:
            return
        self.actual_start_pose = {
            'x_m': base[0],
            'y_m': base[1],
            'yaw_deg': base[5],
            'roll_deg': base[3],
            'pitch_deg': base[4],
        }
        self.command_subscriber_count_at_start = sample.cmd_vel_subscriber_count
        self._publish_zero()
        self._publish_emergency_stop(False)
        self.countdown_start_time = now_sec
        self.last_countdown_value = None
        self.automation_phase = 'COUNTDOWN'

    def _set_precheck_status(self, blockers: Sequence[str]) -> None:
        active = tuple(blockers)
        self.precheck_active_blockers = active
        self.precheck_passed = not active
        self.precheck_failure_reason = ';'.join(active)
        if active != self.last_precheck_blockers:
            if active:
                formatted = '\n'.join(f'- {blocker}' for blocker in active)
                self.get_logger().info(f'PRECHECK blockers:\n{formatted}')
            else:
                self.get_logger().info('PRECHECK passed')
            self.last_precheck_blockers = active

    def _return_to_precheck(self, blockers: Sequence[str]) -> None:
        self._set_precheck_status(blockers)
        self.countdown_start_time = None
        self.countdown_remaining_sec = NAN
        self.last_countdown_value = None
        self.automation_phase = 'PRECHECK'
        self._publish_zero()

    def _start_automated_trial(self, now_sec: float, base: Sequence[float]) -> None:
        sample = self._precheck_sample(now_sec, base)
        blockers = precheck_blockers(sample, self.precheck_config)
        if blockers:
            self._return_to_precheck(blockers)
            return
        self._publish_zero()
        self.machine.start(now_sec)
        self.automation_phase = 'ACCELERATING'
        self.countdown_remaining_sec = 0.0
        self.last_command_update_time = now_sec

    def _automation_tick(self) -> None:
        try:
            self._automation_tick_impl()
        except Exception:
            self.close('INTERNAL_ERROR')
            raise

    def _automation_tick_impl(self) -> None:
        if self.closed or not self.auto_drive_enabled:
            return
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        base = self._base_values()

        if self.machine.state in TERMINAL_STATES and self.automation_phase not in {
            'STOPPING', 'COMPLETE',
        }:
            self._begin_stopping(
                now_sec,
                assert_estop=self.machine.state != 'SUCCESS',
            )

        if self.automation_phase in {'INITIALIZING', 'PRECHECK'}:
            self._run_precheck(now_sec, base)
            return

        if self.automation_phase == 'COUNTDOWN':
            blockers = precheck_blockers(
                self._precheck_sample(now_sec, base), self.precheck_config,
            )
            if blockers:
                self._return_to_precheck(blockers)
                return
            self._publish_zero()
            countdown_start = (
                self.countdown_start_time
                if self.countdown_start_time is not None else now_sec
            )
            elapsed = now_sec - countdown_start
            remaining = max(0.0, self.countdown_sec - elapsed)
            self.countdown_remaining_sec = remaining
            shown = int(math.ceil(remaining))
            if shown > 0 and shown != self.last_countdown_value:
                self.get_logger().info(
                    f'[{self.trial_id}] Starting automated trial in {shown}...'
                )
                self.last_countdown_value = shown
            if remaining <= 0.0:
                self._start_automated_trial(now_sec, base)
            return

        if self.automation_phase == 'STOPPING':
            self._publish_zero()
            if getattr(self, 'stop_assert_estop', False):
                self._publish_emergency_stop(True)
            if self.machine.state not in TERMINAL_STATES:
                return
            if (
                self.stop_start_time is not None
                and now_sec - self.stop_start_time >= self.stop_hold_sec
            ):
                self.automation_phase = 'COMPLETE'
                self.close()
                if self.shutdown_after_result and rclpy.ok():
                    rclpy.shutdown()
            return

        if self.automation_phase == 'COMPLETE':
            self._publish_zero()
            return

        reason, is_data_timeout = self._active_safety_reason(now_sec, base)
        if reason:
            self._set_abort(reason, now_sec, state_data_timeout=is_data_timeout)
            return
        if self.machine.slide_back_detected:
            self._set_abort('SLIDE_BACK', now_sec)
            return
        if self.automation_halted_for_topple:
            self._publish_zero()
            return

        last_update = self.last_command_update_time
        elapsed = 0.0 if last_update is None else now_sec - last_update
        self.last_command_update_time = now_sec
        if elapsed > self.command_timeout_guard_sec:
            self._set_abort('COMMAND_TIMEOUT_GUARD', now_sec)
            return

        if (
            self.decelerate_on_platform_entry
            and math.isfinite(base[0])
            and base[0] >= self.deceleration_start_x_m
            and self.automation_phase in {'ACCELERATING', 'CRUISING'}
        ):
            self.automation_phase = 'DECELERATING'

        if self.automation_phase == 'ACCELERATING':
            self.current_commanded_speed_mps = rate_limited_speed(
                self.current_commanded_speed_mps,
                self.target_speed_mps,
                self.acceleration_mps2,
                elapsed,
            )
            if self.current_commanded_speed_mps >= self.target_speed_mps:
                self.automation_phase = 'CRUISING'
        elif self.automation_phase == 'CRUISING':
            self.current_commanded_speed_mps = self.target_speed_mps
            self.cruise_command_sum_mps += self.current_commanded_speed_mps
            self.cruise_command_count += 1
        elif self.automation_phase == 'DECELERATING':
            self.current_commanded_speed_mps = rate_limited_speed(
                self.current_commanded_speed_mps,
                0.0,
                self.deceleration_mps2,
                elapsed,
            )
        self._publish_drive_command(self.current_commanded_speed_mps)
        self.maximum_generated_command_mps = self._updated_extreme(
            self.maximum_generated_command_mps,
            self.current_commanded_speed_mps,
        )

    def _base_values(self) -> list[float]:
        if self.odometry is None or not self.odometry_valid:
            return [NAN] * 14
        pose = self.odometry.pose.pose
        twist = self.odometry.twist.twist
        quaternion = pose.orientation
        try:
            roll, pitch, yaw = quaternion_to_rpy(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
            )
            alignment = body_up_vector_vertical_component(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
            )
        except ValueError:
            return [NAN] * 14
        roll, pitch, yaw = (math.degrees(value) for value in (roll, pitch, yaw))
        planar_speed = (
            math.hypot(twist.linear.x, twist.linear.y)
            if finite_values(twist.linear.x, twist.linear.y) else NAN
        )
        return [
            pose.position.x, pose.position.y, pose.position.z,
            roll, pitch, yaw, alignment,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z, planar_speed,
        ]

    @staticmethod
    def _updated_extreme(previous: float, value: float, use_minimum: bool = False) -> float:
        if not math.isfinite(value):
            return previous
        if not math.isfinite(previous):
            return value
        return min(previous, value) if use_minimum else max(previous, value)

    def _sample(self) -> None:
        try:
            self._sample_impl()
        except Exception:
            self.close('INTERNAL_ERROR')
            raise

    def _sample_impl(self) -> None:
        if self.closed:
            return
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        base = self._base_values()
        x, y, z, roll, pitch, yaw, alignment = base[:7]
        planar_speed = base[13]
        data_ready = self.odometry_valid and self.joints_valid
        if data_ready and self.initial_base_x is None:
            self.initial_base_x = x
        moving = (
            (math.isfinite(self.cmd_linear) and self.cmd_linear > 0.01)
            or (math.isfinite(base[7]) and base[7] > 0.02)
            or (
                self.initial_base_x is not None
                and math.isfinite(x)
                and x > self.initial_base_x + 0.02
            )
        )
        previous_state = self.machine.state
        trial_started = self.machine.start_time is not None
        state = self.machine.step(
            now_sec=now_sec,
            data_ready=data_ready,
            moving_toward_ramp=(
                moving if not self.auto_drive_enabled or trial_started else False
            ),
            roll_deg=roll,
            pitch_deg=pitch,
            base_z=z,
            body_up_alignment=alignment,
            base_x=x,
            base_y=y,
            emergency_stop=(
                self.emergency_stop is True
                and not self.runner_estop_asserted
                and (not self.auto_drive_enabled or trial_started)
            ),
            planar_speed_mps=planar_speed,
            require_stationary_success=self.auto_drive_enabled,
            stationary_speed_threshold_mps=(
                self.precheck_config.stationary_speed_threshold_mps
            ),
        )
        if self.auto_drive_enabled:
            if self.machine.topple_condition_active:
                self.automation_halted_for_topple = True
                self._publish_zero()
                self._publish_emergency_stop(True)
                if state not in TERMINAL_STATES:
                    self._begin_stopping(now_sec, assert_estop=True)
            elif self.automation_halted_for_topple and state not in TERMINAL_STATES:
                self._set_abort('TOPPLE_CONDITION', now_sec)
                state = self.machine.state
            if state == 'WARNING' and self.stop_on_warning:
                self._set_abort('STABILITY_WARNING', now_sec)
                state = self.machine.state
            if self.machine.slide_back_detected and state not in TERMINAL_STATES:
                self._set_abort('SLIDE_BACK', now_sec)
                state = self.machine.state
            if state in TERMINAL_STATES:
                self._begin_stopping(now_sec, assert_estop=state != 'SUCCESS')
        if self.machine.start_time is not None:
            self.maximum_roll_deg = self._updated_extreme(
                self.maximum_roll_deg, abs(roll),
            )
            self.maximum_pitch_deg = self._updated_extreme(
                self.maximum_pitch_deg, abs(pitch),
            )
            self.minimum_base_height_m = self._updated_extreme(
                self.minimum_base_height_m, z, use_minimum=True,
            )
            self.maximum_commanded_speed_mps = self._updated_extreme(
                self.maximum_commanded_speed_mps,
                abs(self.cmd_linear) if math.isfinite(self.cmd_linear) else NAN,
            )
            self.maximum_measured_speed_mps = self._updated_extreme(
                self.maximum_measured_speed_mps, planar_speed,
            )

        row = self._make_row(now_sec, base, state)
        self.csv_writer.writerow(row)
        self.rows_since_flush += 1
        if self.rows_since_flush >= max(1, int(round(self.sample_rate_hz))):
            self.csv_file.flush()
            self.rows_since_flush = 0

        status = self._status_text(now_sec, base)
        self._publish_status(now, state, roll, pitch, yaw, status)
        display_interval = 2.0 if self.automation_phase == 'PRECHECK' else 0.5
        if (
            self.automation_phase != 'COUNTDOWN'
            and (
                state != previous_state
                or now_sec - self.last_display_time >= display_interval
            )
        ):
            self.get_logger().info(status)
            self.last_display_time = now_sec
            self.last_display_state = state
        if state in TERMINAL_STATES and state != previous_state:
            self.get_logger().info(
                f'[{self.trial_id}] result={state} '
                f'elapsed={self.machine.elapsed(now_sec):.2f}s '
                f'trigger={self.machine.topple_trigger or "NONE"} '
                f'slide_back={self.machine.slide_back_detected}'
            )
        if (
            not self.auto_drive_enabled
            and self.auto_stop
            and self.machine.result_time is not None
            and now_sec - self.machine.result_time >= self.result_hold_sec
        ):
            self.close()

    def _make_row(self, now_sec: float, base: Sequence[float], state: str) -> dict:
        row = dict.fromkeys(csv_columns(), NAN)
        base_keys = (
            'base_x_m', 'base_y_m', 'base_z_m', 'roll_deg', 'pitch_deg',
            'yaw_deg', 'body_up_alignment', 'linear_vx_mps', 'linear_vy_mps',
            'linear_vz_mps', 'angular_vx_radps', 'angular_vy_radps',
            'angular_vz_radps', 'planar_speed_mps',
        )
        row.update({
            'sim_time_sec': now_sec,
            'wall_time_iso': datetime.now().astimezone().isoformat(),
            'trial_id': self.trial_id,
            'ramp_angle_deg': self.ramp_angle_deg,
            'target_speed_mps': self.target_speed_mps,
            'state': state,
            'cmd_linear_x_mps': self.cmd_linear,
            'cmd_angular_z_radps': self.cmd_angular,
            'wheel_cmd_fl_radps': self.wheel_commands[0],
            'wheel_cmd_fr_radps': self.wheel_commands[1],
            'wheel_cmd_rl_radps': self.wheel_commands[2],
            'wheel_cmd_rr_radps': self.wheel_commands[3],
            'emergency_stop': (
                self.emergency_stop if self.emergency_stop is not None else NAN
            ),
            'imu_roll_deg': self.imu_values[0],
            'imu_pitch_deg': self.imu_values[1],
            'imu_yaw_deg': self.imu_values[2],
            'imu_angular_x_radps': self.imu_values[3],
            'imu_angular_y_radps': self.imu_values[4],
            'imu_angular_z_radps': self.imu_values[5],
            'imu_accel_x_mps2': self.imu_values[6],
            'imu_accel_y_mps2': self.imu_values[7],
            'imu_accel_z_mps2': self.imu_values[8],
            'warning_active': self.machine.warning_active,
            'topple_condition_active': self.machine.topple_condition_active,
            'slide_back_detected': self.machine.slide_back_detected,
            'success_region_active': self.machine.success_region_active,
            'topple_trigger': self.machine.topple_trigger,
            'auto_drive_enabled': self.auto_drive_enabled,
            'automation_phase': self.automation_phase,
            'precheck_passed': self.precheck_passed,
            'precheck_failure_reason': self.precheck_failure_reason,
            'countdown_remaining_sec': self.countdown_remaining_sec,
            'generated_cmd_linear_x_mps': self.generated_cmd_linear_x_mps,
            'generated_cmd_angular_z_radps': self.generated_cmd_angular_z_radps,
            'acceleration_mps2': self.acceleration_mps2,
            'deceleration_mps2': self.deceleration_mps2,
            'odometry_fresh': self._odometry_fresh(now_sec),
            'joint_states_fresh': self._joint_states_fresh(now_sec),
            'cmd_vel_subscriber_count': self._cmd_vel_subscriber_count(),
            'lane_error_m': (
                base[1] - self.lane_y if math.isfinite(base[1]) else NAN
            ),
            'heading_error_deg': angular_error_deg(
                base[5], self.precheck_config.expected_yaw_deg,
            ),
            'start_pose_valid': self.start_pose_valid,
            'upright_start_valid': self.upright_start_valid,
            'stationary_start_valid': self.stationary_start_valid,
            'abort_reason': self.abort_reason,
            'deceleration_start_x_m': self.deceleration_start_x_m,
            'platform_success_start_x_m': self.geometry.success_min_x,
            'platform_success_end_x_m': self.geometry.success_max_x,
        })
        row.update(dict(zip(base_keys, base)))
        for name, values in self.joints.items():
            row[f'{name}_position_rad'] = values[0]
            row[f'{name}_velocity_radps'] = values[1]
            row[f'{name}_effort_nm'] = values[2]
        return row

    def _status_text(self, now_sec: float, base: Sequence[float]) -> str:
        def shown(value: float) -> str:
            return f'{value:.2f}' if math.isfinite(value) else 'NaN'

        return (
            f'[{self.trial_id}] {self.machine.state}/{self.automation_phase} | '
            f't={self.machine.elapsed(now_sec):.1f}s | '
            f'x={shown(base[0])} y={shown(base[1])} z={shown(base[2])}m | '
            f'roll={shown(base[3])} pitch={shown(base[4])}deg | '
            f'speed={shown(base[13])} cmd={shown(self.cmd_linear)} '
            f'generated={shown(self.generated_cmd_linear_x_mps)}m/s | '
            f'max_pitch={shown(self.maximum_pitch_deg)}deg | '
            f'trigger={self.machine.topple_trigger or "NONE"} | '
            f'precheck={self.precheck_failure_reason or "OK"}'
        )

    def _publish_status(
        self,
        now,
        state: str,
        roll: float,
        pitch: float,
        yaw: float,
        status: str,
    ) -> None:
        state_message = String()
        state_message.data = state
        self.state_publisher.publish(state_message)
        toppled_message = Bool()
        toppled_message.data = state == 'TOPPLED'
        self.toppled_publisher.publish(toppled_message)
        orientation = Vector3Stamped()
        orientation.header.stamp = now.to_msg()
        orientation.header.frame_id = 'world'
        orientation.vector.x = roll
        orientation.vector.y = pitch
        orientation.vector.z = yaw
        self.orientation_publisher.publish(orientation)
        status_message = String()
        status_message.data = status
        self.status_publisher.publish(status_message)

    def _missing_data_fields(self) -> list[str]:
        missing = []
        if not self.seen_cmd:
            missing.extend(('cmd_linear_x_mps', 'cmd_angular_z_radps'))
        if not self.seen_wheel_commands:
            missing.extend((
                'wheel_cmd_fl_radps', 'wheel_cmd_fr_radps',
                'wheel_cmd_rl_radps', 'wheel_cmd_rr_radps',
            ))
        if not self.seen_emergency_stop:
            missing.append('emergency_stop')
        if not self.seen_imu:
            missing.extend((
                'imu_orientation', 'imu_angular_velocity', 'imu_linear_acceleration',
            ))
        if not self.seen_odometry_twist:
            missing.extend(('base_linear_velocity', 'base_angular_velocity'))
        for name, seen in self.seen_joint_effort.items():
            if not seen:
                missing.append(f'{name}_effort_nm')
        return missing

    @staticmethod
    def _json_number(value: float) -> float | None:
        return float(value) if math.isfinite(value) else None

    def close(self, interrupt_reason: str = 'OPERATOR_INTERRUPT') -> None:
        """Close the CSV and atomically complete the trial summary once."""
        if self.closed:
            return
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.machine.state not in TERMINAL_STATES:
            self.abort_reason = self.abort_reason or interrupt_reason
            self.automation_phase_at_result = (
                self.automation_phase_at_result or self.automation_phase
            )
        self._publish_zero()
        self._publish_emergency_stop(True)
        self.machine.abort(now_sec)
        final_base = self._base_values()
        self.closed = True
        try:
            self.timer.cancel()
        except AttributeError:
            pass
        if self.automation_timer is not None:
            self.automation_timer.cancel()
        mean_cruise_command = (
            self.cruise_command_sum_mps / self.cruise_command_count
            if self.cruise_command_count else NAN
        )
        final_pose = {
            'x_m': self._json_number(final_base[0]),
            'y_m': self._json_number(final_base[1]),
            'z_m': self._json_number(final_base[2]),
            'roll_deg': self._json_number(final_base[3]),
            'pitch_deg': self._json_number(final_base[4]),
            'yaw_deg': self._json_number(final_base[5]),
        }
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            summary = {
                'trial_id': self.trial_id,
                'ramp_angle_deg': self.ramp_angle_deg,
                'lane_y': self.lane_y,
                'target_speed_mps': self.target_speed_mps,
                'result': self.machine.state,
                'auto_drive_enabled': self.auto_drive_enabled,
                'automation_phase_at_result': (
                    self.automation_phase_at_result or self.automation_phase
                ),
                'precheck_result': (
                    'NOT_RUN' if not self.auto_drive_enabled
                    else ('PASSED' if self.precheck_passed else 'FAILED')
                ),
                'precheck_failure_reason': self.precheck_failure_reason or None,
                'expected_start_pose': {
                    'x_m': self.precheck_config.expected_x_m,
                    'y_m': self.precheck_config.expected_y_m,
                    'yaw_deg': self.precheck_config.expected_yaw_deg,
                },
                'actual_start_pose': self.actual_start_pose,
                'acceleration_mps2': self.acceleration_mps2,
                'deceleration_mps2': self.deceleration_mps2,
                'maximum_generated_command_mps': self._json_number(
                    self.maximum_generated_command_mps,
                ),
                'mean_cruise_command_mps': self._json_number(mean_cruise_command),
                'deceleration_start_x_m': self.deceleration_start_x_m,
                'final_base_pose': final_pose,
                'abort_reason': self.abort_reason or None,
                'state_data_timeouts': self.state_data_timeouts,
                'command_subscriber_count_at_start': (
                    self.command_subscriber_count_at_start
                ),
                'topple_trigger': self.machine.topple_trigger or None,
                'elapsed_time_sec': self.machine.elapsed(now_sec),
                'time_to_platform_sec': self.machine.time_to_platform_sec,
                'maximum_roll_deg': self._json_number(self.maximum_roll_deg),
                'maximum_pitch_deg': self._json_number(self.maximum_pitch_deg),
                'minimum_base_height_m': self._json_number(self.minimum_base_height_m),
                'maximum_forward_x_m': self._json_number(
                    self.machine.maximum_forward_x,
                ),
                'maximum_commanded_linear_speed_mps': self._json_number(
                    self.maximum_commanded_speed_mps,
                ),
                'maximum_measured_planar_speed_mps': self._json_number(
                    self.maximum_measured_speed_mps,
                ),
                'slide_back_detected': self.machine.slide_back_detected,
                'csv_path': str(self.csv_path),
                'operator_note': self.operator_note,
                'start_timestamp': self.start_timestamp,
                'end_timestamp': datetime.now().astimezone().isoformat(),
                'data_source_topics': self.topics,
                'missing_data_fields': self._missing_data_fields(),
                'configuration_thresholds': self.thresholds,
            }
            temporary_path = self.summary_path.with_suffix('.json.tmp')
            temporary_path.write_text(
                json.dumps(summary, indent=2, allow_nan=False) + '\n',
                encoding='utf-8',
            )
            temporary_path.replace(self.summary_path)
        print(
            f'[{self.trial_id}] {self.machine.state} | '
            f'elapsed={self.machine.elapsed(now_sec):.2f}s | '
            f'max_roll={self._json_number(self.maximum_roll_deg)}deg | '
            f'max_pitch={self._json_number(self.maximum_pitch_deg)}deg | '
            f'trigger={self.machine.topple_trigger or "NONE"} | '
            f'slide_back={self.machine.slide_back_detected}\n'
            f'CSV: {self.csv_path}\nJSON: {self.summary_path}',
            flush=True,
        )


def main(args=None) -> None:
    """Run the slope monitor."""
    # Keep the context valid while KeyboardInterrupt runs the explicit safe-stop
    # path; the default rclpy SIGINT handler shuts the context down too early.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Go2WSlopeTestMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.close('OPERATOR_INTERRUPT')
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ground_truth_adapter_main(args=None) -> None:
    """Run the Gazebo PoseArray to Odometry adapter."""
    rclpy.init(args=args)
    node = None
    try:
        node = GroundTruthOdometryAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
