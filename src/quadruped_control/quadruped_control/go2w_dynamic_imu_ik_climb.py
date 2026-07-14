#!/usr/bin/env python3
"""Adaptive, slope-independent IMU/IK ramp-climb ROS orchestration node."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from dataclasses import replace
from typing import Any, Sequence

from controller_manager_msgs.srv import ListControllers

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from geometry_msgs.msg import PoseArray

from nav_msgs.msg import Odometry

from quadruped_control.go2w_adaptive_attitude_controller import (
    AdaptiveAttitudeController,
    AttitudeControllerConfig,
)
from quadruped_control.go2w_dynamic_models import (
    AttitudeCommand,
    ClimbState,
    ContactPhase,
    ControllerMode,
    FeasibilityResult,
    JointCommandResult,
    MotionCommand,
    SafetyState,
    StabilityResult,
    TerrainEstimate,
    TractionResult,
)
from quadruped_control.go2w_dynamic_run_logger import (
    DynamicRunLogger,
    RunLoggerConfig,
)
from quadruped_control.go2w_feasibility_solver import (
    FeasibilityConfig,
    OnlineFeasibilitySolver,
)
from quadruped_control.go2w_motion_supervisor import (
    AdaptiveClimbStateMachine,
    ClimbStateMachineConfig,
    MotionSupervisor,
    MotionSupervisorConfig,
    rate_limit,
)
from quadruped_control.go2w_stability_supervisor import (
    StabilityConfig,
    StabilitySupervisor,
    TractionConfig,
    TractionSupervisor,
)
from quadruped_control.go2w_terrain_estimator import (
    TerrainEstimator,
    TerrainEstimatorConfig,
)

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Imu, JointState

from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from std_srvs.srv import Trigger


LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in LEGS
    for joint in ('hip', 'thigh', 'calf')
)
WHEEL_JOINT_NAMES = (
    'FL_foot_joint', 'FR_foot_joint', 'RL_foot_joint', 'RR_foot_joint',
)
DRIVE_STATES = {
    ClimbState.FLAT_DRIVE,
    ClimbState.RAMP_APPROACH,
    ClimbState.MIXED_CONTACT_ENTRY,
    ClimbState.CLIMB,
    ClimbState.MIXED_CONTACT_EXIT,
}
COMMAND_STATES = {
    ClimbState.STARTUP_SETTLE,
    *DRIVE_STATES,
    ClimbState.MIXED_CONTACT_EXIT,
    ClimbState.TOP_TRANSITION,
    ClimbState.TOP_HOLD,
    ClimbState.COMPLETE,
    ClimbState.SAFE_INFEASIBLE,
    ClimbState.FAULT,
    ClimbState.TIMEOUT,
}


class MotionRecoveryRunLogger(DynamicRunLogger):
    """Extend the unchanged run logger schema with motion recovery state."""

    BASE_FIELDS = (
        *DynamicRunLogger.BASE_FIELDS,
        'motion_paused',
        'motion_full_slope_recovered',
        'motion_recovery_floor_mps',
        'motion_mixed_contact_escape_active',
        'motion_mixed_contact_escape_speed_mps',
        'motion_mixed_contact_escape_elapsed_sec',
        'motion_mixed_contact_escape_margin_m',
        'motion_mixed_contact_escape_reason',
    )


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to inclusive bounds."""
    return max(lower, min(upper, value))


def finite(*values: float) -> bool:
    """Report whether all scalar inputs are finite."""
    return all(math.isfinite(value) for value in values)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float,
) -> tuple[float, float, float]:
    """Convert a normalized quaternion to roll, pitch, and yaw."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1.0 else math.asin(sinp)
    )
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


class Go2WDynamicImuIkClimb(Node):
    """Orchestrate pure adaptive-control modules through ROS."""

    def __init__(self) -> None:
        """Configure pure modules, ROS interfaces, logging, and timer."""
        super().__init__(
            'go2w_dynamic_imu_ik_climb',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=False,
        )
        self._declare_parameters()
        self._load_and_validate_parameters()
        self._create_modules()

        self.stop_requested = False
        self.start_requested = (
            self.auto_start and self.mode != ControllerMode.MONITOR
        )
        self.pending_fault_reason = ''
        self.fault_reason = ''
        self.infeasible_reason = ''
        self.safe_stop_reason = ''
        self.final_result = ''
        self.finalized = False
        self.run_start_ros_sec: float | None = None
        self.run_start_wall_sec: float | None = None
        self.node_start_wall_sec = time.monotonic()
        self.state_machine = AdaptiveClimbStateMachine(
            ClimbStateMachineConfig(
                startup_settle_sec=self.startup_settle_sec,
                top_hold_sec=self.top_hold_sec,
                maximum_run_time_sec=self.maximum_run_time_sec,
            )
        )

        self.raw_yaw = 0.0
        self.vertical_accel_mps2 = 0.0
        self.imu_valid = False
        self.last_imu_stamp_sec: float | None = None
        self.last_imu_wall_sec: float | None = None
        self.latest_joints: dict[str, float] = {}
        self.wheel_velocities_radps = (0.0, 0.0, 0.0, 0.0)
        self.last_joint_wall_sec: float | None = None

        self.pose_observations: dict[str, dict[str, Any]] = {}
        self.pose_array_previous: tuple[float, ...] | None = None
        self.pose_source = ''
        self.pose_source_lock = ''
        self.pose_valid = False
        self.pose_missing_since_wall: float | None = None
        self.pose_stamp_sec: float | None = None
        self.last_pose_wall_sec: float | None = None
        self.base_x = self.base_y = self.base_z = 0.0
        self.base_vx = self.base_vy = self.base_vz = 0.0
        self.base_yaw = self.base_yaw_rate = 0.0

        self.initial_x = self.initial_y = self.initial_z = 0.0
        self.initial_yaw = 0.0
        self.heading_x = 1.0
        self.heading_y = 0.0
        self.heading_error = 0.0
        self.measured_forward_speed = 0.0
        self.maximum_progress = 0.0
        self.maximum_height_gain = 0.0
        self.maximum_estimated_slope_deg = 0.0
        self.start_pose: dict[str, float] | None = None
        self.last_terrain_pose_stamp: float | None = None

        self.nominal_feet: dict[str, tuple[float, float, float]] = {}
        self.current_feet: dict[str, tuple[float, float, float]] = {}
        self.captured_joints: list[float] = []
        self.initial_joint_margin_rad = 0.0
        self.last_safe_leg_command: list[float] = []
        self.proposed_leg_command: list[float] = []
        self.ik_failures = 0
        self.workspace_exhausted_since: float | None = None
        self.stability_stop_since: float | None = None
        self.last_logged_feasibility: tuple[Any, ...] | None = None

        self.terrain = TerrainEstimate()
        self.feasibility = FeasibilityResult()
        self.stability = StabilityResult()
        self.traction = TractionResult()
        self.attitude_command = AttitudeCommand()
        self.joint_command = JointCommandResult(False)
        self.motion_command = MotionCommand()
        self.ramp_detected = False
        self.top_detected = False

        self.controllers_active = False
        self.controller_status_known = False
        self.command_topics_ready = False
        self.ownership_exclusive = False
        self.controller_conflict = False
        self.controller_future = None
        self.last_controller_poll_wall_sec = 0.0
        self.last_graph_poll_wall_sec = 0.0
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
        )

        self.bridge_process: subprocess.Popen | None = None
        self.bridge_start_attempts = 0
        self.bridge_next_start_wall_sec = (
            self.node_start_wall_sec + self.pose_bridge_start_delay_sec
        )

        self.logger = MotionRecoveryRunLogger(
            RunLoggerConfig(
                output_directory=self.output_directory,
                run_id=self.run_id,
                expected_slope_deg=self.expected_slope_deg,
                flush_period_sec=self.log_flush_period_sec,
            ),
            LEG_JOINT_NAMES,
        )
        self._create_ros_interfaces()
        self.last_control_wall_sec = time.monotonic()
        self.last_ros_time_sec: float | None = None
        self.last_status_wall_sec = 0.0
        self.last_contact_phase = self.terrain.contact_phase
        self.last_stability_state = self.stability.state
        self.last_escape_active = False
        self.last_escape_reason = ''
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_cycle,
            clock=steady_clock,
        )
        self.get_logger().info(
            'configuration: '
            f'mode={self.mode.value} '
            f'auto_start={str(self.auto_start).lower()} '
            f'expected_slope={self.expected_slope_deg:.3f} deg '
            f'desired_level_pitch={self.desired_level_pitch_deg:.3f} deg '
            f'maximum_requested_speed={self.commanded_speed_mps:.3f} m/s '
            f'adaptive_target={str(self.adaptive_target_enabled).lower()} '
            f'feedforward={str(self.terrain_feedforward_enabled).lower()} '
            f'run_id={self.logger.run_id}'
        )

    @property
    def state(self) -> ClimbState:
        """Expose the current pure state-machine state."""
        return self.state_machine.state

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            'mode': 'shadow',
            'auto_start': False,
            'expected_slope_deg': 0.0,
            'target_pitch_deg': 0.0,
            'desired_level_pitch_deg': 0.0,
            'desired_roll_deg': 0.0,
            'adaptive_target_enabled': True,
            'terrain_feedforward_enabled': True,
            'roll_control_enabled': False,
            'commanded_speed_mps': 0.10,
            'wheel_accel_limit_mps2': 0.15,
            'wheel_decel_limit_mps2': 0.30,
            'wheel_radius_m': 0.086,
            'track_width_m': 0.3802,
            'wheelbase_m': 0.3868,
            'wheel_velocity_sign_fl': 1.0,
            'wheel_velocity_sign_fr': 1.0,
            'wheel_velocity_sign_rl': 1.0,
            'wheel_velocity_sign_rr': 1.0,
            'control_rate_hz': 100.0,
            'pitch_filter_cutoff_hz': 5.0,
            'roll_filter_cutoff_hz': 4.0,
            'feedforward_gain': 1.0,
            'pitch_kp_flat': 0.55,
            'pitch_kp_entry': 0.75,
            'pitch_kp_climb': 0.65,
            'pitch_kd_flat': 0.08,
            'pitch_kd_entry': 0.12,
            'pitch_kd_climb': 0.10,
            'kp': 0.65,
            'ki': 0.0,
            'kd': 0.10,
            'integral_limit': 8.0,
            'roll_kp': 0.30,
            'roll_kd': 0.06,
            'max_body_correction_deg': 25.0,
            'max_leg_offset_m': 0.060,
            'max_offset_rate_mps': 0.020,
            'feasibility_update_hz': 15.0,
            'joint_limit_margin_rad': 0.035,
            'workspace_margin_m': 0.0005,
            'absolute_roll_correction_deg': 5.0,
            'imu_timeout_sec': 0.20,
            'joint_state_timeout_sec': 0.20,
            'pose_timeout_sec': 0.25,
            'pose_startup_grace_sec': 0.60,
            'startup_settle_sec': 2.0,
            'flat_start_pitch_tolerance_deg': 3.0,
            'flat_start_roll_tolerance_deg': 3.0,
            'allow_nonlevel_start': False,
            'fast_slope_window_sec': 0.25,
            'stable_slope_window_sec': 0.75,
            'minimum_slope_window_progress_m': 0.018,
            'ramp_entry_slope_threshold_deg': 2.5,
            'top_flat_slope_threshold_deg': 1.8,
            'ramp_entry_hold_sec': 0.12,
            'top_flat_hold_sec': 0.80,
            'transition_confidence_threshold': 0.45,
            'minimum_crest_progress_m': 0.80,
            'minimum_crest_height_m': 0.15,
            'maximum_uphill_slope_deg': 50.0,
            'maximum_run_time_sec': 360.0,
            'top_hold_sec': 3.0,
            'heading_hold_enabled': True,
            'yaw_kp': 0.080,
            'yaw_kd': 0.010,
            'maximum_yaw_correction_mps': 0.025,
            'maximum_heading_error_deg': 25.0,
            'entry_speed_scale': 0.45,
            'crest_speed_scale': 0.40,
            'minimum_speed_scale': 0.25,
            'full_slope_speed_scale': 0.80,
            'full_slope_recovery_hold_sec': 0.50,
            'full_slope_pitch_error_limit_deg': 3.0,
            'full_slope_workspace_limit': 0.90,
            'slope_speed_gain': 0.018,
            'pitch_error_speed_gain': 0.10,
            'workspace_speed_gain': 0.35,
            'safe_pause_hold_sec': 0.35,
            'stability_caution_margin_m': 0.045,
            'stability_stop_margin_m': 0.015,
            'mixed_contact_support_inset_m': 0.012,
            'base_com_x_m': 0.021112,
            'base_com_y_m': 0.0,
            'base_com_z_m': -0.005366,
            'slip_caution_threshold': 0.30,
            'slip_stop_threshold': 0.55,
            'slip_hold_sec': 0.40,
            'infeasible_hold_sec': 1.50,
            'slip_epsilon_mps': 0.025,
            'mixed_contact_escape_enabled': True,
            'mixed_contact_escape_speed_mps': 0.012,
            'mixed_contact_escape_margin_min_m': 0.005,
            'mixed_contact_escape_margin_max_m': 0.015,
            'mixed_contact_escape_workspace_limit': 0.90,
            'mixed_contact_escape_pitch_error_limit_deg': 18.0,
            'mixed_contact_escape_max_duration_sec': 2.0,
            'mixed_contact_escape_rearm_sec': 0.50,
            'mixed_contact_escape_progress_timeout_sec': 0.75,
            'mixed_contact_escape_min_progress_m': 0.003,
            'output_directory': '/home/svm/quad_ws/logs/dynamic_imu_ik',
            'run_id': '',
            'shutdown_after_top': False,
            'imu_topic': '/imu/data',
            'joint_state_topic': '/joint_states',
            'odometry_topic': '/go2w/ground_truth/odom',
            'leg_command_topic': '/go2w_leg_position_controller/commands',
            'wheel_command_topic': '/go2w_wheel_velocity_controller/commands',
            'expected_imu_frame': 'imu',
            'leg_controller_name': 'go2w_leg_position_controller',
            'wheel_controller_name': 'go2w_wheel_velocity_controller',
            'max_consecutive_ik_failures': 3,
            'max_roll_deg': 35.0,
            'max_pitch_deg': 40.0,
            'maximum_backward_progress_m': 0.10,
            'joint_limit_tolerance_rad': 0.02,
            'controller_poll_period_sec': 0.20,
            'data_wait_timeout_sec': 15.0,
            'pose_bridge_start_delay_sec': 1.0,
            'manage_pose_bridge': True,
            'gazebo_pose_topic': '/world/go2w_slope_test/dynamic_pose/info',
            'pose_array_topic': '/go2w/dynamic_imu_ik/pose_array',
            'ground_truth_model_pose_index': 0,
            'log_flush_period_sec': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_and_validate_parameters(self) -> None:
        strings = {
            'mode', 'output_directory', 'run_id', 'imu_topic',
            'joint_state_topic', 'odometry_topic', 'leg_command_topic',
            'wheel_command_topic', 'expected_imu_frame',
            'leg_controller_name', 'wheel_controller_name',
            'gazebo_pose_topic', 'pose_array_topic',
        }
        booleans = {
            'auto_start', 'adaptive_target_enabled',
            'terrain_feedforward_enabled', 'roll_control_enabled',
            'allow_nonlevel_start', 'heading_hold_enabled',
            'shutdown_after_top', 'manage_pose_bridge',
            'mixed_contact_escape_enabled',
        }
        integers = {
            'max_consecutive_ik_failures',
            'ground_truth_model_pose_index',
        }
        for name, parameter in self._parameters.items():
            if name == 'use_sim_time':
                continue
            if name in strings:
                value: Any = str(parameter.value)
            elif name in booleans:
                if type(parameter.value) is not bool:
                    raise ValueError(f'{name} must be bool')
                value = parameter.value
            elif name in integers:
                value = int(parameter.value)
            else:
                value = float(parameter.value)
                if not math.isfinite(value):
                    raise ValueError(f'{name} must be finite')
            setattr(self, name, value)
        try:
            self.mode = ControllerMode(self.mode)
        except ValueError as error:
            raise ValueError(
                'mode must be monitor, shadow, or active'
            ) from error
        if not 0.0 <= self.expected_slope_deg <= self.maximum_uphill_slope_deg:
            raise ValueError(
                'expected_slope_deg must be 0/auto or within range'
            )
        if not 0.0 < self.commanded_speed_mps <= 2.0:
            raise ValueError('commanded_speed_mps must be in (0, 2]')
        if not 0.0 <= self.full_slope_speed_scale <= 1.0:
            raise ValueError('full_slope_speed_scale must be in [0, 1]')
        if self.full_slope_recovery_hold_sec < 0.0:
            raise ValueError(
                'full_slope_recovery_hold_sec must be non-negative'
            )
        if self.full_slope_pitch_error_limit_deg <= 0.0:
            raise ValueError(
                'full_slope_pitch_error_limit_deg must be greater than zero'
            )
        if not 0.0 < self.full_slope_workspace_limit <= 1.0:
            raise ValueError('full_slope_workspace_limit must be in (0, 1]')
        if type(self.mixed_contact_escape_enabled) is not bool:
            raise ValueError('mixed_contact_escape_enabled must be bool')
        if self.mixed_contact_escape_speed_mps <= 0.0:
            raise ValueError(
                'mixed_contact_escape_speed_mps must be greater than zero'
            )
        if self.mixed_contact_escape_speed_mps > self.commanded_speed_mps:
            raise ValueError(
                'mixed_contact_escape_speed_mps must not exceed '
                'commanded_speed_mps'
            )
        if self.mixed_contact_escape_margin_min_m < 0.0:
            raise ValueError(
                'mixed_contact_escape_margin_min_m must be non-negative'
            )
        if (
            self.mixed_contact_escape_margin_max_m
            <= self.mixed_contact_escape_margin_min_m
        ):
            raise ValueError('invalid mixed-contact escape margin band')
        if (
            self.mixed_contact_escape_margin_max_m
            > self.stability_stop_margin_m
        ):
            raise ValueError(
                'mixed_contact_escape_margin_max_m must not exceed '
                'stability_stop_margin_m'
            )
        if not 0.0 < self.mixed_contact_escape_workspace_limit <= 1.0:
            raise ValueError(
                'mixed_contact_escape_workspace_limit must be in (0, 1]'
            )
        if self.mixed_contact_escape_pitch_error_limit_deg <= 0.0:
            raise ValueError(
                'mixed_contact_escape_pitch_error_limit_deg must be positive'
            )
        if self.mixed_contact_escape_max_duration_sec <= 0.0:
            raise ValueError(
                'mixed_contact_escape_max_duration_sec must be positive'
            )
        if self.mixed_contact_escape_rearm_sec < 0.0:
            raise ValueError(
                'mixed_contact_escape_rearm_sec must be non-negative'
            )
        if self.mixed_contact_escape_progress_timeout_sec <= 0.0:
            raise ValueError(
                'mixed_contact_escape_progress_timeout_sec must be positive'
            )
        if self.mixed_contact_escape_min_progress_m < 0.0:
            raise ValueError(
                'mixed_contact_escape_min_progress_m must be non-negative'
            )
        if not 20.0 <= self.control_rate_hz <= 250.0:
            raise ValueError('control_rate_hz must be in [20, 250]')
        if self.pitch_filter_cutoff_hz >= 0.45 * self.control_rate_hz:
            raise ValueError(
                'pitch filter cutoff is too high for control rate'
            )
        if not -10.0 <= self.desired_level_pitch_deg <= 10.0:
            raise ValueError('desired_level_pitch_deg is outside safe range')
        if not 0.0 < self.max_body_correction_deg <= 30.0:
            raise ValueError('max_body_correction_deg must be in (0, 30]')
        if self.max_leg_offset_m <= 0.0 or self.max_leg_offset_m > 0.080:
            raise ValueError('max_leg_offset_m must be in (0, 0.080]')
        if self.ground_truth_model_pose_index < 0:
            raise ValueError(
                'ground_truth_model_pose_index must be non-negative'
            )
        signs = (
            self.wheel_velocity_sign_fl, self.wheel_velocity_sign_fr,
            self.wheel_velocity_sign_rl, self.wheel_velocity_sign_rr,
        )
        if any(value not in (-1.0, 1.0) for value in signs):
            raise ValueError('wheel velocity signs must each be -1 or 1')
        positive = (
            'wheel_accel_limit_mps2', 'wheel_decel_limit_mps2',
            'wheel_radius_m', 'wheelbase_m', 'imu_timeout_sec',
            'joint_state_timeout_sec', 'pose_timeout_sec',
            'startup_settle_sec', 'fast_slope_window_sec',
            'stable_slope_window_sec', 'feasibility_update_hz',
            'maximum_run_time_sec', 'top_hold_sec',
            'controller_poll_period_sec', 'data_wait_timeout_sec',
            'log_flush_period_sec', 'infeasible_hold_sec',
        )
        for name in positive:
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        if not self.output_directory:
            raise ValueError('output_directory must not be empty')

    def _create_modules(self) -> None:
        self.terrain_estimator = TerrainEstimator(TerrainEstimatorConfig(
            fast_window_sec=self.fast_slope_window_sec,
            stable_window_sec=self.stable_slope_window_sec,
            minimum_progress_m=self.minimum_slope_window_progress_m,
            entry_slope_deg=self.ramp_entry_slope_threshold_deg,
            flat_slope_deg=self.top_flat_slope_threshold_deg,
            transition_confidence=self.transition_confidence_threshold,
            minimum_crest_progress_m=self.minimum_crest_progress_m,
            minimum_crest_height_m=self.minimum_crest_height_m,
            entry_hold_sec=self.ramp_entry_hold_sec,
            top_hold_sec=self.top_flat_hold_sec,
            wheelbase_m=self.wheelbase_m,
            maximum_slope_deg=self.maximum_uphill_slope_deg + 5.0,
        ))
        self.attitude_controller = AdaptiveAttitudeController(
            AttitudeControllerConfig(
                desired_level_pitch_deg=self.desired_level_pitch_deg,
                desired_roll_deg=self.desired_roll_deg,
                adaptive_target_enabled=self.adaptive_target_enabled,
                terrain_feedforward_enabled=self.terrain_feedforward_enabled,
                roll_control_enabled=self.roll_control_enabled,
                feedforward_gain=self.feedforward_gain,
                pitch_kp_flat=self.pitch_kp_flat,
                pitch_kp_entry=self.pitch_kp_entry,
                pitch_kp_climb=self.pitch_kp_climb,
                pitch_kd_flat=self.pitch_kd_flat,
                pitch_kd_entry=self.pitch_kd_entry,
                pitch_kd_climb=self.pitch_kd_climb,
                pitch_ki=self.ki,
                integral_limit_deg_sec=self.integral_limit,
                roll_kp=self.roll_kp,
                roll_kd=self.roll_kd,
                pitch_filter_cutoff_hz=self.pitch_filter_cutoff_hz,
                roll_filter_cutoff_hz=self.roll_filter_cutoff_hz,
                absolute_correction_cap_deg=self.max_body_correction_deg,
                absolute_roll_cap_deg=self.absolute_roll_correction_deg,
            )
        )
        self.feasibility_solver = OnlineFeasibilitySolver(FeasibilityConfig(
            update_hz=self.feasibility_update_hz,
            joint_limit_margin_rad=self.joint_limit_margin_rad,
            workspace_margin_m=self.workspace_margin_m,
            absolute_pitch_cap_deg=self.max_body_correction_deg,
            absolute_roll_cap_deg=self.absolute_roll_correction_deg,
            absolute_leg_offset_cap_m=self.max_leg_offset_m,
        ))
        self.stability_supervisor = StabilitySupervisor(StabilityConfig(
            base_com_x_m=self.base_com_x_m,
            base_com_y_m=self.base_com_y_m,
            base_com_z_m=self.base_com_z_m,
            wheel_radius_m=self.wheel_radius_m,
            caution_margin_m=self.stability_caution_margin_m,
            stop_margin_m=self.stability_stop_margin_m,
            mixed_contact_inset_m=self.mixed_contact_support_inset_m,
        ))
        signs = (
            self.wheel_velocity_sign_fl, self.wheel_velocity_sign_fr,
            self.wheel_velocity_sign_rl, self.wheel_velocity_sign_rr,
        )
        self.traction_supervisor = TractionSupervisor(TractionConfig(
            wheel_radius_m=self.wheel_radius_m,
            wheel_forward_signs=signs,
            slip_epsilon_mps=self.slip_epsilon_mps,
            caution_threshold=self.slip_caution_threshold,
            stop_threshold=self.slip_stop_threshold,
            slip_hold_sec=self.slip_hold_sec,
            infeasible_hold_sec=self.infeasible_hold_sec,
        ))
        self.motion_supervisor = MotionSupervisor(MotionSupervisorConfig(
            requested_speed_mps=self.commanded_speed_mps,
            wheel_radius_m=self.wheel_radius_m,
            wheel_accel_limit_mps2=self.wheel_accel_limit_mps2,
            wheel_decel_limit_mps2=self.wheel_decel_limit_mps2,
            yaw_kp=self.yaw_kp,
            yaw_kd=self.yaw_kd,
            maximum_yaw_correction_mps=self.maximum_yaw_correction_mps,
            entry_speed_scale=self.entry_speed_scale,
            crest_speed_scale=self.crest_speed_scale,
            minimum_speed_scale=self.minimum_speed_scale,
            full_slope_speed_scale=self.full_slope_speed_scale,
            full_slope_recovery_hold_sec=self.full_slope_recovery_hold_sec,
            full_slope_pitch_error_limit_deg=(
                self.full_slope_pitch_error_limit_deg
            ),
            full_slope_workspace_limit=self.full_slope_workspace_limit,
            slope_speed_gain=self.slope_speed_gain,
            pitch_error_speed_gain=self.pitch_error_speed_gain,
            workspace_speed_gain=self.workspace_speed_gain,
            stability_caution_margin_m=self.stability_caution_margin_m,
            stability_stop_margin_m=self.stability_stop_margin_m,
            slip_caution_threshold=self.slip_caution_threshold,
            slip_stop_threshold=self.slip_stop_threshold,
            safe_pause_hold_sec=self.safe_pause_hold_sec,
            mixed_contact_escape_enabled=(
                self.mixed_contact_escape_enabled
            ),
            mixed_contact_escape_speed_mps=(
                self.mixed_contact_escape_speed_mps
            ),
            mixed_contact_escape_margin_min_m=(
                self.mixed_contact_escape_margin_min_m
            ),
            mixed_contact_escape_margin_max_m=(
                self.mixed_contact_escape_margin_max_m
            ),
            mixed_contact_escape_workspace_limit=(
                self.mixed_contact_escape_workspace_limit
            ),
            mixed_contact_escape_pitch_error_limit_deg=(
                self.mixed_contact_escape_pitch_error_limit_deg
            ),
            mixed_contact_escape_max_duration_sec=(
                self.mixed_contact_escape_max_duration_sec
            ),
            mixed_contact_escape_rearm_sec=(
                self.mixed_contact_escape_rearm_sec
            ),
            mixed_contact_escape_progress_timeout_sec=(
                self.mixed_contact_escape_progress_timeout_sec
            ),
            mixed_contact_escape_min_progress_m=(
                self.mixed_contact_escape_min_progress_m
            ),
        ))

    def _create_ros_interfaces(self) -> None:
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 20)
        self.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_state, 20,
        )
        self.create_subscription(
            Odometry, self.odometry_topic, self._on_odometry, 20,
        )
        self.create_subscription(
            PoseArray, self.pose_array_topic, self._on_pose_array, 20,
        )
        self.create_service(Trigger, '~/start', self._on_start)
        self.leg_command_publisher = None
        self.wheel_command_publisher = None
        if self.mode == ControllerMode.ACTIVE:
            self.leg_command_publisher = self.create_publisher(
                Float64MultiArray, self.leg_command_topic, 10,
            )
            self.wheel_command_publisher = self.create_publisher(
                Float64MultiArray, self.wheel_command_topic, 10,
            )
        prefix = '/go2w/dynamic_imu_ik'
        self.state_publisher = self.create_publisher(
            String, f'{prefix}/state', 10,
        )
        float_topics = {
            'raw_pitch': 'pitch/raw_deg',
            'filtered_pitch': 'pitch/filtered_deg',
            'target_pitch': 'pitch/target_deg',
            'desired_level_pitch': 'pitch/desired_level_deg',
            'feasible_target_pitch': 'pitch/feasible_target_deg',
            'pitch_error': 'pitch/error_deg',
            'pitch_rate': 'pitch/rate_radps',
            'pid_p': 'pid/p_term',
            'pid_i': 'pid/i_term',
            'pid_d': 'pid/d_term',
            'pid_output': 'pid/output_deg',
            'feedforward': 'pitch/feedforward_deg',
            'feedback': 'pitch/feedback_deg',
            'final_correction': 'pitch/final_correction_deg',
            'raw_roll': 'roll/raw_deg',
            'filtered_roll': 'roll/filtered_deg',
            'target_roll': 'roll/target_deg',
            'roll_correction': 'roll/correction_deg',
            'front_offset': 'offset/front_m',
            'rear_offset': 'offset/rear_m',
            'terrain_slope': 'terrain/slope_estimate_deg',
            'fast_slope': 'terrain/fast_slope_deg',
            'stable_slope': 'terrain/stable_slope_deg',
            'slope_confidence': 'terrain/slope_confidence',
            'slope_rate': 'terrain/slope_rate_degps',
            'height_gain': 'terrain/height_gain_m',
            'forward_progress': 'terrain/forward_progress_m',
            'max_positive': 'feasibility/max_positive_correction_deg',
            'max_negative': 'feasibility/max_negative_correction_deg',
            'joint_margin': 'feasibility/joint_margin_rad',
            'longitudinal_margin': 'stability/longitudinal_margin_m',
            'lateral_margin': 'stability/lateral_margin_m',
            'minimum_margin': 'stability/minimum_margin_m',
            'slip_fl': 'traction/slip_fl',
            'slip_fr': 'traction/slip_fr',
            'slip_rl': 'traction/slip_rl',
            'slip_rr': 'traction/slip_rr',
            'average_slip': 'traction/average_abs_slip',
            'requested_speed': 'motion/requested_speed_mps',
            'effective_speed': 'motion/effective_speed_mps',
            'commanded_speed': 'motion/commanded_speed_mps',
            'measured_speed': 'motion/measured_forward_speed_mps',
            'slope_scale': 'motion/slope_scale',
            'transition_scale': 'motion/transition_scale',
            'pitch_scale': 'motion/pitch_scale',
            'stability_scale': 'motion/stability_scale',
            'traction_scale': 'motion/traction_scale',
            'workspace_scale': 'motion/workspace_scale',
            'recovery_floor': 'motion/recovery_floor_mps',
            'mixed_contact_escape_speed': (
                'motion/mixed_contact_escape_speed_mps'
            ),
            'mixed_contact_escape_elapsed': (
                'motion/mixed_contact_escape_elapsed_sec'
            ),
            'mixed_contact_escape_margin': (
                'motion/mixed_contact_escape_margin_m'
            ),
        }
        self.float_publishers = {
            key: self.create_publisher(Float64, f'{prefix}/{topic}', 10)
            for key, topic in float_topics.items()
        }
        bool_topics = {
            'ramp_detected': 'terrain/ramp_detected',
            'top_detected': 'terrain/top_detected',
            'ik_valid': 'ik_valid',
            'saturated': 'saturated',
            'workspace_limited': 'workspace_limited',
            'enabled': 'enabled',
            'motion_paused': 'motion/paused',
            'full_slope_recovered': 'motion/full_slope_recovered',
            'mixed_contact_escape_active': (
                'motion/mixed_contact_escape_active'
            ),
        }
        self.bool_publishers = {
            key: self.create_publisher(Bool, f'{prefix}/{topic}', 10)
            for key, topic in bool_topics.items()
        }
        string_topics = {
            'contact_phase': 'terrain/contact_phase',
            'limiting_leg': 'feasibility/limiting_leg',
            'limiting_joint': 'feasibility/limiting_joint',
            'limiting_reason': 'feasibility/limiting_reason',
            'stability_state': 'stability/state',
            'traction_state': 'traction/state',
            'safe_stop_reason': 'safe_stop_reason',
            'infeasible_reason': 'infeasible_reason',
            'mixed_contact_escape_reason': (
                'motion/mixed_contact_escape_reason'
            ),
        }
        self.string_publishers = {
            key: self.create_publisher(String, f'{prefix}/{topic}', 10)
            for key, topic in string_topics.items()
        }
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, f'{prefix}/diagnostics', 10,
        )
        self.joint_targets_publisher = self.create_publisher(
            JointState, f'{prefix}/joint_targets', 10,
        )

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response):
        if self.mode == ControllerMode.MONITOR:
            response.success = False
            response.message = 'monitor mode cannot start actuator sequencing'
        elif self.state != ClimbState.READY:
            response.success = False
            response.message = f'cannot start from {self.state.value}'
        else:
            self.start_requested = True
            response.success = True
            response.message = 'start accepted'
        return response

    def _request_fault(self, reason: str) -> None:
        if (
            not self.pending_fault_reason
            and self.state not in self.state_machine.TERMINAL
        ):
            self.pending_fault_reason = reason

    @staticmethod
    def _stamp_sec(message: Any) -> float:
        return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9

    def _on_imu(self, message: Imu) -> None:
        stamp = self._stamp_sec(message)
        q = message.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        frame_ok = not self.expected_imu_frame or (
            message.header.frame_id == self.expected_imu_frame
        )
        monotonic = (
            self.last_imu_stamp_sec is None
            or stamp >= self.last_imu_stamp_sec - 1.0e-9
        )
        values = (
            stamp, q.x, q.y, q.z, q.w,
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z, message.linear_acceleration.z,
        )
        if not (
            stamp > 0.0 and finite(*values) and abs(norm - 1.0) <= 0.02
            and frame_ok and monotonic and norm > 1.0e-9
        ):
            self.imu_valid = False
            if not monotonic:
                self._request_fault('simulation time moved backward in IMU')
            elif not frame_ok:
                self._request_fault(
                    f'unexpected IMU frame {message.header.frame_id!r}'
                )
            else:
                self._request_fault('invalid IMU quaternion or values')
            return
        roll, pitch, yaw = quaternion_to_rpy(
            q.x / norm, q.y / norm, q.z / norm, q.w / norm,
        )
        try:
            self.attitude_controller.update_imu(
                stamp, pitch, roll,
                float(message.angular_velocity.y),
                float(message.angular_velocity.x),
            )
        except ValueError as error:
            self.imu_valid = False
            self._request_fault(str(error))
            return
        self.raw_yaw = yaw
        self.vertical_accel_mps2 = float(message.linear_acceleration.z)
        self.last_imu_stamp_sec = stamp
        self.last_imu_wall_sec = time.monotonic()
        self.imu_valid = True

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(
            name in positions and math.isfinite(float(positions[name]))
            for name in LEG_JOINT_NAMES
        ):
            return
        self.latest_joints = {
            name: float(positions[name]) for name in LEG_JOINT_NAMES
        }
        velocities = dict(zip(message.name, message.velocity))
        if all(
            name in velocities and math.isfinite(float(velocities[name]))
            for name in WHEEL_JOINT_NAMES
        ):
            self.wheel_velocities_radps = tuple(
                float(velocities[name]) for name in WHEEL_JOINT_NAMES
            )
        self.last_joint_wall_sec = time.monotonic()

    @staticmethod
    def _normalize_quaternion(
        quaternion: Sequence[float],
    ) -> tuple[float, float, float, float] | None:
        if len(quaternion) != 4 or not finite(*quaternion):
            return None
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1.0e-9 or abs(norm - 1.0) > 0.02:
            return None
        return tuple(  # type: ignore[return-value]
            value / norm for value in quaternion
        )

    def _store_pose(
        self,
        source: str,
        stamp: float,
        position: Sequence[float],
        quaternion: Sequence[float],
        velocity: Sequence[float],
        yaw_rate: float,
    ) -> None:
        normalized = self._normalize_quaternion(quaternion)
        previous = self.pose_observations.get(source)
        monotonic = previous is None or stamp >= previous['stamp'] - 1.0e-9
        if not (
            stamp > 0.0 and normalized is not None and monotonic
            and finite(*position, *velocity, yaw_rate)
        ):
            if not monotonic:
                self._request_fault(
                    f'simulation time moved backward in {source} pose'
                )
            return
        _, _, yaw = quaternion_to_rpy(*normalized)
        self.pose_observations[source] = {
            'stamp': stamp,
            'wall': time.monotonic(),
            'position': tuple(float(value) for value in position),
            'velocity': tuple(float(value) for value in velocity),
            'yaw': yaw,
            'yaw_rate': float(yaw_rate),
        }

    def _on_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self._store_pose(
            'odometry', self._stamp_sec(message),
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ),
            (twist.linear.x, twist.linear.y, twist.linear.z),
            twist.angular.z,
        )

    def _on_pose_array(self, message: PoseArray) -> None:
        index = self.ground_truth_model_pose_index
        if index >= len(message.poses):
            return
        stamp = self._stamp_sec(message)
        pose = message.poses[index]
        normalized = self._normalize_quaternion((
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ))
        if normalized is None:
            self._request_fault('invalid fallback PoseArray quaternion')
            return
        _, _, yaw = quaternion_to_rpy(*normalized)
        velocity = (0.0, 0.0, 0.0)
        yaw_rate = 0.0
        if self.pose_array_previous is not None:
            old_stamp, old_x, old_y, old_z, old_yaw = self.pose_array_previous
            dt = stamp - old_stamp
            if dt < -1.0e-9:
                self._request_fault(
                    'simulation time moved backward in PoseArray'
                )
                return
            if 1.0e-5 <= dt <= 0.30:
                velocity = (
                    (pose.position.x - old_x) / dt,
                    (pose.position.y - old_y) / dt,
                    (pose.position.z - old_z) / dt,
                )
                yaw_rate = wrap_angle(yaw - old_yaw) / dt
        self.pose_array_previous = (
            stamp, pose.position.x, pose.position.y, pose.position.z, yaw,
        )
        self._store_pose(
            'pose_array_bridge', stamp,
            (pose.position.x, pose.position.y, pose.position.z),
            normalized, velocity, yaw_rate,
        )

    @staticmethod
    def _fresh(
        last_wall: float | None, timeout: float, now_wall: float,
    ) -> bool:
        return last_wall is not None and 0.0 <= now_wall - last_wall <= timeout

    def _select_pose(self, now_wall: float) -> None:
        def fresh(source: str) -> bool:
            item = self.pose_observations.get(source)
            return item is not None and self._fresh(
                item['wall'], self.pose_timeout_sec, now_wall,
            )

        selected = ''
        if self.pose_source_lock:
            selected = (
                self.pose_source_lock if fresh(self.pose_source_lock) else ''
            )
        elif fresh('odometry'):
            selected = 'odometry'
        elif fresh('pose_array_bridge'):
            selected = 'pose_array_bridge'
        if not selected:
            self.pose_valid = False
            self.pose_missing_since_wall = (
                self.pose_missing_since_wall or now_wall
            )
            return
        item = self.pose_observations[selected]
        self.pose_source = selected
        self.pose_stamp_sec = item['stamp']
        self.last_pose_wall_sec = item['wall']
        self.base_x, self.base_y, self.base_z = item['position']
        self.base_vx, self.base_vy, self.base_vz = item['velocity']
        self.base_yaw = item['yaw']
        self.base_yaw_rate = item['yaw_rate']
        self.pose_valid = True
        self.pose_missing_since_wall = None

    def _poll_controllers_and_ownership(self, now_wall: float) -> None:
        if (
            self.controller_future is not None
            and self.controller_future.done()
        ):
            try:
                result = self.controller_future.result()
                states = {item.name: item.state for item in result.controller}
                self.controllers_active = all(
                    states.get(name) == 'active'
                    for name in (
                        self.leg_controller_name, self.wheel_controller_name,
                    )
                )
                self.controller_status_known = True
            except Exception as error:  # ROS service transport error
                self.controllers_active = False
                self.controller_status_known = False
                self.get_logger().warning(
                    f'controller status query failed: {error}',
                    throttle_duration_sec=2.0,
                )
            self.controller_future = None
        if (
            self.controller_future is None
            and now_wall - self.last_controller_poll_wall_sec
            >= self.controller_poll_period_sec
            and self.controller_client.service_is_ready()
        ):
            self.controller_future = self.controller_client.call_async(
                ListControllers.Request()
            )
            self.last_controller_poll_wall_sec = now_wall
        if (
            now_wall - self.last_graph_poll_wall_sec
            >= self.controller_poll_period_sec
        ):
            self.command_topics_ready = bool(
                self.get_subscriptions_info_by_topic(self.leg_command_topic)
                and self.get_subscriptions_info_by_topic(
                    self.wheel_command_topic
                )
            )
            conflicts = (
                self._other_publishers(self.leg_command_topic)
                + self._other_publishers(self.wheel_command_topic)
            )
            self.controller_conflict = bool(conflicts)
            self.ownership_exclusive = not self.controller_conflict
            self.last_graph_poll_wall_sec = now_wall

    def _other_publishers(self, topic: str) -> list[Any]:
        own_name = self.get_name().lstrip('/')
        own_namespace = self.get_namespace().rstrip('/') or '/'
        return [
            info for info in self.get_publishers_info_by_topic(topic)
            if not (
                info.node_name.lstrip('/') == own_name
                and (info.node_namespace.rstrip('/') or '/') == own_namespace
            )
        ]

    def _start_pose_bridge(self) -> None:
        mapping = (
            f'{self.gazebo_pose_topic}'
            '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'
        )
        command = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', mapping,
            '--ros-args', '-r',
            f'{self.gazebo_pose_topic}:={self.pose_array_topic}',
        ]
        try:
            self.bridge_process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.bridge_start_attempts += 1
            self.get_logger().info(
                'started pose fallback bridge '
                f'attempt={self.bridge_start_attempts}'
            )
        except OSError as error:
            self._request_fault(f'failed to start pose bridge: {error}')

    def _manage_pose_bridge(self, now_wall: float) -> None:
        if not self.manage_pose_bridge:
            return
        if self.pose_source == 'odometry':
            self.stop_managed_bridge()
            return
        if (
            self.bridge_process is not None
            and self.bridge_process.poll() is not None
        ):
            self.bridge_process.wait()
            self.bridge_process = None
            if (
                self.run_start_ros_sec is None
                and self.bridge_start_attempts < 2
            ):
                self.bridge_next_start_wall_sec = now_wall + 0.50
                self.get_logger().warning(
                    'pose fallback bridge exited; restarting once'
                )
            else:
                self._request_fault('managed pose bridge exited unexpectedly')
        if (
            self.bridge_process is None
            and self.bridge_start_attempts < 2
            and now_wall >= self.bridge_next_start_wall_sec
            and 'odometry' not in self.pose_observations
        ):
            self._start_pose_bridge()

    def _data_ready(self, now_wall: float) -> bool:
        ownership_ok = (
            self.mode != ControllerMode.ACTIVE or self.ownership_exclusive
        )
        return all((
            self.imu_valid,
            self.pose_valid,
            bool(self.latest_joints),
            self._fresh(
                self.last_imu_wall_sec, self.imu_timeout_sec, now_wall,
            ),
            self._fresh(
                self.last_joint_wall_sec,
                self.joint_state_timeout_sec,
                now_wall,
            ),
            self._fresh(
                self.last_pose_wall_sec, self.pose_timeout_sec, now_wall,
            ),
            self.controller_status_known,
            self.controllers_active,
            self.command_topics_ready,
            ownership_ok,
        ))

    def _capture_current_state(self, now_ros: float) -> bool:
        try:
            feet, _ = self.feasibility_solver.capture_stance(
                self.latest_joints
            )
            envelope = self.feasibility_solver.envelope(
                now_ros, ContactPhase.FLAT_APPROACH, 0.0, force=True,
            )
        except ValueError as error:
            self.fault_reason = str(error)
            return False
        if not envelope.valid:
            self.fault_reason = (
                envelope.limiting_reason or 'invalid IK envelope'
            )
            return False
        self.nominal_feet = feet
        self.current_feet = dict(feet)
        self.feasibility = envelope
        self.initial_joint_margin_rad = envelope.joint_margin_rad
        self.captured_joints = [
            self.latest_joints[name] for name in LEG_JOINT_NAMES
        ]
        self.last_safe_leg_command = list(self.captured_joints)
        self.proposed_leg_command = list(self.captured_joints)
        self.initial_x = self.base_x
        self.initial_y = self.base_y
        self.initial_z = self.base_z
        self.initial_yaw = self.base_yaw
        self.heading_x = math.cos(self.initial_yaw)
        self.heading_y = math.sin(self.initial_yaw)
        self.terrain_estimator.reset(
            self.initial_x, self.initial_y, self.initial_z, self.initial_yaw,
        )
        self.start_pose = {
            'x_m': self.initial_x,
            'y_m': self.initial_y,
            'z_m': self.initial_z,
            'roll_deg': math.degrees(
                self.attitude_controller.filtered_roll_rad
            ),
            'pitch_deg': math.degrees(
                self.attitude_controller.filtered_pitch_rad
            ),
            'yaw_deg': math.degrees(self.initial_yaw),
        }
        self.get_logger().info(
            'captured current state: '
            f'pose=({self.initial_x:.3f},{self.initial_y:.3f},'
            f'{self.initial_z:.3f}) '
            f'yaw={math.degrees(self.initial_yaw):.3f} deg '
            'pitch='
            f'{math.degrees(self.attitude_controller.filtered_pitch_rad):.3f} '
            'deg'
        )
        self._log_feasibility_if_changed(envelope)
        return True

    def _log_feasibility_if_changed(
        self, envelope: FeasibilityResult,
    ) -> None:
        signature = (
            round(envelope.min_correction_deg, 1),
            round(envelope.max_correction_deg, 1),
            round(envelope.joint_margin_rad, 2),
            envelope.limiting_leg,
            envelope.limiting_joint,
            envelope.limiting_reason,
        )
        if signature == self.last_logged_feasibility:
            return
        self.last_logged_feasibility = signature
        self.get_logger().info(
            'feasibility envelope: '
            f'correction=[{envelope.min_correction_deg:.3f},'
            f'{envelope.max_correction_deg:.3f}] deg '
            f'joint_margin={envelope.joint_margin_rad:.3f} rad '
            f'limiter={envelope.limiting_leg or "cap"}/'
            f'{envelope.limiting_joint or "none"}'
        )

    def _transition(
        self, state: ClimbState, now_ros: float, reason: str = '',
    ) -> None:
        old = self.state
        if not self.state_machine.transition(state, now_ros):
            return
        suffix = f' ({reason})' if reason else ''
        self.get_logger().info(f'state: {old.value} -> {state.value}{suffix}')
        if state == ClimbState.STARTUP_SETTLE:
            self.run_start_ros_sec = now_ros
            self.run_start_wall_sec = time.monotonic()
            self.pose_source_lock = self.pose_source
            self.attitude_controller.reset_integral()
            self.motion_supervisor.reset()
            self.traction_supervisor.reset()
        elif state in (
            ClimbState.RAMP_APPROACH,
            ClimbState.MIXED_CONTACT_ENTRY,
            ClimbState.CLIMB,
        ):
            self.ramp_detected = True
        elif state in (ClimbState.TOP_TRANSITION, ClimbState.TOP_HOLD):
            self.top_detected = True
        elif state == ClimbState.COMPLETE:
            self.final_result = 'CLIMB_SUCCESS'
        elif state == ClimbState.SAFE_INFEASIBLE:
            self.final_result = 'SAFE_INFEASIBLE_STOP'
        elif state == ClimbState.FAULT:
            self.final_result = 'FAULT'
        elif state == ClimbState.TIMEOUT:
            self.final_result = 'TIMEOUT'

    def _enter_fault(self, now_ros: float, reason: str) -> None:
        if self.state in self.state_machine.TERMINAL:
            return
        self.fault_reason = reason
        self.safe_stop_reason = reason
        self._transition(ClimbState.FAULT, now_ros, reason)
        self.get_logger().error(f'fault: {reason}')

    def _enter_infeasible(self, now_ros: float, reason: str) -> None:
        if self.state in self.state_machine.TERMINAL:
            return
        self.infeasible_reason = reason
        self.safe_stop_reason = reason
        self._transition(ClimbState.SAFE_INFEASIBLE, now_ros, reason)
        self.get_logger().warning(f'safe infeasible stop: {reason}')

    def _update_terrain(self) -> None:
        if (
            not self.pose_valid or not self.terrain_estimator.initialized
            or self.pose_stamp_sec is None
            or self.pose_stamp_sec == self.last_terrain_pose_stamp
        ):
            return
        self.last_terrain_pose_stamp = self.pose_stamp_sec
        dynamic_vertical_accel = self.vertical_accel_mps2 - 9.80665 * math.cos(
            self.attitude_controller.filtered_pitch_rad
        )
        try:
            estimate = self.terrain_estimator.update(
                self.pose_stamp_sec, self.base_x, self.base_y, self.base_z,
                self.base_vz, self.attitude_controller.pitch_rate_radps,
                dynamic_vertical_accel,
            )
        except ValueError as error:
            self._request_fault(str(error))
            return
        self.terrain = estimate
        self.maximum_progress = max(
            self.maximum_progress, estimate.forward_progress_m,
        )
        self.maximum_height_gain = max(
            self.maximum_height_gain, estimate.height_gain_m,
        )
        self.maximum_estimated_slope_deg = max(
            self.maximum_estimated_slope_deg,
            estimate.fast_slope_deg,
            estimate.stable_slope_deg,
        )
        if estimate.contact_phase != self.last_contact_phase:
            self.get_logger().info(
                f'contact phase: {self.last_contact_phase.value} -> '
                f'{estimate.contact_phase.value}'
            )
            self.last_contact_phase = estimate.contact_phase

    def _update_observed_metrics(self) -> None:
        dx = self.base_x - self.initial_x
        dy = self.base_y - self.initial_y
        self.measured_forward_speed = (
            self.base_vx * self.heading_x + self.base_vy * self.heading_y
        )
        self.heading_error = wrap_angle(self.initial_yaw - self.base_yaw)
        if self.nominal_feet and self.latest_joints:
            try:
                feet, _ = self.feasibility_solver.observed_stance(
                    self.latest_joints, self.joint_limit_tolerance_rad,
                )
                self.current_feet = feet
            except ValueError as error:
                self._request_fault(str(error))
        if not self.terrain_estimator.initialized:
            self.maximum_progress = max(
                self.maximum_progress,
                dx * self.heading_x + dy * self.heading_y,
            )

    def _safety_check(self, now_ros: float, now_wall: float) -> None:
        if self.state in self.state_machine.TERMINAL:
            return
        if self.pending_fault_reason:
            reason = self.pending_fault_reason
            self.pending_fault_reason = ''
            self._enter_fault(now_ros, reason)
            return
        if self.state in (ClimbState.WAITING_FOR_DATA, ClimbState.READY):
            return
        if not self.imu_valid or not self._fresh(
            self.last_imu_wall_sec, self.imu_timeout_sec, now_wall,
        ):
            self._enter_fault(now_ros, 'stale IMU')
            return
        if not self._fresh(
            self.last_joint_wall_sec, self.joint_state_timeout_sec, now_wall,
        ):
            self._enter_fault(now_ros, 'stale joint states')
            return
        moving = max(
            self.motion_command.wheel_linear_mps, default=0.0,
        ) > 1.0e-3
        pose_grace = (
            self.pose_missing_since_wall is not None
            and now_wall - self.pose_missing_since_wall
            < self.pose_startup_grace_sec
            and not moving
            and self.state == ClimbState.STARTUP_SETTLE
        )
        if (
            not self.pose_valid
            or not self._fresh(
                self.last_pose_wall_sec, self.pose_timeout_sec, now_wall,
            )
        ) and not pose_grace:
            self._enter_fault(now_ros, 'stale base pose')
            return
        if not self.controllers_active:
            self._enter_fault(now_ros, 'required controller became inactive')
            return
        if not self.command_topics_ready:
            self._enter_fault(
                now_ros, 'controller command subscription disappeared',
            )
            return
        if self.mode == ControllerMode.ACTIVE and not self.ownership_exclusive:
            self._enter_fault(now_ros, 'competing command publisher detected')
            return
        roll_deg = abs(math.degrees(self.attitude_controller.raw_roll_rad))
        pitch_deg = abs(math.degrees(self.attitude_controller.raw_pitch_rad))
        if roll_deg > self.max_roll_deg:
            self._enter_fault(now_ros, 'unsafe roll angle')
            return
        if pitch_deg > self.max_pitch_deg:
            self._enter_fault(now_ros, 'unsafe pitch angle')
            return
        if (
            self.heading_hold_enabled
            and abs(math.degrees(self.heading_error))
            > self.maximum_heading_error_deg
        ):
            self._enter_fault(now_ros, 'excessive heading error')
            return
        if (
            self.state in DRIVE_STATES
            and self.maximum_progress - self.terrain.forward_progress_m
            > self.maximum_backward_progress_m
        ):
            self._enter_fault(now_ros, 'unexpected backward movement')

    def _evaluate_supervisors(self, now_ros: float) -> None:
        if not self.current_feet:
            return
        self.stability = self.stability_supervisor.evaluate(
            self.current_feet,
            self.attitude_controller.filtered_roll_rad,
            self.attitude_controller.filtered_pitch_rad,
            self.attitude_controller.roll_rate_radps,
            self.attitude_controller.pitch_rate_radps,
            self.terrain.contact_phase,
        )
        requested_for_slip = max(
            self.motion_command.effective_speed_mps,
            max(self.motion_command.wheel_linear_mps, default=0.0),
        )
        self.traction = self.traction_supervisor.evaluate(
            now_ros, self.wheel_velocities_radps,
            self.measured_forward_speed, requested_for_slip,
        )
        if self.stability.state != self.last_stability_state:
            self.get_logger().info(
                f'stability: {self.last_stability_state.value} -> '
                f'{self.stability.state.value} '
                f'margin={self.stability.minimum_margin_m:.3f} m'
            )
            self.last_stability_state = self.stability.state

    def _update_infeasibility(self, now_ros: float) -> None:
        if self.state not in DRIVE_STATES | {
            ClimbState.MIXED_CONTACT_EXIT,
            ClimbState.TOP_TRANSITION,
        }:
            self.stability_stop_since = None
            self.workspace_exhausted_since = None
            return
        escape_reason = self.motion_supervisor.mixed_contact_escape_reason
        if escape_reason.startswith('ESCAPE_ABORT_'):
            self._enter_infeasible(now_ros, escape_reason)
            return
        if self.traction.state == SafetyState.INFEASIBLE:
            self._enter_infeasible(now_ros, 'INFEASIBLE_TRACTION')
            return
        if self.stability.state == SafetyState.INFEASIBLE:
            self._enter_infeasible(now_ros, 'INFEASIBLE_STABILITY')
            return
        if self.motion_supervisor.mixed_contact_escape_active:
            self.stability_stop_since = None
        elif self.stability.state == SafetyState.STOP:
            self.stability_stop_since = self.stability_stop_since or now_ros
            if now_ros - self.stability_stop_since >= self.infeasible_hold_sec:
                self._enter_infeasible(now_ros, 'INFEASIBLE_STABILITY_MARGIN')
                return
        else:
            self.stability_stop_since = None
        if (
            self.terrain.valid
            and max(self.terrain.fast_slope_deg, self.terrain.stable_slope_deg)
            > self.maximum_uphill_slope_deg
        ):
            self._enter_infeasible(now_ros, 'SLOPE_ABOVE_CONFIGURED_RANGE')
            return
        if self.feasibility.workspace_usage >= 0.98:
            self.workspace_exhausted_since = (
                self.workspace_exhausted_since or now_ros
            )
            if (
                now_ros - self.workspace_exhausted_since
                >= self.infeasible_hold_sec
            ):
                self._enter_infeasible(now_ros, 'INFEASIBLE_WORKSPACE')
        else:
            self.workspace_exhausted_since = None

    def _update_state_machine(self, now_ros: float) -> None:
        if self.state in self.state_machine.TERMINAL:
            return
        old = self.state
        level = (
            abs(math.degrees(self.attitude_controller.filtered_pitch_rad))
            <= self.flat_start_pitch_tolerance_deg
            and abs(math.degrees(self.attitude_controller.filtered_roll_rad))
            <= self.flat_start_roll_tolerance_deg
        ) or self.allow_nonlevel_start
        wheels_stopped = max(
            (abs(value) for value in self.motion_command.wheel_linear_mps),
            default=0.0,
        ) <= 1.0e-4
        posture_neutral = max(
            abs(self.joint_command.front_offset_m),
            abs(self.joint_command.rear_offset_m),
            abs(self.joint_command.left_offset_m),
            abs(self.joint_command.right_offset_m),
        ) <= 5.0e-4
        new = self.state_machine.update(
            now_ros,
            self.terrain.contact_phase,
            start_requested=self.start_requested,
            level_start=level,
            wheels_stopped=wheels_stopped,
            posture_neutral=posture_neutral,
        )
        if new != old:
            # Rewind so the wrapper records ROS side effects exactly once.
            self.state_machine.state = old
            self._transition(new, now_ros)
            if new == ClimbState.FAULT and not level:
                self.fault_reason = 'robot did not start level on flat ground'
                self.safe_stop_reason = self.fault_reason

    def _update_control_modules(self, now_ros: float, dt: float) -> None:
        if not self.nominal_feet:
            return
        self.feasibility = self.feasibility_solver.envelope(
            now_ros, self.terrain.contact_phase,
            max(self.terrain.fast_slope_deg, self.terrain.stable_slope_deg),
        )
        if self.feasibility.recomputed:
            self._log_feasibility_if_changed(self.feasibility)
        if self.state not in self.state_machine.TERMINAL:
            self.attitude_command = self.attitude_controller.compute(
                self.terrain, self.feasibility, self.stability, self.state, dt,
            )
            result = self.feasibility_solver.solve_command(
                self.attitude_command.final_correction_deg,
                self.attitude_command.roll_correction_deg,
            )
            self.joint_command = result
            if result.valid:
                self.ik_failures = 0
                self.proposed_leg_command = list(result.positions)
                self.last_safe_leg_command = list(result.positions)
                capability = max(
                    abs(self.feasibility.min_correction_deg),
                    abs(self.feasibility.max_correction_deg), 1.0e-6,
                )
                usage = max(
                    abs(result.applied_pitch_correction_deg) / capability,
                    1.0 - result.scale,
                )
                self.feasibility = replace(
                    self.feasibility,
                    workspace_usage=clamp(usage, 0.0, 1.0),
                    joint_margin_rad=result.joint_margin_rad,
                )
            else:
                self.ik_failures += 1
                if self.ik_failures >= self.max_consecutive_ik_failures:
                    self._enter_fault(
                        now_ros, 'repeated analytical IK failures',
                    )
        drive_enabled = (
            self.state in DRIVE_STATES
            and self.pose_valid
            and self.joint_command.valid
        )
        self.motion_command = self.motion_supervisor.update(
            now_ros, dt, self.terrain,
            self.attitude_command.pitch_error_deg,
            self.stability, self.traction, self.feasibility,
            self.heading_error if self.heading_hold_enabled else 0.0,
            self.base_yaw_rate, drive_enabled,
        )
        self.safe_stop_reason = (
            self.infeasible_reason or self.fault_reason
            or self.motion_command.stop_reason
        )

    def _log_escape_transition(self) -> None:
        """Log each mixed-contact escape transition exactly once."""
        active = self.motion_supervisor.mixed_contact_escape_active
        reason = self.motion_supervisor.mixed_contact_escape_reason
        if active and not self.last_escape_active:
            escape_speed = min(
                self.mixed_contact_escape_speed_mps,
                self.commanded_speed_mps,
            )
            self.get_logger().warning(
                'mixed-contact escape activated: '
                f'margin={self.stability.minimum_margin_m:.6f} m '
                f'speed={escape_speed:.3f} m/s'
            )
        elif not active and self.last_escape_active:
            if reason.startswith('ESCAPE_SUCCESS_'):
                self.get_logger().info(
                    f'mixed-contact escape succeeded: {reason}'
                )
            elif reason.startswith('ESCAPE_ABORT_'):
                self.get_logger().warning(
                    f'mixed-contact escape aborted: {reason}'
                )
        elif reason == 'ESCAPE_REARMED' and reason != self.last_escape_reason:
            self.get_logger().info('mixed-contact escape rearmed')
        self.last_escape_active = active
        self.last_escape_reason = reason

    def _publish_actuators(self) -> None:
        if (
            self.mode != ControllerMode.ACTIVE
            or self.state not in COMMAND_STATES
            or self.leg_command_publisher is None
            or self.wheel_command_publisher is None
        ):
            return
        wheel_message = Float64MultiArray()
        wheel_message.data = [
            float(value) for value in self.motion_command.wheel_radps
        ]
        self.wheel_command_publisher.publish(wheel_message)
        if len(self.last_safe_leg_command) == 12:
            leg_message = Float64MultiArray()
            leg_message.data = [
                float(value) for value in self.last_safe_leg_command
            ]
            self.leg_command_publisher.publish(leg_message)

    def _publish_monitoring(self) -> None:
        state_message = String()
        state_message.data = self.state.value
        self.state_publisher.publish(state_message)
        attitude = self.attitude_command
        joint = self.joint_command
        terrain = self.terrain
        feasibility = self.feasibility
        stability = self.stability
        traction = self.traction
        motion = self.motion_command
        floats = {
            'raw_pitch': math.degrees(self.attitude_controller.raw_pitch_rad),
            'filtered_pitch': math.degrees(
                self.attitude_controller.filtered_pitch_rad
            ),
            'target_pitch': attitude.feasible_target_pitch_deg,
            'desired_level_pitch': attitude.desired_level_pitch_deg,
            'feasible_target_pitch': attitude.feasible_target_pitch_deg,
            'pitch_error': attitude.pitch_error_deg,
            'pitch_rate': self.attitude_controller.pitch_rate_radps,
            'pid_p': math.radians(attitude.p_term_deg),
            'pid_i': math.radians(attitude.i_term_deg),
            'pid_d': math.radians(attitude.d_term_deg),
            'pid_output': attitude.final_correction_deg,
            'feedforward': attitude.feedforward_deg,
            'feedback': attitude.feedback_deg,
            'final_correction': attitude.final_correction_deg,
            'raw_roll': math.degrees(self.attitude_controller.raw_roll_rad),
            'filtered_roll': math.degrees(
                self.attitude_controller.filtered_roll_rad
            ),
            'target_roll': attitude.desired_roll_deg,
            'roll_correction': attitude.roll_correction_deg,
            'front_offset': joint.front_offset_m,
            'rear_offset': joint.rear_offset_m,
            'terrain_slope': terrain.stable_slope_deg,
            'fast_slope': terrain.fast_slope_deg,
            'stable_slope': terrain.stable_slope_deg,
            'slope_confidence': terrain.slope_confidence,
            'slope_rate': terrain.slope_rate_degps,
            'height_gain': terrain.height_gain_m,
            'forward_progress': terrain.forward_progress_m,
            'max_positive': feasibility.max_correction_deg,
            'max_negative': feasibility.min_correction_deg,
            'joint_margin': feasibility.joint_margin_rad,
            'longitudinal_margin': stability.longitudinal_margin_m,
            'lateral_margin': stability.lateral_margin_m,
            'minimum_margin': stability.minimum_margin_m,
            'slip_fl': traction.slips[0],
            'slip_fr': traction.slips[1],
            'slip_rl': traction.slips[2],
            'slip_rr': traction.slips[3],
            'average_slip': traction.average_abs_slip,
            'requested_speed': motion.requested_speed_mps,
            'effective_speed': motion.effective_speed_mps,
            'commanded_speed': motion.effective_speed_mps,
            'measured_speed': self.measured_forward_speed,
            'slope_scale': motion.slope_scale,
            'transition_scale': motion.transition_scale,
            'pitch_scale': motion.pitch_scale,
            'stability_scale': motion.stability_scale,
            'traction_scale': motion.traction_scale,
            'workspace_scale': motion.workspace_scale,
            'recovery_floor': (
                motion.requested_speed_mps * self.full_slope_speed_scale
                if self.motion_supervisor.full_slope_recovered else 0.0
            ),
            'mixed_contact_escape_speed': (
                min(
                    self.mixed_contact_escape_speed_mps,
                    self.commanded_speed_mps,
                )
                if self.motion_supervisor.mixed_contact_escape_active
                else 0.0
            ),
            'mixed_contact_escape_elapsed': (
                self.motion_supervisor.mixed_contact_escape_elapsed_sec
            ),
            'mixed_contact_escape_margin': stability.minimum_margin_m,
        }
        for key, value in floats.items():
            if math.isfinite(value):
                message = Float64()
                message.data = float(value)
                self.float_publishers[key].publish(message)
        bools = {
            'ramp_detected': self.ramp_detected,
            'top_detected': self.top_detected,
            'ik_valid': joint.valid,
            'saturated': attitude.saturated,
            'workspace_limited': feasibility.workspace_usage >= 0.95,
            'enabled': (
                self.mode == ControllerMode.ACTIVE
                and self.state in COMMAND_STATES
            ),
            'motion_paused': motion.paused,
            'full_slope_recovered': (
                self.motion_supervisor.full_slope_recovered
            ),
            'mixed_contact_escape_active': (
                self.motion_supervisor.mixed_contact_escape_active
            ),
        }
        for key, value in bools.items():
            message = Bool()
            message.data = bool(value)
            self.bool_publishers[key].publish(message)
        strings = {
            'contact_phase': terrain.contact_phase.value,
            'limiting_leg': feasibility.limiting_leg,
            'limiting_joint': feasibility.limiting_joint,
            'limiting_reason': feasibility.limiting_reason,
            'stability_state': stability.state.value,
            'traction_state': traction.state.value,
            'safe_stop_reason': self.safe_stop_reason,
            'infeasible_reason': self.infeasible_reason,
            'mixed_contact_escape_reason': (
                self.motion_supervisor.mixed_contact_escape_reason
            ),
        }
        for key, value in strings.items():
            message = String()
            message.data = value
            self.string_publishers[key].publish(message)
        if (
            len(self.proposed_leg_command) == 12
            and finite(*self.proposed_leg_command)
        ):
            targets = JointState()
            targets.header.stamp = self.get_clock().now().to_msg()
            targets.name = list(LEG_JOINT_NAMES)
            targets.position = list(self.proposed_leg_command)
            self.joint_targets_publisher.publish(targets)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'go2w_dynamic_imu_ik_climb'
        status.hardware_id = 'go2w'
        status.level = (
            DiagnosticStatus.ERROR
            if self.state in (ClimbState.FAULT, ClimbState.TIMEOUT)
            else DiagnosticStatus.WARN
            if self.state in (
                ClimbState.WAITING_FOR_DATA,
                ClimbState.SAFE_INFEASIBLE,
            )
            else DiagnosticStatus.OK
        )
        status.message = self.safe_stop_reason or self.state.value
        status.values = [
            KeyValue(key='mode', value=self.mode.value),
            KeyValue(key='pose_source', value=self.pose_source or 'waiting'),
            KeyValue(key='contact_phase', value=terrain.contact_phase.value),
            KeyValue(key='stability_state', value=stability.state.value),
            KeyValue(key='traction_state', value=traction.state.value),
            KeyValue(key='run_id', value=self.logger.run_id),
        ]
        diagnostics.status = [status]
        self.diagnostics_publisher.publish(diagnostics)

    def _elapsed(self, now_ros: float) -> float:
        if self.run_start_ros_sec is None:
            return 0.0
        return max(0.0, now_ros - self.run_start_ros_sec)

    def _log_cycle(self, now_ros: float, now_wall: float) -> None:
        attitude = self.attitude_command
        joint = self.joint_command
        terrain = self.terrain
        feasibility = self.feasibility
        stability = self.stability
        traction = self.traction
        motion = self.motion_command
        row: dict[str, Any] = {
            'ros_time_sec': now_ros,
            'elapsed_time_sec': self._elapsed(now_ros),
            'state': self.state.value,
            'contact_phase': terrain.contact_phase.value,
            'expected_slope_deg': self.expected_slope_deg,
            'estimated_slope_deg': terrain.stable_slope_deg,
            'slope_estimate_valid': terrain.valid,
            'raw_slope_deg': terrain.raw_slope_deg,
            'fast_slope_deg': terrain.fast_slope_deg,
            'stable_slope_deg': terrain.stable_slope_deg,
            'slope_confidence': terrain.slope_confidence,
            'slope_rate_degps': terrain.slope_rate_degps,
            'base_x_m': self.base_x if self.pose_valid else '',
            'base_y_m': self.base_y if self.pose_valid else '',
            'base_z_m': self.base_z if self.pose_valid else '',
            'height_gain_m': terrain.height_gain_m,
            'forward_progress_m': terrain.forward_progress_m,
            'base_vx_mps': self.base_vx,
            'base_vy_mps': self.base_vy,
            'base_vz_mps': self.base_vz,
            'measured_forward_speed_mps': self.measured_forward_speed,
            'commanded_speed_mps': motion.effective_speed_mps,
            'requested_speed_mps': motion.requested_speed_mps,
            'effective_speed_mps': motion.effective_speed_mps,
            'raw_roll_deg': math.degrees(
                self.attitude_controller.raw_roll_rad
            ),
            'filtered_roll_deg': math.degrees(
                self.attitude_controller.filtered_roll_rad
            ),
            'desired_roll_deg': attitude.desired_roll_deg,
            'roll_correction_deg': attitude.roll_correction_deg,
            'raw_pitch_deg': math.degrees(
                self.attitude_controller.raw_pitch_rad
            ),
            'filtered_pitch_deg': math.degrees(
                self.attitude_controller.filtered_pitch_rad
            ),
            'target_pitch_deg': attitude.feasible_target_pitch_deg,
            'desired_level_pitch_deg': attitude.desired_level_pitch_deg,
            'feasible_target_pitch_deg': attitude.feasible_target_pitch_deg,
            'pitch_error_deg': attitude.pitch_error_deg,
            'pitch_rate_radps': self.attitude_controller.pitch_rate_radps,
            'pid_p_rad': math.radians(attitude.p_term_deg),
            'pid_i_rad': math.radians(attitude.i_term_deg),
            'pid_d_rad': math.radians(attitude.d_term_deg),
            'pid_output_deg': attitude.final_correction_deg,
            'feedforward_deg': attitude.feedforward_deg,
            'feedback_deg': attitude.feedback_deg,
            'final_correction_deg': attitude.final_correction_deg,
            'front_offset_m': joint.front_offset_m,
            'rear_offset_m': joint.rear_offset_m,
            'left_offset_m': joint.left_offset_m,
            'right_offset_m': joint.right_offset_m,
            'max_positive_correction_deg': feasibility.max_correction_deg,
            'max_negative_correction_deg': feasibility.min_correction_deg,
            'feasibility_joint_margin_rad': feasibility.joint_margin_rad,
            'limiting_leg': feasibility.limiting_leg,
            'limiting_joint': feasibility.limiting_joint,
            'limiting_reason': feasibility.limiting_reason,
            'workspace_usage': feasibility.workspace_usage,
            'joint_limit_usage': clamp(
                1.0 - feasibility.joint_margin_rad
                / max(self.initial_joint_margin_rad, 1.0e-6),
                0.0,
                1.0,
            ),
            'longitudinal_margin_m': stability.longitudinal_margin_m,
            'lateral_margin_m': stability.lateral_margin_m,
            'minimum_stability_margin_m': stability.minimum_margin_m,
            'stability_state': stability.state.value,
            'slip_fl': traction.slips[0],
            'slip_fr': traction.slips[1],
            'slip_rl': traction.slips[2],
            'slip_rr': traction.slips[3],
            'average_abs_slip': traction.average_abs_slip,
            'peak_abs_slip': traction.peak_abs_slip,
            'traction_state': traction.state.value,
            'slope_scale': motion.slope_scale,
            'transition_scale': motion.transition_scale,
            'pitch_scale': motion.pitch_scale,
            'stability_scale': motion.stability_scale,
            'traction_scale': motion.traction_scale,
            'workspace_scale': motion.workspace_scale,
            'motion_paused': motion.paused,
            'motion_full_slope_recovered': (
                self.motion_supervisor.full_slope_recovered
            ),
            'motion_recovery_floor_mps': (
                motion.requested_speed_mps * self.full_slope_speed_scale
                if self.motion_supervisor.full_slope_recovered else 0.0
            ),
            'motion_mixed_contact_escape_active': (
                self.motion_supervisor.mixed_contact_escape_active
            ),
            'motion_mixed_contact_escape_speed_mps': (
                min(
                    self.mixed_contact_escape_speed_mps,
                    self.commanded_speed_mps,
                )
                if self.motion_supervisor.mixed_contact_escape_active
                else 0.0
            ),
            'motion_mixed_contact_escape_elapsed_sec': (
                self.motion_supervisor.mixed_contact_escape_elapsed_sec
            ),
            'motion_mixed_contact_escape_margin_m': (
                stability.minimum_margin_m
                if math.isfinite(stability.minimum_margin_m) else ''
            ),
            'motion_mixed_contact_escape_reason': (
                self.motion_supervisor.mixed_contact_escape_reason
            ),
            'ik_valid': joint.valid,
            'saturated': attitude.saturated,
            'workspace_limited': feasibility.workspace_usage >= 0.95,
            'ramp_detected': self.ramp_detected,
            'top_detected': self.top_detected,
            'controller_conflict': self.controller_conflict,
            'safe_stop_reason': self.safe_stop_reason,
            'infeasible_reason': self.infeasible_reason,
            'final_result': self.final_result,
        }
        for index, label in enumerate(('fl', 'fr', 'rl', 'rr')):
            row[f'wheel_cmd_{label}_radps'] = motion.wheel_radps[index]
        current = [self.latest_joints.get(name) for name in LEG_JOINT_NAMES]
        commanded = (
            self.last_safe_leg_command
            if len(self.last_safe_leg_command) == 12 else [None] * 12
        )
        for name, current_value, command_value in zip(
            LEG_JOINT_NAMES, current, commanded,
        ):
            row[f'current_{name}_rad'] = (
                '' if current_value is None else current_value
            )
            row[f'commanded_{name}_rad'] = (
                '' if command_value is None else command_value
            )
            row[f'error_{name}_rad'] = (
                '' if current_value is None or command_value is None
                else command_value - current_value
            )
        self.logger.write(row, now_wall)

    def _terminal_ready_to_finalize(self) -> bool:
        return max(
            (abs(value) for value in self.motion_command.wheel_linear_mps),
            default=0.0,
        ) <= 1.0e-4

    def _finalize(self, result: str) -> None:
        if self.finalized:
            return
        physically_feasible = result == 'CLIMB_SUCCESS'
        summary = {
            'mode': self.mode.value,
            'expected_slope_deg': self.expected_slope_deg,
            'maximum_estimated_slope_deg': self.maximum_estimated_slope_deg,
            'start_pose': self.start_pose,
            'heading_yaw_deg': math.degrees(self.initial_yaw),
            'climb_result': result,
            'physically_feasible': physically_feasible,
            'limiting_factor': (
                self.infeasible_reason or self.fault_reason
                or self.feasibility.limiting_reason
            ),
            'final_feasible_target_pitch_deg': (
                self.attitude_command.feasible_target_pitch_deg
            ),
            'maximum_feasible_positive_correction_deg': (
                self.feasibility.max_correction_deg
            ),
            'maximum_feasible_negative_correction_deg': (
                self.feasibility.min_correction_deg
            ),
            'distance_m': self.maximum_progress,
            'height_gain_m': self.maximum_height_gain,
            'duration_sec': self._elapsed(self._ros_now_sec()),
            'ramp_detected': self.ramp_detected,
            'top_detected': self.top_detected,
            'top_result': (
                'TOP_REACHED' if self.top_detected else 'NOT_REACHED'
            ),
            'no_topple_result': self.state != ClimbState.FAULT or (
                'unsafe roll' not in self.fault_reason
                and 'unsafe pitch' not in self.fault_reason
            ),
            'wheel_stop_confirmed': self._terminal_ready_to_finalize(),
            'pose_source': self.pose_source or None,
            'safe_stop_reason': self.safe_stop_reason,
            'infeasible_reason': self.infeasible_reason,
            'fault_reason': self.fault_reason,
            'final_result': result,
        }
        self.logger.finalize(summary)
        self.finalized = True
        self.get_logger().info(
            f'run finalized: result={result} csv={self.logger.csv_path} '
            f'json={self.logger.summary_path}'
        )

    def _status_line(self, now_wall: float) -> None:
        if now_wall - self.last_status_wall_sec < 1.0:
            return
        self.last_status_wall_sec = now_wall
        escape_active = str(
            self.motion_supervisor.mixed_contact_escape_active
        ).lower()
        self.get_logger().info(
            f'status state={self.state.value} '
            f'phase={self.terrain.contact_phase.value} '
            'pitch='
            f'{math.degrees(self.attitude_controller.filtered_pitch_rad):.3f} '
            'deg '
            'target='
            f'{self.attitude_command.feasible_target_pitch_deg:.3f} deg '
            f'slope={self.terrain.stable_slope_deg:.3f} deg '
            f'margin={self.stability.minimum_margin_m:.3f} m '
            f'slip={self.traction.average_abs_slip:.3f} '
            f'speed={self.motion_command.effective_speed_mps:.3f} m/s '
            f'paused={str(self.motion_command.paused).lower()} '
            'recovered='
            f'{str(self.motion_supervisor.full_slope_recovered).lower()} '
            'escape='
            f'{escape_active} '
            f'escape_margin={self.stability.minimum_margin_m:.6f} '
            'escape_elapsed='
            f'{self.motion_supervisor.mixed_contact_escape_elapsed_sec:.3f}'
        )

    def _ros_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _control_cycle(self) -> None:
        now_wall = time.monotonic()
        wall_dt = clamp(now_wall - self.last_control_wall_sec, 0.0, 0.05)
        self.last_control_wall_sec = now_wall
        now_ros = self._ros_now_sec()
        ros_dt = (
            None if self.last_ros_time_sec is None
            else now_ros - self.last_ros_time_sec
        )
        if ros_dt is not None and ros_dt < -1.0e-9:
            self._request_fault('simulation clock moved backward')
        dt = (
            wall_dt if self.state in self.state_machine.TERMINAL
            else clamp(ros_dt or 0.0, 0.0, 0.05)
        )
        self.last_ros_time_sec = now_ros

        self._manage_pose_bridge(now_wall)
        self._poll_controllers_and_ownership(now_wall)
        self._select_pose(now_wall)
        if self.state == ClimbState.WAITING_FOR_DATA:
            if self.pending_fault_reason:
                reason = self.pending_fault_reason
                self.pending_fault_reason = ''
                self._enter_fault(now_ros, reason)
            elif self._data_ready(now_wall):
                if self._capture_current_state(now_ros):
                    self._transition(ClimbState.READY, now_ros)
                else:
                    self._enter_fault(
                        now_ros,
                        self.fault_reason
                        or 'current stance is invalid for FK/IK',
                    )
            elif (
                now_wall - self.node_start_wall_sec
                > self.data_wait_timeout_sec
            ):
                missing = (
                    'reliable pose source' if not self.pose_valid
                    else 'required data/controllers'
                )
                self._enter_fault(now_ros, f'timed out waiting for {missing}')
        else:
            self._update_terrain()
            self._update_observed_metrics()
            self._evaluate_supervisors(now_ros)
            self._safety_check(now_ros, now_wall)
            if self.state not in self.state_machine.TERMINAL:
                self._update_state_machine(now_ros)
                self._update_control_modules(now_ros, dt)
                self._update_infeasibility(now_ros)
            else:
                self.motion_command = self.motion_supervisor.update(
                    now_ros, dt, self.terrain,
                    self.attitude_command.pitch_error_deg,
                    self.stability, self.traction, self.feasibility,
                    0.0, self.base_yaw_rate, False,
                )
        self._log_escape_transition()
        self._publish_actuators()
        self._publish_monitoring()
        self._log_cycle(now_ros, now_wall)
        self._status_line(now_wall)

        if (
            self.state in self.state_machine.TERMINAL
            and self._terminal_ready_to_finalize()
        ):
            self._finalize(self.final_result or self.state.value)
            if self.state == ClimbState.COMPLETE and self.shutdown_after_top:
                self.stop_requested = True

    def stop_managed_bridge(self) -> None:
        """Terminate and reap the managed fallback bridge process."""
        process = self.bridge_process
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=1.0)
            except ProcessLookupError:
                pass
        else:
            process.wait()
        self.bridge_process = None

    def shutdown_safely(self, result: str = 'INTERRUPTED') -> None:
        """Decelerate wheels, hold posture, finalize logs, and clean up."""
        self.control_timer.cancel()
        if (
            self.mode == ControllerMode.ACTIVE
            and self.wheel_command_publisher is not None
        ):
            deadline = time.monotonic() + 0.60
            wheel_linear = list(self.motion_command.wheel_linear_mps)
            previous = time.monotonic()
            while time.monotonic() < deadline and rclpy.ok():
                current = time.monotonic()
                dt = clamp(current - previous, 0.005, 0.03)
                previous = current
                wheel_linear = [
                    rate_limit(
                        value, 0.0, self.wheel_decel_limit_mps2, dt,
                    )
                    for value in wheel_linear
                ]
                wheel_radps = tuple(
                    value / self.wheel_radius_m for value in wheel_linear
                )
                self.motion_command = replace(
                    self.motion_command,
                    effective_speed_mps=0.0,
                    wheel_linear_mps=tuple(wheel_linear),
                    wheel_radps=wheel_radps,
                )
                self._publish_actuators()
                if max(
                    (abs(value) for value in wheel_linear), default=0.0,
                ) <= 1.0e-4:
                    break
                time.sleep(0.01)
            self.motion_command = replace(
                self.motion_command,
                effective_speed_mps=0.0,
                wheel_linear_mps=(0.0, 0.0, 0.0, 0.0),
                wheel_radps=(0.0, 0.0, 0.0, 0.0),
            )
            for _ in range(5):
                self._publish_actuators()
                time.sleep(0.01)
        if not self.finalized:
            self.final_result = self.final_result or result
            self._log_cycle(self._ros_now_sec(), time.monotonic())
            self._finalize(self.final_result)
        self.stop_managed_bridge()


def main(args=None) -> None:
    """Run the ROS node with explicit safe signal handling."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2WDynamicImuIkClimb()

    def request_stop(_signum, _frame) -> None:
        node.stop_requested = True

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.10)
    finally:
        node.shutdown_safely()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
