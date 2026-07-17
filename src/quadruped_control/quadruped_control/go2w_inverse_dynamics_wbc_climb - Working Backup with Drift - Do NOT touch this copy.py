#!/usr/bin/env python3
"""Contact-aware inverse-dynamics WBC for the Unitree Go2-W ramp world.

The node always computes a rigid-body inverse-dynamics solution in ``shadow``
or ``active`` mode.  With the repository's stock ros2_control configuration,
``active`` uses a deliberately labelled position/velocity-backed WBC
approximation; it does not claim to apply the optimized torques.
"""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, fields
from datetime import datetime, timezone
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
import osqp
import pinocchio as pin
from scipy import sparse

# Make package imports work when this source file is executed directly.
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from controller_manager_msgs.srv import (  # noqa: E402
    ListControllers,
    ListHardwareInterfaces,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from geometry_msgs.msg import PoseArray, Vector3  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt32  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from quadruped_control.go2w_dynamic_models import (  # noqa: E402
    ContactPhase,
    TerrainEstimate,
)
from quadruped_control.go2w_stability_supervisor import (  # noqa: E402
    TractionConfig,
    TractionSupervisor,
)
from quadruped_control.go2w_terrain_estimator import (  # noqa: E402
    TerrainEstimator,
    TerrainEstimatorConfig,
)


LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in LEGS
    for joint in ('hip', 'thigh', 'calf')
)
WHEEL_JOINT_NAMES = tuple(f'{leg}_foot_joint' for leg in LEGS)
ACTUATED_JOINT_NAMES = tuple(
    name
    for leg in LEGS
    for name in (
        f'{leg}_hip_joint', f'{leg}_thigh_joint',
        f'{leg}_calf_joint', f'{leg}_foot_joint',
    )
)
MAX_BOUND = 1.0e20
GRAVITY = 9.81


def finite(*values: Any) -> bool:
    """Return true when every scalar/array value is finite."""
    return all(np.all(np.isfinite(value)) for value in values)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp one finite scalar."""
    return max(lower, min(upper, value))


def rate_limit(current: float, target: float, limit_per_sec: float, dt: float) -> float:
    """Apply a symmetric first-order rate limit."""
    step = max(0.0, limit_per_sec) * max(0.0, dt)
    return current + clamp(target - current, -step, step)


def quaternion_to_rpy(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Convert normalized ROS xyzw quaternion to roll, pitch, yaw."""
    x, y, z, w = quaternion
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


def normalized_quaternion(values: Sequence[float]) -> np.ndarray | None:
    """Normalize a near-unit xyzw quaternion, rejecting corrupt samples."""
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not finite(q):
        return None
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-9 or abs(norm - 1.0) > 0.02:
        return None
    return q / norm


@dataclass(frozen=True)
class WbcConfig:
    """Validated ROS parameters for the complete controller."""

    mode: str = 'shadow'
    backend: str = 'shadow'
    auto_start: bool = False
    control_rate_hz: float = 100.0
    commanded_speed_mps: float = 0.08
    desired_level_pitch_deg: float = 0.0
    maximum_slope_deg: float = 45.0
    friction_mu_longitudinal: float = 1.2
    friction_mu_lateral: float = 1.0
    normal_force_min_n: float = 1.0
    normal_force_max_n: float = 180.0
    leg_torque_scale: float = 0.80
    wheel_torque_scale: float = 0.75
    torque_rate_limit_nmps: float = 250.0
    max_joint_accel_radps2: float = 35.0
    max_wheel_accel_radps2: float = 25.0
    contact_kp: float = 40.0
    contact_kd: float = 10.0
    com_kp: float = 18.0
    com_kd: float = 7.0
    orientation_kp: float = 45.0
    orientation_kd: float = 10.0
    posture_kp: float = 20.0
    posture_kd: float = 5.0
    rolling_constraint_weight: float = 35.0
    base_orientation_weight: float = 900.0
    com_weight: float = 650.0
    speed_weight: float = 100.0
    posture_weight: float = 18.0
    force_regularization_weight: float = 2.0e-3
    torque_regularization_weight: float = 8.0e-4
    qdd_regularization_weight: float = 2.0e-4
    solver_eps_abs: float = 2.0e-4
    solver_eps_rel: float = 2.0e-4
    solver_max_iter: int = 1200
    solver_time_limit_sec: float = 0.0075
    maximum_consecutive_solver_failures: int = 5
    state_timeout_sec: float = 0.35
    pose_timeout_sec: float = 0.35
    imu_timeout_sec: float = 0.25
    joint_timeout_sec: float = 0.25
    output_directory: str = '/home/svm/quad_ws/logs/inverse_dynamics_wbc'
    run_id: str = ''
    shutdown_after_top: bool = False
    urdf_path: str = ''
    ros2_control_xacro_path: str = ''
    imu_topic: str = '/imu/data'
    expected_imu_frame: str = 'imu'
    joint_state_topic: str = '/joint_states'
    odometry_topic: str = '/go2w/ground_truth/odom'
    gazebo_pose_topic: str = '/world/go2w_slope_test/dynamic_pose/info'
    pose_array_topic: str = '/go2w/wbc/pose_array'
    ground_truth_model_pose_index: int = 0
    manage_pose_bridge: bool = True
    leg_command_topic: str = '/go2w_leg_position_controller/commands'
    wheel_command_topic: str = '/go2w_wheel_velocity_controller/commands'
    effort_command_topic: str = '/go2w_effort_controller/commands'
    leg_controller_name: str = 'go2w_leg_position_controller'
    wheel_controller_name: str = 'go2w_wheel_velocity_controller'
    effort_controller_name: str = 'go2w_effort_controller'
    controller_manager_name: str = '/controller_manager'
    controller_poll_period_sec: float = 0.25
    max_leg_position_step_rad: float = 0.020
    virtual_leg_stiffness_nm_rad: float = 100.0
    joint_limit_margin_rad: float = 0.025
    velocity_lookahead_sec: float = 0.050
    contact_force_rate_limit_nps: float = 1400.0
    maximum_wheel_speed_radps: float = 10.0
    maximum_com_accel_mps2: float = 2.5
    maximum_roll_deg: float = 35.0
    maximum_pitch_deg: float = 65.0
    maximum_slip_ratio: float = 0.65
    maximum_backward_progress_m: float = 0.10
    maximum_runtime_sec: float = 180.0
    minimum_stability_margin_m: float = 0.0
    mixed_contact_minimum_speed_mps: float = 0.040
    mixed_contact_escape_max_duration_sec: float = 40.0
    mixed_contact_progress_timeout_sec: float = 5.0
    mixed_contact_min_progress_m: float = 0.010
    traction_slip_hold_sec: float = 4.50
    traction_infeasible_hold_sec: float = 7.00
    wheel_torque_velocity_gain_radps_per_nm: float = 0.35
    maximum_torque_velocity_correction_radps: float = 0.60
    maximum_deadline_misses: int = 20
    startup_settle_sec: float = 1.0
    status_period_sec: float = 1.0
    log_flush_period_sec: float = 1.0
    pose_velocity_filter_alpha: float = 0.05
    slip_filter_alpha: float = 0.05

    @classmethod
    def from_node(cls, node: Node) -> 'WbcConfig':
        """Declare every parameter and construct the immutable config."""
        defaults = cls()
        values: dict[str, Any] = {}
        for field in fields(cls):
            default = getattr(defaults, field.name)
            node.declare_parameter(field.name, default)
            values[field.name] = node.get_parameter(field.name).value
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        """Reject unsafe or internally inconsistent settings."""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{field.name} must be finite')
        if self.mode not in {'monitor', 'shadow', 'active'}:
            raise ValueError('mode must be monitor, shadow, or active')
        if self.backend not in {
            'shadow', 'hybrid_position_velocity', 'effort_wbc',
        }:
            raise ValueError(
                'backend must be shadow, hybrid_position_velocity, or effort_wbc'
            )
        if self.mode == 'active' and self.backend == 'shadow':
            raise ValueError('active mode requires an actuating backend')
        positive = {
            'control_rate_hz': self.control_rate_hz,
            'maximum_slope_deg': self.maximum_slope_deg,
            'friction_mu_longitudinal': self.friction_mu_longitudinal,
            'friction_mu_lateral': self.friction_mu_lateral,
            'normal_force_max_n': self.normal_force_max_n,
            'max_joint_accel_radps2': self.max_joint_accel_radps2,
            'max_wheel_accel_radps2': self.max_wheel_accel_radps2,
            'solver_eps_abs': self.solver_eps_abs,
            'solver_eps_rel': self.solver_eps_rel,
            'solver_time_limit_sec': self.solver_time_limit_sec,
            'state_timeout_sec': self.state_timeout_sec,
            'pose_timeout_sec': self.pose_timeout_sec,
            'imu_timeout_sec': self.imu_timeout_sec,
            'joint_timeout_sec': self.joint_timeout_sec,
            'virtual_leg_stiffness_nm_rad': self.virtual_leg_stiffness_nm_rad,
            'velocity_lookahead_sec': self.velocity_lookahead_sec,
            'maximum_wheel_speed_radps': self.maximum_wheel_speed_radps,
            'maximum_runtime_sec': self.maximum_runtime_sec,
            'startup_settle_sec': self.startup_settle_sec,
            'mixed_contact_escape_max_duration_sec': (
                self.mixed_contact_escape_max_duration_sec
            ),
            'mixed_contact_progress_timeout_sec': (
                self.mixed_contact_progress_timeout_sec
            ),
            'traction_slip_hold_sec': self.traction_slip_hold_sec,
            'traction_infeasible_hold_sec': self.traction_infeasible_hold_sec,
            'wheel_torque_velocity_gain_radps_per_nm': (
                self.wheel_torque_velocity_gain_radps_per_nm
            ),
            'maximum_torque_velocity_correction_radps': (
                self.maximum_torque_velocity_correction_radps
            ),
        }
        if not all(math.isfinite(v) and v > 0.0 for v in positive.values()):
            bad = [name for name, value in positive.items()
                   if not math.isfinite(value) or value <= 0.0]
            raise ValueError(f'parameters must be finite and positive: {bad}')
        nonnegative = {
            'commanded_speed_mps': self.commanded_speed_mps,
            'normal_force_min_n': self.normal_force_min_n,
            'torque_rate_limit_nmps': self.torque_rate_limit_nmps,
            'minimum_stability_margin_m': self.minimum_stability_margin_m,
            'mixed_contact_minimum_speed_mps': self.mixed_contact_minimum_speed_mps,
            'mixed_contact_min_progress_m': self.mixed_contact_min_progress_m,
        }
        if not all(math.isfinite(v) and v >= 0.0 for v in nonnegative.values()):
            raise ValueError('nonnegative parameters contain an invalid value')
        gains = (
            self.contact_kp, self.contact_kd, self.com_kp, self.com_kd,
            self.orientation_kp, self.orientation_kd, self.posture_kp,
            self.posture_kd, self.rolling_constraint_weight,
            self.base_orientation_weight, self.com_weight, self.speed_weight,
            self.posture_weight, self.force_regularization_weight,
            self.torque_regularization_weight, self.qdd_regularization_weight,
        )
        if not all(math.isfinite(v) and v >= 0.0 for v in gains):
            raise ValueError('gains and QP weights must be finite and nonnegative')
        if self.normal_force_min_n >= self.normal_force_max_n:
            raise ValueError('normal force minimum must be below maximum')
        if not 0.0 < self.leg_torque_scale <= 1.0:
            raise ValueError('leg_torque_scale must be in (0, 1]')
        if not 0.0 < self.wheel_torque_scale <= 1.0:
            raise ValueError('wheel_torque_scale must be in (0, 1]')
        if not 0.0 < self.maximum_slip_ratio <= 2.0:
            raise ValueError('maximum_slip_ratio must be in (0, 2]')
        if not 0.0 < self.pose_velocity_filter_alpha <= 1.0:
            raise ValueError('pose_velocity_filter_alpha must be in (0, 1]')
        if not 0.0 < self.slip_filter_alpha <= 1.0:
            raise ValueError('slip_filter_alpha must be in (0, 1]')
        if not 10 <= self.solver_max_iter <= 100000:
            raise ValueError('solver_max_iter is outside [10, 100000]')
        if self.maximum_consecutive_solver_failures < 1:
            raise ValueError('maximum_consecutive_solver_failures must be positive')
        if self.maximum_deadline_misses < 1:
            raise ValueError('maximum_deadline_misses must be positive')
        if self.ground_truth_model_pose_index < 0:
            raise ValueError('ground_truth_model_pose_index must be nonnegative')
        required_strings = {
            'output_directory': self.output_directory,
            'imu_topic': self.imu_topic,
            'joint_state_topic': self.joint_state_topic,
            'odometry_topic': self.odometry_topic,
            'leg_command_topic': self.leg_command_topic,
            'wheel_command_topic': self.wheel_command_topic,
            'controller_manager_name': self.controller_manager_name,
        }
        if any(not value.strip() for value in required_strings.values()):
            raise ValueError('required path/topic/controller parameters cannot be empty')


@dataclass(frozen=True)
class EstimatedState:
    """One name-mapped, freshness-checked floating-base sample."""

    stamp_sec: float
    q: np.ndarray
    v: np.ndarray
    joint_efforts: np.ndarray
    base_position: np.ndarray
    base_quaternion: np.ndarray
    base_linear_velocity_body: np.ndarray
    base_angular_velocity_body: np.ndarray
    pose_source: str


@dataclass(frozen=True)
class WheelKinematics:
    """World-aligned wheel-center kinematics from Pinocchio."""

    center: np.ndarray
    rotation: np.ndarray
    jacobian: np.ndarray
    jdot_v: np.ndarray
    axis_world: np.ndarray
    omega_radps: float


@dataclass(frozen=True)
class DynamicsSnapshot:
    """Finite rigid-body quantities needed by the QP."""

    M: np.ndarray
    h: np.ndarray
    gravity: np.ndarray
    com_position: np.ndarray
    com_velocity: np.ndarray
    com_drift_acceleration: np.ndarray
    com_jacobian: np.ndarray
    base_rotation: np.ndarray
    wheels: Mapping[str, WheelKinematics]


class RobotModel:
    """Authoritative free-flyer Pinocchio model and exact joint mapping."""

    def __init__(self, urdf_path: str) -> None:
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f'authoritative URDF not found: {self.urdf_path}')
        self.xml_root = ET.parse(self.urdf_path).getroot()
        self.model = pin.buildModelFromUrdf(
            str(self.urdf_path), pin.JointModelFreeFlyer(),
        )
        self.data = self.model.createData()
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.joint_ids: dict[str, int] = {}
        self.q_indices: dict[str, int] = {}
        self.v_indices: dict[str, int] = {}
        for name in ACTUATED_JOINT_NAMES:
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise ValueError(f'URDF is missing actuated joint {name}')
            joint = self.model.joints[joint_id]
            if joint.nv != 1 or joint.nq not in (1, 2):
                raise ValueError(f'{name} is not a supported one-DoF joint')
            if name in WHEEL_JOINT_NAMES and joint.nq != 2:
                raise ValueError(f'{name} must be represented as a continuous joint')
            if name in LEG_JOINT_NAMES and joint.nq != 1:
                raise ValueError(f'{name} must be represented as a bounded revolute joint')
            self.joint_ids[name] = joint_id
            self.q_indices[name] = joint.idx_q
            self.v_indices[name] = joint.idx_v
        self.actuated_v_indices = np.asarray(
            [self.v_indices[name] for name in ACTUATED_JOINT_NAMES], dtype=int,
        )
        self.actuated_q_indices = np.asarray(
            [self.q_indices[name] for name in ACTUATED_JOINT_NAMES], dtype=int,
        )
        self.frame_ids = {
            leg: self.model.getFrameId(f'{leg}_foot') for leg in LEGS
        }
        if any(frame_id >= self.model.nframes for frame_id in self.frame_ids.values()):
            raise ValueError('URDF is missing one or more wheel-center frames')
        self.wheel_radius = self._wheel_radius_from_urdf()
        self.wheel_axes = self._wheel_axes_from_urdf()
        self.mass = float(pin.computeTotalMass(self.model))
        self.position_lower = np.asarray([
            self.model.lowerPositionLimit[self.q_indices[name]]
            for name in LEG_JOINT_NAMES
        ])
        self.position_upper = np.asarray([
            self.model.upperPositionLimit[self.q_indices[name]]
            for name in LEG_JOINT_NAMES
        ])
        self.velocity_limits = np.asarray([
            self.model.velocityLimit[self.v_indices[name]]
            for name in ACTUATED_JOINT_NAMES
        ])
        self.effort_limits = np.asarray([
            self.model.effortLimit[self.v_indices[name]]
            for name in ACTUATED_JOINT_NAMES
        ])
        if not finite(
            self.mass, self.wheel_radius, self.position_lower,
            self.position_upper, self.velocity_limits, self.effort_limits,
        ) or self.mass <= 0.0 or self.wheel_radius <= 0.0:
            raise ValueError('URDF contains non-finite or non-physical limits')
        self.nominal_q = pin.neutral(self.model)
        for leg in LEGS:
            for joint, value in (
                ('hip', 0.0), ('thigh', 0.5), ('calf', -1.4),
            ):
                self.nominal_q[self.q_indices[f'{leg}_{joint}_joint']] = value

    def _wheel_radius_from_urdf(self) -> float:
        radii: list[float] = []
        for leg in LEGS:
            link = self.xml_root.find(f"./link[@name='{leg}_foot']")
            if link is None:
                raise ValueError(f'wheel link {leg}_foot is absent')
            cylinder = link.find('./collision/geometry/cylinder')
            if cylinder is None or cylinder.get('radius') is None:
                raise ValueError(f'wheel collision radius missing for {leg}')
            radii.append(float(cylinder.get('radius', 'nan')))
        if max(radii) - min(radii) > 1.0e-9:
            raise ValueError(f'inconsistent wheel radii: {radii}')
        return radii[0]

    def _wheel_axes_from_urdf(self) -> dict[str, np.ndarray]:
        axes: dict[str, np.ndarray] = {}
        for leg in LEGS:
            joint = self.xml_root.find(f"./joint[@name='{leg}_foot_joint']")
            axis = None if joint is None else joint.find('axis')
            if axis is None:
                raise ValueError(f'wheel axis missing for {leg}')
            values = np.fromstring(axis.get('xyz', ''), sep=' ', dtype=float)
            if values.shape != (3,) or np.linalg.norm(values) <= 1.0e-9:
                raise ValueError(f'invalid wheel axis for {leg}')
            axes[leg] = values / np.linalg.norm(values)
        return axes

    def make_configuration(
        self,
        base_position: Sequence[float],
        base_quaternion: Sequence[float],
        joint_positions: Mapping[str, float],
    ) -> np.ndarray:
        """Build q by exact name rather than JointState ordering."""
        q = self.nominal_q.copy()
        q[:3] = np.asarray(base_position, dtype=float)
        quaternion = normalized_quaternion(base_quaternion)
        if quaternion is None:
            raise ValueError('INVALID_QUATERNION')
        q[3:7] = quaternion
        for name in ACTUATED_JOINT_NAMES:
            value = float(joint_positions[name])
            q_index = self.q_indices[name]
            if name in WHEEL_JOINT_NAMES:
                q[q_index:q_index + 2] = (math.cos(value), math.sin(value))
            else:
                q[q_index] = value
        if not finite(q):
            raise ValueError('NONFINITE_CONFIGURATION')
        return q

    def make_velocity(
        self,
        linear_body: Sequence[float],
        angular_body: Sequence[float],
        joint_velocities: Mapping[str, float],
    ) -> np.ndarray:
        """Build Pinocchio's local free-flyer velocity by exact name."""
        v = np.zeros(self.nv)
        v[:3] = np.asarray(linear_body, dtype=float)
        v[3:6] = np.asarray(angular_body, dtype=float)
        for name in ACTUATED_JOINT_NAMES:
            v[self.v_indices[name]] = float(joint_velocities[name])
        if not finite(v):
            raise ValueError('NONFINITE_VELOCITY')
        return v

    def compute(self, state: EstimatedState) -> DynamicsSnapshot:
        """Compute all dynamics and kinematics once for one control cycle."""
        q, v = state.q, state.v
        if q.shape != (self.nq,) or v.shape != (self.nv,) or not finite(q, v):
            raise ValueError('INVALID_MODEL_STATE')
        pin.computeAllTerms(self.model, self.data, q, v)
        pin.forwardKinematics(self.model, self.data, q, v, np.zeros(self.nv))
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        M = np.asarray(pin.crba(self.model, self.data, q), dtype=float)
        M = 0.5 * (M + M.T)
        h = np.asarray(pin.nonLinearEffects(self.model, self.data, q, v), dtype=float)
        gravity = np.asarray(pin.computeGeneralizedGravity(
            self.model, self.data, q,
        ), dtype=float)
        pin.centerOfMass(self.model, self.data, q, v, np.zeros(self.nv))
        com_position = np.asarray(self.data.com[0], dtype=float).copy()
        com_velocity = np.asarray(self.data.vcom[0], dtype=float).copy()
        com_drift = np.asarray(self.data.acom[0], dtype=float).copy()
        com_jacobian = np.asarray(pin.jacobianCenterOfMass(
            self.model, self.data, q, False,
        ), dtype=float)
        base_rotation = pin.Quaternion(q[6], q[3], q[4], q[5]).matrix()
        wheels: dict[str, WheelKinematics] = {}
        for leg in LEGS:
            frame_id = self.frame_ids[leg]
            placement = self.data.oMf[frame_id]
            jacobian = np.asarray(pin.getFrameJacobian(
                self.model, self.data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ), dtype=float)
            jacobian_dot = np.asarray(pin.getFrameJacobianTimeVariation(
                self.model, self.data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ), dtype=float)
            joint_id = self.joint_ids[f'{leg}_foot_joint']
            axis_world = self.data.oMi[joint_id].rotation @ self.wheel_axes[leg]
            wheels[leg] = WheelKinematics(
                center=np.asarray(placement.translation, dtype=float).copy(),
                rotation=np.asarray(placement.rotation, dtype=float).copy(),
                jacobian=jacobian,
                jdot_v=jacobian_dot @ v,
                axis_world=axis_world,
                omega_radps=float(v[self.v_indices[f'{leg}_foot_joint']]),
            )
        if not finite(
            M, h, gravity, com_position, com_velocity, com_drift,
            com_jacobian, base_rotation,
        ):
            raise ValueError('NONFINITE_DYNAMICS')
        if np.linalg.eigvalsh(M)[0] <= 1.0e-9:
            raise ValueError('NON_POSITIVE_DEFINITE_MASS_MATRIX')
        return DynamicsSnapshot(
            M=M, h=h, gravity=gravity, com_position=com_position,
            com_velocity=com_velocity,
            com_drift_acceleration=com_drift,
            com_jacobian=com_jacobian, base_rotation=base_rotation,
            wheels=wheels,
        )


class StateEstimator:
    """Freshness-aware IMU, odometry, pose-fallback, and joint fusion."""

    def __init__(self, robot: RobotModel, config: WbcConfig) -> None:
        self.robot = robot
        self.config = config
        self.imu_quaternion: np.ndarray | None = None
        self.imu_angular_velocity = np.zeros(3)
        self.imu_linear_acceleration = np.zeros(3)
        self.imu_stamp_sec: float | None = None
        self.imu_wall_sec: float | None = None
        self.joint_positions: dict[str, float] = {}
        self.joint_velocities: dict[str, float] = {}
        self.joint_efforts: dict[str, float] = {}
        self.joint_stamp_sec: float | None = None
        self.joint_wall_sec: float | None = None
        self.poses: dict[str, dict[str, Any]] = {}
        self.pose_array_previous: tuple[float, np.ndarray, float] | None = None
        self.last_error = ''

    @staticmethod
    def message_stamp(message: Any) -> float:
        return float(message.header.stamp.sec) + 1.0e-9 * float(
            message.header.stamp.nanosec
        )

    def on_imu(self, message: Imu) -> None:
        stamp = self.message_stamp(message)
        q = normalized_quaternion((
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w,
        ))
        angular = np.asarray((
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z,
        ), dtype=float)
        linear = np.asarray((
            message.linear_acceleration.x, message.linear_acceleration.y,
            message.linear_acceleration.z,
        ), dtype=float)
        monotonic = self.imu_stamp_sec is None or stamp >= self.imu_stamp_sec - 1e-9
        frame_valid = (
            not self.config.expected_imu_frame
            or message.header.frame_id == self.config.expected_imu_frame
        )
        if (
            stamp <= 0.0 or q is None or not finite(angular, linear)
            or not monotonic or not frame_valid
        ):
            self.last_error = (
                'TIME_REVERSAL_IMU' if not monotonic
                else 'UNEXPECTED_IMU_FRAME' if not frame_valid
                else 'INVALID_IMU'
            )
            return
        self.imu_quaternion = q
        self.imu_angular_velocity = angular
        self.imu_linear_acceleration = linear
        self.imu_stamp_sec = stamp
        self.imu_wall_sec = time.monotonic()

    def on_joint_state(self, message: JointState) -> None:
        position = dict(zip(message.name, message.position))
        velocity = dict(zip(message.name, message.velocity))
        effort = dict(zip(message.name, message.effort))
        if not all(
            name in position and name in velocity
            and finite(float(position[name]), float(velocity[name]))
            for name in ACTUATED_JOINT_NAMES
        ):
            self.last_error = 'INCOMPLETE_JOINT_STATE'
            return
        stamp = self.message_stamp(message)
        if self.joint_stamp_sec is not None and stamp > 0.0:
            if stamp < self.joint_stamp_sec - 1.0e-9:
                self.last_error = 'TIME_REVERSAL_JOINTS'
                return
        self.joint_positions = {
            name: float(position[name]) for name in ACTUATED_JOINT_NAMES
        }
        self.joint_velocities = {
            name: float(velocity[name]) for name in ACTUATED_JOINT_NAMES
        }
        self.joint_efforts = {
            name: float(effort.get(name, 0.0))
            if finite(float(effort.get(name, 0.0))) else 0.0
            for name in ACTUATED_JOINT_NAMES
        }
        self.joint_stamp_sec = stamp if stamp > 0.0 else self.joint_stamp_sec
        self.joint_wall_sec = time.monotonic()

    def _store_pose(
        self,
        source: str,
        stamp: float,
        position: Sequence[float],
        quaternion: Sequence[float],
        velocity: Sequence[float],
        velocity_is_body: bool,
    ) -> None:
        q = normalized_quaternion(quaternion)
        p = np.asarray(position, dtype=float)
        vel = np.asarray(velocity, dtype=float)
        previous = self.poses.get(source)
        monotonic = previous is None or stamp >= previous['stamp'] - 1.0e-9
        if stamp <= 0.0 or q is None or not finite(p, vel) or not monotonic:
            self.last_error = 'TIME_REVERSAL_POSE' if not monotonic else 'INVALID_POSE'
            return
        self.poses[source] = {
            'stamp': stamp, 'wall': time.monotonic(), 'position': p,
            'quaternion': q, 'velocity': vel,
            'velocity_is_body': velocity_is_body,
        }

    def on_odometry(self, message: Odometry) -> None:
        pose, twist = message.pose.pose, message.twist.twist
        self._store_pose(
            'odometry', self.message_stamp(message),
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w),
            (twist.linear.x, twist.linear.y, twist.linear.z), True,
        )

    def on_pose_array(self, message: PoseArray) -> None:
        index = self.config.ground_truth_model_pose_index
        if index < 0 or index >= len(message.poses):
            return
        stamp = self.message_stamp(message)
        pose = message.poses[index]
        position = np.asarray((
            pose.position.x, pose.position.y, pose.position.z,
        ), dtype=float)
        q = normalized_quaternion((
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ))
        if q is None:
            self.last_error = 'INVALID_FALLBACK_QUATERNION'
            return
        velocity = np.zeros(3)
        if self.pose_array_previous is not None:
            old_stamp, old_position, _ = self.pose_array_previous
            dt = stamp - old_stamp
            if dt < -1.0e-9:
                self.last_error = 'TIME_REVERSAL_POSE_ARRAY'
                return
            if 1.0e-5 <= dt <= 0.30:
                velocity = (position - old_position) / dt
                previous_pose = self.poses.get('pose_array_bridge')
                if previous_pose is not None:
                    alpha = self.config.pose_velocity_filter_alpha
                    velocity = (
                        alpha * velocity
                        + (1.0 - alpha) * previous_pose['velocity']
                    )
        self.pose_array_previous = (stamp, position.copy(), 0.0)
        self._store_pose(
            'pose_array_bridge', stamp, position, q, velocity, False,
        )

    @staticmethod
    def _fresh(stamp: float | None, timeout: float, now: float) -> bool:
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def selected_pose(self, now_wall: float) -> tuple[str, dict[str, Any]] | None:
        for source in ('odometry', 'pose_array_bridge'):
            item = self.poses.get(source)
            if item is not None and self._fresh(
                item['wall'], self.config.pose_timeout_sec, now_wall,
            ):
                return source, item
        return None

    def readiness_reason(self, now_wall: float) -> str:
        if self.last_error.startswith('TIME_REVERSAL'):
            return self.last_error
        if self.imu_quaternion is None or not self._fresh(
            self.imu_wall_sec, self.config.imu_timeout_sec, now_wall,
        ):
            return 'STALE_IMU'
        if not self.joint_positions or not self._fresh(
            self.joint_wall_sec, self.config.joint_timeout_sec, now_wall,
        ):
            return 'STALE_JOINTS'
        if self.selected_pose(now_wall) is None:
            return 'STALE_POSE'
        return ''

    def snapshot(self, now_wall: float) -> EstimatedState:
        reason = self.readiness_reason(now_wall)
        if reason:
            raise ValueError(reason)
        selected = self.selected_pose(now_wall)
        assert selected is not None and self.imu_quaternion is not None
        source, pose = selected
        rotation = pin.Quaternion(
            self.imu_quaternion[3], self.imu_quaternion[0],
            self.imu_quaternion[1], self.imu_quaternion[2],
        ).matrix()
        linear = pose['velocity'].copy()
        if not pose['velocity_is_body']:
            linear = rotation.T @ linear
        q = self.robot.make_configuration(
            pose['position'], self.imu_quaternion, self.joint_positions,
        )
        v = self.robot.make_velocity(
            linear, self.imu_angular_velocity, self.joint_velocities,
        )
        stamp = max(
            float(self.imu_stamp_sec or 0.0),
            float(self.joint_stamp_sec or 0.0), float(pose['stamp']),
        )
        efforts = np.asarray([
            self.joint_efforts.get(name, 0.0)
            for name in ACTUATED_JOINT_NAMES
        ])
        return EstimatedState(
            stamp_sec=stamp, q=q, v=v, joint_efforts=efforts,
            base_position=pose['position'].copy(),
            base_quaternion=self.imu_quaternion.copy(),
            base_linear_velocity_body=linear,
            base_angular_velocity_body=self.imu_angular_velocity.copy(),
            pose_source=source,
        )


@dataclass(frozen=True)
class ContactFrame:
    """One wheel's local terrain basis and contact geometry."""

    leg: str
    tangent: np.ndarray
    lateral: np.ndarray
    normal: np.ndarray
    basis: np.ndarray
    center: np.ndarray
    point: np.ndarray
    center_jacobian: np.ndarray
    center_jdot_v: np.ndarray
    point_jacobian: np.ndarray
    penetration_error_m: float
    rolling_sign: float


class ContactEstimator:
    """Reuse terrain phases and assign a distinct plane to each axle."""

    _ENTRY = {
        ContactPhase.ENTRY_DETECTED,
        ContactPhase.FRONT_AXLE_ON_RAMP,
        ContactPhase.MIXED_CONTACT_ENTRY,
    }
    _EXIT = {
        ContactPhase.CREST_DETECTED,
        ContactPhase.FRONT_AXLE_ON_TOP,
        ContactPhase.MIXED_CONTACT_EXIT,
    }

    def __init__(self, config: WbcConfig, wheel_radius: float) -> None:
        self.config = config
        terrain_config = TerrainEstimatorConfig(
            maximum_slope_deg=max(55.0, config.maximum_slope_deg + 5.0),
        )
        self.terrain_estimator = TerrainEstimator(terrain_config)
        self.traction = TractionSupervisor(TractionConfig(
            wheel_radius_m=wheel_radius,
            slip_epsilon_mps=0.005,
            caution_threshold=min(0.45, 0.65 * config.maximum_slip_ratio),
            stop_threshold=config.maximum_slip_ratio,
            slip_hold_sec=config.traction_slip_hold_sec,
            infeasible_hold_sec=config.traction_infeasible_hold_sec,
        ))
        self.wheel_radius = wheel_radius
        self.terrain = TerrainEstimate()
        self.last_pose_stamp: float | None = None
        self.origin_position = np.zeros(3)
        self.heading = np.asarray((1.0, 0.0, 0.0))
        self.filtered_slips = np.zeros(4)
        self.full_slope_reference_deg = 0.0

    def reset(self, state: EstimatedState) -> None:
        _, _, yaw = quaternion_to_rpy(state.base_quaternion)
        self.terrain_estimator.reset(
            *state.base_position, yaw,
        )
        self.traction.reset()
        self.terrain = TerrainEstimate()
        self.last_pose_stamp = None
        self.origin_position = state.base_position.copy()
        self.heading = np.asarray((math.cos(yaw), math.sin(yaw), 0.0))
        self.filtered_slips[:] = 0.0
        self.full_slope_reference_deg = 0.0

    def update_terrain(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        imu_linear_acceleration_body: Sequence[float] | None = None,
    ) -> TerrainEstimate:
        if not self.terrain_estimator.initialized:
            self.reset(state)
        if self.last_pose_stamp is not None and state.stamp_sec <= self.last_pose_stamp:
            return self.terrain
        roll, pitch, _ = quaternion_to_rpy(state.base_quaternion)
        del roll
        vertical_world = (
            pin.Quaternion(
                state.base_quaternion[3], state.base_quaternion[0],
                state.base_quaternion[1], state.base_quaternion[2],
            ).matrix() @ state.base_linear_velocity_body
        )[2]
        del pitch
        rotation = pin.Quaternion(
            state.base_quaternion[3], state.base_quaternion[0],
            state.base_quaternion[1], state.base_quaternion[2],
        ).matrix()
        dynamic_vertical_accel = 0.0
        if imu_linear_acceleration_body is not None:
            world_specific_force = rotation @ np.asarray(
                imu_linear_acceleration_body, dtype=float,
            )
            dynamic_vertical_accel = float(world_specific_force[2] - GRAVITY)
        raw = self.terrain_estimator.update(
            state.stamp_sec, *state.base_position,
            vertical_speed_mps=float(vertical_world),
            pitch_rate_radps=float(state.base_angular_velocity_body[1]),
            vertical_accel_mps2=dynamic_vertical_accel,
        )
        self.last_pose_stamp = state.stamp_sec
        front_center = 0.5 * (
            dynamics.wheels['FL'].center + dynamics.wheels['FR'].center
        )
        rear_center = 0.5 * (
            dynamics.wheels['RL'].center + dynamics.wheels['RR'].center
        )
        separation = front_center - rear_center
        forward_separation = float(separation @ self.heading)
        kinematic_slope = math.degrees(math.atan2(
            float(separation[2]), max(0.05, forward_separation),
        ))
        kinematic_slope = clamp(kinematic_slope, 0.0, 89.0)
        progress = float((state.base_position - self.origin_position) @ self.heading)
        height_gain = float(state.base_position[2] - self.origin_position[2])
        entry_threshold = self.terrain_estimator.config.entry_slope_deg
        if (
            raw.contact_phase == ContactPhase.FULL_SLOPE
            and kinematic_slope >= entry_threshold
            and (
                self.full_slope_reference_deg <= 0.0
                or kinematic_slope >= 0.75 * self.full_slope_reference_deg
            )
        ):
            if self.full_slope_reference_deg <= 0.0:
                self.full_slope_reference_deg = kinematic_slope
            else:
                self.full_slope_reference_deg = (
                    0.98 * self.full_slope_reference_deg
                    + 0.02 * kinematic_slope
                )
        if kinematic_slope < entry_threshold:
            previous_phase = self.terrain.contact_phase
            crest_context = (
                previous_phase == ContactPhase.FULL_SLOPE
                or previous_phase in self._EXIT
                or previous_phase == ContactPhase.TOP_PLATFORM
            ) and height_gain > 0.10
            if crest_context:
                exit_phase = raw.contact_phase
                if exit_phase not in self._EXIT and exit_phase != ContactPhase.TOP_PLATFORM:
                    exit_phase = ContactPhase.CREST_DETECTED
                exit_slope = (
                    0.0
                    if exit_phase == ContactPhase.TOP_PLATFORM
                    else self.full_slope_reference_deg
                )
                if exit_slope <= 0.0:
                    exit_slope = max(
                        self.terrain.fast_slope_deg,
                        self.terrain.stable_slope_deg,
                    )
                self.terrain = TerrainEstimate(
                    forward_progress_m=progress,
                    height_gain_m=height_gain,
                    raw_slope_deg=raw.raw_slope_deg,
                    fast_slope_deg=exit_slope,
                    stable_slope_deg=exit_slope,
                    slope_confidence=max(raw.slope_confidence, 0.75),
                    slope_rate_degps=raw.slope_rate_degps,
                    transition_probability=max(
                        raw.transition_probability, 0.75,
                    ),
                    contact_phase=exit_phase,
                    valid=True,
                    stale=False,
                )
                return self.terrain
            # Body-height transients on flat terrain can look like a steep
            # dz/dx ramp. Wheel-center geometry rejects that false positive.
            if (
                raw.contact_phase != ContactPhase.FLAT_APPROACH
                or max(raw.fast_slope_deg, raw.stable_slope_deg)
                >= entry_threshold
            ):
                _, _, yaw = quaternion_to_rpy(state.base_quaternion)
                self.terrain_estimator.reset(
                    *state.base_position, yaw,
                )
            self.terrain = TerrainEstimate(
                forward_progress_m=progress,
                height_gain_m=height_gain,
                raw_slope_deg=kinematic_slope,
                fast_slope_deg=kinematic_slope,
                stable_slope_deg=kinematic_slope,
                slope_confidence=1.0,
                contact_phase=ContactPhase.FLAT_APPROACH,
                valid=True,
                stale=False,
            )
            return self.terrain
        raw_slope = max(raw.fast_slope_deg, raw.stable_slope_deg)
        fused_slope = max(
            kinematic_slope,
            min(raw_slope, kinematic_slope + 5.0),
        )
        self.terrain = TerrainEstimate(
            forward_progress_m=progress,
            height_gain_m=height_gain,
            raw_slope_deg=raw.raw_slope_deg,
            fast_slope_deg=fused_slope,
            stable_slope_deg=fused_slope,
            slope_confidence=max(raw.slope_confidence, 0.75),
            slope_rate_degps=raw.slope_rate_degps,
            transition_probability=raw.transition_probability,
            contact_phase=raw.contact_phase,
            valid=True,
            stale=False,
        )
        return self.terrain

    @staticmethod
    def contact_basis(slope_rad: float, yaw_rad: float) -> np.ndarray:
        """Return [uphill tangent, lateral, upward normal] in world axes."""
        heading = np.asarray((math.cos(yaw_rad), math.sin(yaw_rad), 0.0))
        lateral = np.asarray((-math.sin(yaw_rad), math.cos(yaw_rad), 0.0))
        tangent = math.cos(slope_rad) * heading + np.asarray((
            0.0, 0.0, math.sin(slope_rad),
        ))
        normal = np.cross(tangent, lateral)
        tangent /= np.linalg.norm(tangent)
        lateral /= np.linalg.norm(lateral)
        normal /= np.linalg.norm(normal)
        basis = np.column_stack((tangent, lateral, normal))
        if not finite(basis) or np.linalg.det(basis) < 0.99:
            raise ValueError('INVALID_CONTACT_FRAME')
        return basis

    def slope_for_leg(self, leg: str, terrain: TerrainEstimate) -> float:
        """Assign front/rear normals correctly during mixed contacts."""
        slope = math.radians(clamp(
            max(terrain.fast_slope_deg, terrain.stable_slope_deg),
            0.0, 89.0,
        ))
        phase = terrain.contact_phase
        if phase in self._ENTRY:
            return slope if leg.startswith('F') else 0.0
        if phase == ContactPhase.FULL_SLOPE:
            return slope
        if phase in self._EXIT:
            return 0.0 if leg.startswith('F') else slope
        return 0.0

    def frames(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        terrain: TerrainEstimate,
    ) -> dict[str, ContactFrame]:
        _, _, yaw = quaternion_to_rpy(state.base_quaternion)
        result: dict[str, ContactFrame] = {}
        for leg in LEGS:
            wheel = dynamics.wheels[leg]
            terrain_basis = self.contact_basis(
                self.slope_for_leg(leg, terrain), yaw,
            )
            nominal_tangent, nominal_lateral, normal = terrain_basis.T
            lateral = wheel.axis_world - float(
                wheel.axis_world @ normal
            ) * normal
            lateral_norm = float(np.linalg.norm(lateral))
            if lateral_norm < 0.50:
                raise ValueError(f'WHEEL_AXIS_DEGENERATE_{leg}')
            lateral /= lateral_norm
            if float(lateral @ nominal_lateral) < 0.0:
                lateral *= -1.0
            tangent = np.cross(lateral, normal)
            tangent /= np.linalg.norm(tangent)
            if float(tangent @ nominal_tangent) < 0.0:
                tangent *= -1.0
                lateral *= -1.0
            basis = np.column_stack((tangent, lateral, normal))
            offset = -self.wheel_radius * normal
            point = wheel.center + offset
            linear_jacobian = wheel.jacobian[:3, :]
            angular_jacobian = wheel.jacobian[3:, :]
            point_jacobian = linear_jacobian - pin.skew(offset) @ angular_jacobian
            axis_sign = float(np.dot(wheel.axis_world, lateral))
            if abs(axis_sign) < 0.50:
                raise ValueError(f'WHEEL_AXIS_MISALIGNED_{leg}')
            result[leg] = ContactFrame(
                leg=leg, tangent=tangent, lateral=lateral, normal=normal,
                basis=basis, center=wheel.center, point=point,
                center_jacobian=linear_jacobian,
                center_jdot_v=wheel.jdot_v[:3],
                point_jacobian=point_jacobian,
                penetration_error_m=0.0,
                rolling_sign=math.copysign(1.0, axis_sign),
            )
        return result

    def slip_ratios(
        self,
        state: EstimatedState,
        frames: Mapping[str, ContactFrame],
        dynamics: DynamicsSnapshot,
        requested_speed: float,
    ) -> tuple[float, float, float, float]:
        speeds: list[float] = []
        for leg in LEGS:
            wheel = dynamics.wheels[leg]
            center_velocity = wheel.jacobian[:3, :] @ state.v
            center_long = float(frames[leg].tangent @ center_velocity)
            surface = (
                self.wheel_radius * frames[leg].rolling_sign
                * wheel.omega_radps
            )
            denominator = max(abs(surface), abs(center_long), 0.025)
            speeds.append((surface - center_long) / denominator)
        if abs(requested_speed) < 0.01:
            return (0.0, 0.0, 0.0, 0.0)
        alpha = self.config.slip_filter_alpha
        self.filtered_slips = (
            alpha * np.asarray(speeds)
            + (1.0 - alpha) * self.filtered_slips
        )
        return tuple(self.filtered_slips)  # type: ignore[return-value]


@dataclass(frozen=True)
class PostureTarget:
    """Continuous slope-aware pitch and articulated COM targets."""

    desired_pitch_rad: float
    feasible_pitch_rad: float
    desired_yaw_rad: float
    desired_com_position: np.ndarray
    desired_com_acceleration: np.ndarray
    longitudinal_margin_m: float
    lateral_margin_m: float


class SlopePosturePlanner:
    """Plan continuous uphill lean and conservative full-body COM motion."""

    def __init__(self, config: WbcConfig) -> None:
        self.config = config
        self.reference_yaw_rad = 0.0
        self.reference_lateral_m: float | None = None

    def reset(self, state: EstimatedState) -> None:
        """Latch the initial lane heading and lateral center for this run."""
        _, _, self.reference_yaw_rad = quaternion_to_rpy(
            state.base_quaternion,
        )
        lateral = np.asarray((
            -math.sin(self.reference_yaw_rad),
            math.cos(self.reference_yaw_rad),
            0.0,
        ))
        self.reference_lateral_m = float(state.base_position @ lateral)

    @staticmethod
    def stability_margins(
        com: np.ndarray,
        contacts: Mapping[str, ContactFrame],
        yaw: float,
    ) -> tuple[float, float, float, float]:
        heading = np.asarray((math.cos(yaw), math.sin(yaw), 0.0))
        lateral = np.asarray((-math.sin(yaw), math.cos(yaw), 0.0))
        contact_s = [float(frame.point @ heading) for frame in contacts.values()]
        contact_l = [float(frame.point @ lateral) for frame in contacts.values()]
        com_s = float(com @ heading)
        com_l = float(com @ lateral)
        longitudinal = min(com_s - min(contact_s), max(contact_s) - com_s)
        lateral_margin = min(com_l - min(contact_l), max(contact_l) - com_l)
        support_s = 0.5 * (min(contact_s) + max(contact_s))
        support_l = 0.5 * (min(contact_l) + max(contact_l))
        return longitudinal, lateral_margin, support_s, support_l

    def plan(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        contacts: Mapping[str, ContactFrame],
        terrain: TerrainEstimate,
        desired_speed: float,
    ) -> PostureTarget:
        roll, measured_pitch, _ = quaternion_to_rpy(state.base_quaternion)
        del roll
        yaw = self.reference_yaw_rad
        if self.reference_lateral_m is None:
            self.reset(state)
            yaw = self.reference_yaw_rad
        slope_deg = clamp(
            max(terrain.fast_slope_deg, terrain.stable_slope_deg),
            0.0, self.config.maximum_slope_deg,
        )
        normalized = slope_deg / max(self.config.maximum_slope_deg, 1.0)
        smooth = normalized * normalized * (3.0 - 2.0 * normalized)
        alignment = 0.18 + 0.62 * smooth
        desired_pitch = math.radians(self.config.desired_level_pitch_deg) - (
            alignment * math.radians(slope_deg)
        )
        phase_scale = 0.85 if terrain.contact_phase in (
            ContactPhase.MIXED_CONTACT_ENTRY,
            ContactPhase.MIXED_CONTACT_EXIT,
        ) else 1.0
        maximum_lean = math.radians(min(
            self.config.maximum_pitch_deg - 5.0,
            0.90 * self.config.maximum_slope_deg,
        ))
        feasible_pitch = clamp(
            phase_scale * desired_pitch,
            -maximum_lean, maximum_lean,
        )
        longitudinal, lateral_margin, support_s, support_l = self.stability_margins(
            dynamics.com_position, contacts, yaw,
        )
        heading = np.asarray((math.cos(yaw), math.sin(yaw), 0.0))
        lateral_axis = np.asarray((-math.sin(yaw), math.cos(yaw), 0.0))
        uphill_shift = clamp(0.070 * math.sin(math.radians(slope_deg)), 0.0, 0.060)
        desired_com = dynamics.com_position.copy()
        current_s = float(desired_com @ heading)
        current_l = float(desired_com @ lateral_axis)
        desired_com += (support_s + uphill_shift - current_s) * heading
        lateral_target = clamp(
            float(self.reference_lateral_m),
            support_l - 0.08,
            support_l + 0.08,
        )
        desired_com += (lateral_target - current_l) * lateral_axis
        measured_forward = float(dynamics.com_velocity @ heading)
        desired_velocity = desired_speed * math.cos(math.radians(slope_deg)) * heading
        desired_velocity[2] = desired_speed * math.sin(math.radians(slope_deg))
        feedforward = clamp(
            3.0 * (desired_speed - measured_forward),
            -self.config.maximum_com_accel_mps2,
            self.config.maximum_com_accel_mps2,
        ) * heading
        acceleration = (
            feedforward
            + self.config.com_kp * (desired_com - dynamics.com_position)
            + self.config.com_kd * (desired_velocity - dynamics.com_velocity)
        )
        norm = float(np.linalg.norm(acceleration))
        if norm > self.config.maximum_com_accel_mps2:
            acceleration *= self.config.maximum_com_accel_mps2 / norm
        # Avoid requesting an instantaneous discontinuous pitch target.
        feasible_pitch = measured_pitch + clamp(
            feasible_pitch - measured_pitch, -math.radians(12.0), math.radians(12.0),
        )
        return PostureTarget(
            desired_pitch_rad=desired_pitch,
            feasible_pitch_rad=feasible_pitch,
            desired_yaw_rad=yaw,
            desired_com_position=desired_com,
            desired_com_acceleration=acceleration,
            longitudinal_margin_m=longitudinal,
            lateral_margin_m=lateral_margin,
        )


@dataclass(frozen=True)
class QpResult:
    """Atomic complete solver result; partial results are never published."""

    success: bool
    status: str
    qdd: np.ndarray
    force_components: np.ndarray
    world_forces: np.ndarray
    tau: np.ndarray
    iterations: int = 0
    runtime_sec: float = 0.0
    objective: float = 0.0
    primal_residual: float = 0.0
    dual_residual: float = 0.0
    warm_started: bool = False


def complete_solution_or_fallback(
    candidate: QpResult,
    last_safe: QpResult | None,
    consecutive_failures: int,
    maximum_failures: int,
) -> QpResult | None:
    """Select only a complete current or bounded-duration last-safe solution."""
    if candidate.success:
        return candidate
    if last_safe is not None and consecutive_failures <= maximum_failures:
        return last_safe
    return None


class WholeBodyQp:
    """Fixed-sparsity, warm-started inverse-dynamics/rolling-contact QP."""

    def __init__(self, robot: RobotModel, config: WbcConfig) -> None:
        self.robot = robot
        self.config = config
        self.nqdd = robot.nv
        self.nforce = 12
        self.ntau = len(ACTUATED_JOINT_NAMES)
        self.force_offset = self.nqdd
        self.tau_offset = self.force_offset + self.nforce
        self.nvar = self.tau_offset + self.ntau
        self.dynamics_rows = slice(0, self.nqdd)
        self.contact_rows = slice(self.nqdd, self.nqdd + 12)
        self.bound_rows = slice(self.nqdd + 12, self.nqdd + 12 + self.nvar)
        self.friction_rows = slice(self.bound_rows.stop, self.bound_rows.stop + 16)
        self.ncon = self.friction_rows.stop
        self._P_template, self._P_lookup = self._build_p_pattern()
        self._A_template, self._A_lookup = self._build_a_pattern()
        self.solver: osqp.OSQP | None = None
        self.previous_x: np.ndarray | None = None
        self.previous_tau: np.ndarray | None = None
        self.previous_force: np.ndarray | None = None
        self.solve_count = 0

    @staticmethod
    def _matrix_lookup(matrix: sparse.csc_matrix) -> dict[tuple[int, int], int]:
        lookup: dict[tuple[int, int], int] = {}
        for col in range(matrix.shape[1]):
            for index in range(matrix.indptr[col], matrix.indptr[col + 1]):
                lookup[(int(matrix.indices[index]), col)] = index
        return lookup

    def _build_p_pattern(self) -> tuple[sparse.csc_matrix, dict[tuple[int, int], int]]:
        rows: list[int] = []
        cols: list[int] = []
        for col in range(self.nqdd):
            for row in range(col + 1):
                rows.append(row)
                cols.append(col)
        for index in range(self.force_offset, self.nvar):
            rows.append(index)
            cols.append(index)
        matrix = sparse.csc_matrix(
            (np.ones(len(rows)), (rows, cols)), shape=(self.nvar, self.nvar),
        )
        return matrix, self._matrix_lookup(matrix)

    def _build_a_pattern(self) -> tuple[sparse.csc_matrix, dict[tuple[int, int], int]]:
        entries: set[tuple[int, int]] = set()
        # Floating-base/all-joint dynamics: dense M and contact Jacobians.
        for row in range(self.nqdd):
            entries.update((row, col) for col in range(self.nqdd + self.nforce))
        for actuator, velocity_index in enumerate(self.robot.actuated_v_indices):
            entries.add((int(velocity_index), self.tau_offset + actuator))
        # Three acceleration constraints per rolling wheel.
        for row in range(self.contact_rows.start, self.contact_rows.stop):
            entries.update((row, col) for col in range(self.nqdd))
        # One finite variable-bound row for every decision variable.
        for col in range(self.nvar):
            entries.add((self.bound_rows.start + col, col))
        # Four pyramid faces per wheel, each using tangent/lateral and normal.
        for wheel in range(4):
            normal_col = self.force_offset + 3 * wheel + 2
            for face in range(4):
                row = self.friction_rows.start + 4 * wheel + face
                tangent_col = self.force_offset + 3 * wheel + (0 if face < 2 else 1)
                entries.add((row, tangent_col))
                entries.add((row, normal_col))
        ordered = sorted(entries, key=lambda value: (value[1], value[0]))
        rows = [item[0] for item in ordered]
        cols = [item[1] for item in ordered]
        matrix = sparse.csc_matrix(
            (np.ones(len(rows)), (rows, cols)), shape=(self.ncon, self.nvar),
        )
        return matrix, self._matrix_lookup(matrix)

    @staticmethod
    def _add_task(
        H: np.ndarray,
        g: np.ndarray,
        matrix: np.ndarray,
        target: np.ndarray,
        weight: float | np.ndarray,
    ) -> None:
        if np.isscalar(weight):
            weighted = float(weight) * np.eye(matrix.shape[0])
        else:
            weighted = np.diag(np.asarray(weight, dtype=float))
        H += 2.0 * matrix.T @ weighted @ matrix
        g += -2.0 * matrix.T @ weighted @ target

    def _set_block(
        self,
        data: np.ndarray,
        lookup: Mapping[tuple[int, int], int],
        row_start: int,
        col_start: int,
        block: np.ndarray,
    ) -> None:
        rows, cols = block.shape
        for row in range(rows):
            for col in range(cols):
                index = lookup.get((row_start + row, col_start + col))
                if index is not None:
                    data[index] = block[row, col]

    def _variable_bounds(
        self,
        state: EstimatedState,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        lower = np.full(self.nvar, -MAX_BOUND)
        upper = np.full(self.nvar, MAX_BOUND)
        lower[:6] = -30.0
        upper[:6] = 30.0
        lookahead = self.config.velocity_lookahead_sec
        for actuator, name in enumerate(ACTUATED_JOINT_NAMES):
            v_index = self.robot.v_indices[name]
            qdd_index = v_index
            accel = (
                self.config.max_wheel_accel_radps2
                if name in WHEEL_JOINT_NAMES
                else self.config.max_joint_accel_radps2
            )
            velocity_limit = min(
                self.robot.velocity_limits[actuator],
                self.config.maximum_wheel_speed_radps
                if name in WHEEL_JOINT_NAMES else self.robot.velocity_limits[actuator],
            )
            velocity = state.v[v_index]
            qdd_lower = max(-accel, (-velocity_limit - velocity) / lookahead)
            qdd_upper = min(accel, (velocity_limit - velocity) / lookahead)
            if name in LEG_JOINT_NAMES:
                leg_index = LEG_JOINT_NAMES.index(name)
                position = state.q[self.robot.q_indices[name]]
                position_lower = (
                    self.robot.position_lower[leg_index]
                    + self.config.joint_limit_margin_rad
                )
                position_upper = (
                    self.robot.position_upper[leg_index]
                    - self.config.joint_limit_margin_rad
                )
                scale = 2.0 / (lookahead * lookahead)
                qdd_lower = max(
                    qdd_lower,
                    scale * (position_lower - position - velocity * lookahead),
                )
                qdd_upper = min(
                    qdd_upper,
                    scale * (position_upper - position - velocity * lookahead),
                )
            lower[qdd_index] = qdd_lower
            upper[qdd_index] = qdd_upper
        for wheel in range(4):
            base = self.force_offset + 3 * wheel
            lower[base:base + 2] = -self.config.normal_force_max_n
            upper[base:base + 2] = self.config.normal_force_max_n
            lower[base + 2] = self.config.normal_force_min_n
            upper[base + 2] = self.config.normal_force_max_n
        # ACTUATED_JOINT_NAMES interleaves each wheel, so apply by name.
        for actuator, name in enumerate(ACTUATED_JOINT_NAMES):
            scale = (
                self.config.wheel_torque_scale
                if name in WHEEL_JOINT_NAMES else self.config.leg_torque_scale
            )
            limit = self.robot.effort_limits[actuator] * scale
            lower[self.tau_offset + actuator] = -limit
            upper[self.tau_offset + actuator] = limit
        if self.previous_tau is not None:
            step = self.config.torque_rate_limit_nmps * max(dt, 1.0e-4)
            lower[self.tau_offset:] = np.maximum(
                lower[self.tau_offset:], self.previous_tau - step,
            )
            upper[self.tau_offset:] = np.minimum(
                upper[self.tau_offset:], self.previous_tau + step,
            )
        if self.previous_force is not None:
            step = self.config.contact_force_rate_limit_nps * max(dt, 1.0e-4)
            force_slice = slice(self.force_offset, self.tau_offset)
            lower[force_slice] = np.maximum(
                lower[force_slice], self.previous_force - step,
            )
            upper[force_slice] = np.minimum(
                upper[force_slice], self.previous_force + step,
            )
        if np.any(lower > upper) or not finite(lower, upper):
            raise ValueError('INVALID_QP_VARIABLE_BOUNDS')
        return lower, upper

    def _assemble(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        contacts: Mapping[str, ContactFrame],
        target: PostureTarget,
        desired_speed: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        H = np.zeros((self.nvar, self.nvar))
        g = np.zeros(self.nvar)
        # Full-body COM acceleration task.
        task = np.zeros((3, self.nvar))
        task[:, :self.nqdd] = dynamics.com_jacobian
        self._add_task(
            H, g, task,
            target.desired_com_acceleration - dynamics.com_drift_acceleration,
            self.config.com_weight,
        )
        # SO(3) body task in the local free-flyer tangent coordinates.
        desired_rotation = pin.rpy.rpyToMatrix(
            0.0, target.feasible_pitch_rad, target.desired_yaw_rad,
        )
        orientation_error = pin.log3(dynamics.base_rotation.T @ desired_rotation)
        angular_target = (
            self.config.orientation_kp * orientation_error
            - self.config.orientation_kd * state.v[3:6]
        )
        task = np.zeros((3, self.nvar))
        task[:, 3:6] = np.eye(3)
        self._add_task(
            H, g, task, angular_target,
            self.config.base_orientation_weight * np.asarray((1.5, 1.0, 0.2)),
        )
        # Joint posture regularization, excluding continuously rotating wheels.
        task = np.zeros((12, self.nvar))
        posture_target = np.zeros(12)
        for row, name in enumerate(LEG_JOINT_NAMES):
            v_index = self.robot.v_indices[name]
            q_index = self.robot.q_indices[name]
            task[row, v_index] = 1.0
            posture_target[row] = (
                self.config.posture_kp * (self.robot.nominal_q[q_index] - state.q[q_index])
                - self.config.posture_kd * state.v[v_index]
            )
        self._add_task(H, g, task, posture_target, self.config.posture_weight)
        # Wheel acceleration objective supports speed recovery without replacing
        # the hard rolling constraints.
        task = np.zeros((4, self.nvar))
        wheel_target = np.zeros(4)
        for row, leg in enumerate(LEGS):
            name = f'{leg}_foot_joint'
            task[row, self.robot.v_indices[name]] = 1.0
            desired_omega = desired_speed / self.robot.wheel_radius
            current_omega = state.v[self.robot.v_indices[name]]
            wheel_target[row] = clamp(
                5.0 * (desired_omega - current_omega),
                -self.config.max_wheel_accel_radps2,
                self.config.max_wheel_accel_radps2,
            )
        self._add_task(
            H, g, task, wheel_target,
            self.config.speed_weight + self.config.rolling_constraint_weight,
        )
        # Physically meaningful force targets; dynamics remains the hard arbiter.
        slope = math.asin(clamp(float(np.mean([
            frame.tangent[2] for frame in contacts.values()
        ])), -1.0, 1.0))
        force_target = np.zeros(self.nforce)
        force_target[0::3] = self.robot.mass * GRAVITY * math.sin(slope) / 4.0
        force_target[2::3] = self.robot.mass * GRAVITY * math.cos(slope) / 4.0
        task = np.zeros((self.nforce, self.nvar))
        task[:, self.force_offset:self.tau_offset] = np.eye(self.nforce)
        self._add_task(
            H, g, task, force_target,
            self.config.force_regularization_weight,
        )
        task = np.zeros((self.ntau, self.nvar))
        task[:, self.tau_offset:] = np.eye(self.ntau)
        self._add_task(
            H, g, task, np.zeros(self.ntau),
            self.config.torque_regularization_weight,
        )
        task = np.zeros((self.nqdd, self.nvar))
        task[:, :self.nqdd] = np.eye(self.nqdd)
        self._add_task(
            H, g, task, np.zeros(self.nqdd),
            self.config.qdd_regularization_weight,
        )
        H = 0.5 * (H + H.T)
        H[:self.nqdd, :self.nqdd] += 1.0e-10
        P_data = np.empty_like(self._P_template.data)
        for (row, col), index in self._P_lookup.items():
            P_data[index] = H[row, col]

        A_data = np.zeros_like(self._A_template.data)
        lower = np.full(self.ncon, -MAX_BOUND)
        upper = np.full(self.ncon, MAX_BOUND)
        # M qdd - Jc.T f - S.T tau = -h.
        self._set_block(A_data, self._A_lookup, 0, 0, dynamics.M)
        for wheel, leg in enumerate(LEGS):
            frame = contacts[leg]
            force_map = -(frame.point_jacobian.T @ frame.basis)
            self._set_block(
                A_data, self._A_lookup, 0,
                self.force_offset + 3 * wheel, force_map,
            )
        for actuator, v_index in enumerate(self.robot.actuated_v_indices):
            A_data[self._A_lookup[(int(v_index), self.tau_offset + actuator)]] = -1.0
        lower[self.dynamics_rows] = -dynamics.h
        upper[self.dynamics_rows] = -dynamics.h
        # Normal, lateral no-slip, and longitudinal rolling acceleration.
        for wheel, leg in enumerate(LEGS):
            frame = contacts[leg]
            wheel_v_index = self.robot.v_indices[f'{leg}_foot_joint']
            center_velocity = frame.center_jacobian @ state.v
            wheel_omega = state.v[wheel_v_index]
            directions = (frame.normal, frame.lateral, frame.tangent)
            for axis, direction in enumerate(directions):
                row = self.contact_rows.start + 3 * wheel + axis
                coefficients = direction @ frame.center_jacobian
                self._set_block(
                    A_data, self._A_lookup, row, 0,
                    coefficients.reshape(1, -1),
                )
                error_velocity = float(direction @ center_velocity)
                desired = -self.config.contact_kd * error_velocity
                if axis == 0:
                    desired += -self.config.contact_kp * frame.penetration_error_m
                if axis == 2:
                    A_data[self._A_lookup[(row, wheel_v_index)]] += (
                        -self.robot.wheel_radius * frame.rolling_sign
                    )
                    rolling_error = error_velocity - (
                        self.robot.wheel_radius * frame.rolling_sign * wheel_omega
                    )
                    desired = -self.config.contact_kd * rolling_error
                rhs = desired - float(direction @ frame.center_jdot_v)
                rhs = clamp(rhs, -25.0, 25.0)
                lower[row] = rhs
                upper[row] = rhs
        variable_lower, variable_upper = self._variable_bounds(state, dt)
        for col in range(self.nvar):
            row = self.bound_rows.start + col
            A_data[self._A_lookup[(row, col)]] = 1.0
        lower[self.bound_rows] = variable_lower
        upper[self.bound_rows] = variable_upper
        # Linearized friction cones in each contact's own mixed-phase frame.
        for wheel in range(4):
            tangential_cols = (
                self.force_offset + 3 * wheel,
                self.force_offset + 3 * wheel + 1,
            )
            normal_col = self.force_offset + 3 * wheel + 2
            coefficients = (
                (tangential_cols[0], 1.0, self.config.friction_mu_longitudinal),
                (tangential_cols[0], -1.0, self.config.friction_mu_longitudinal),
                (tangential_cols[1], 1.0, self.config.friction_mu_lateral),
                (tangential_cols[1], -1.0, self.config.friction_mu_lateral),
            )
            for face, (tangent_col, tangent_sign, mu) in enumerate(coefficients):
                row = self.friction_rows.start + 4 * wheel + face
                A_data[self._A_lookup[(row, tangent_col)]] = tangent_sign
                A_data[self._A_lookup[(row, normal_col)]] = -mu
                lower[row] = -MAX_BOUND
                upper[row] = 0.0
        if not finite(P_data, g, A_data, lower, upper):
            raise ValueError('NONFINITE_QP_ASSEMBLY')
        return P_data, g, A_data, lower, upper

    def solve(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        contacts: Mapping[str, ContactFrame],
        target: PostureTarget,
        desired_speed: float,
        dt: float,
    ) -> QpResult:
        """Update numeric values without rebuilding immutable sparsity."""
        try:
            P_data, q, A_data, lower, upper = self._assemble(
                state, dynamics, contacts, target, desired_speed, dt,
            )
            if self.solver is None:
                P = self._P_template.copy()
                P.data = P_data
                A = self._A_template.copy()
                A.data = A_data
                self.solver = osqp.OSQP()
                self.solver.setup(
                    P=P, q=q, A=A, l=lower, u=upper,
                    verbose=False, warm_starting=True, polishing=False,
                    eps_abs=self.config.solver_eps_abs,
                    eps_rel=self.config.solver_eps_rel,
                    max_iter=self.config.solver_max_iter,
                    time_limit=self.config.solver_time_limit_sec,
                    check_termination=10, adaptive_rho=True,
                )
            else:
                self.solver.update(
                    Px=P_data, q=q, Ax=A_data, l=lower, u=upper,
                )
                if self.previous_x is not None:
                    self.solver.warm_start(x=self.previous_x)
            result = self.solver.solve(raise_error=False)
        except Exception as error:
            return self.failed_result(f'QP_ASSEMBLY_ERROR:{error}')
        info = result.info
        solved = str(info.status).lower() in {'solved', 'solved inaccurate'}
        vector = None if result.x is None else np.asarray(result.x, dtype=float)
        if not solved or vector is None or vector.shape != (self.nvar,) or not finite(vector):
            return self.failed_result(
                str(info.status), info,
            )
        candidate_forces = vector[self.force_offset:self.tau_offset]
        safety_violations: list[float] = []
        for wheel in range(4):
            longitudinal, lateral, normal = candidate_forces[
                3 * wheel:3 * wheel + 3
            ]
            safety_violations.extend((
                self.config.normal_force_min_n - normal,
                normal - self.config.normal_force_max_n,
                abs(longitudinal)
                - self.config.friction_mu_longitudinal * normal,
                abs(lateral) - self.config.friction_mu_lateral * normal,
            ))
        candidate_tau = vector[self.tau_offset:]
        for actuator, name in enumerate(ACTUATED_JOINT_NAMES):
            scale = (
                self.config.wheel_torque_scale
                if name in WHEEL_JOINT_NAMES else self.config.leg_torque_scale
            )
            safety_violations.append(
                abs(candidate_tau[actuator])
                - self.robot.effort_limits[actuator] * scale
            )
        maximum_violation = float(max(*safety_violations, 0.0))
        constraint_tolerance = max(0.010, 10.0 * self.config.solver_eps_abs)
        if not math.isfinite(maximum_violation) or maximum_violation > constraint_tolerance:
            return self.failed_result('QP_CONSTRAINT_RESIDUAL', info)
        qdd = vector[:self.nqdd].copy()
        force_components = vector[self.force_offset:self.tau_offset].copy()
        tau = vector[self.tau_offset:].copy()
        world_forces = np.vstack([
            contacts[leg].basis @ force_components[3 * wheel:3 * wheel + 3]
            for wheel, leg in enumerate(LEGS)
        ])
        if not finite(qdd, force_components, world_forces, tau):
            return self.failed_result('NONFINITE_QP_SOLUTION', info)
        warm_started = self.previous_x is not None
        self.previous_x = vector.copy()
        self.previous_tau = tau.copy()
        self.previous_force = force_components.copy()
        self.solve_count += 1
        return QpResult(
            success=True, status=str(info.status), qdd=qdd,
            force_components=force_components, world_forces=world_forces,
            tau=tau, iterations=int(info.iter), runtime_sec=float(info.run_time),
            objective=float(info.obj_val), primal_residual=float(info.prim_res),
            dual_residual=float(info.dual_res), warm_started=warm_started,
        )

    def failed_result(self, status: str, info: Any | None = None) -> QpResult:
        return QpResult(
            success=False, status=status,
            qdd=np.zeros(self.nqdd), force_components=np.zeros(self.nforce),
            world_forces=np.zeros((4, 3)), tau=np.zeros(self.ntau),
            iterations=int(getattr(info, 'iter', 0)),
            runtime_sec=float(getattr(info, 'run_time', 0.0)),
            objective=float(getattr(info, 'obj_val', 0.0)),
            primal_residual=float(getattr(info, 'prim_res', 0.0)),
            dual_residual=float(getattr(info, 'dual_res', 0.0)),
            warm_started=self.previous_x is not None,
        )

    def reset(self) -> None:
        self.solver = None
        self.previous_x = None
        self.previous_tau = None
        self.previous_force = None
        self.solve_count = 0


class CommandInterfaceInspector:
    """Combine authoritative configuration with live controller-manager state."""

    def __init__(
        self, node: Node, config: WbcConfig, control_xacro_path: str,
    ) -> None:
        self.node = node
        self.config = config
        self.configured_commands = self._configured_commands(control_xacro_path)
        self.controller_client = node.create_client(
            ListControllers, f'{config.controller_manager_name}/list_controllers',
        )
        self.hardware_client = node.create_client(
            ListHardwareInterfaces,
            f'{config.controller_manager_name}/list_hardware_interfaces',
        )
        self.controller_future: Any | None = None
        self.hardware_future: Any | None = None
        self.controller_states: dict[str, str] = {}
        self.live_commands: dict[str, tuple[bool, bool]] = {}
        self.last_poll_wall = -math.inf
        self.controller_status_known = False
        self.hardware_status_known = False

    @staticmethod
    def _configured_commands(path: str) -> dict[str, set[str]]:
        result: dict[str, set[str]] = {}
        source = Path(path).expanduser().resolve()
        if not source.is_file():
            return result
        root = ET.parse(source).getroot()
        for joint in root.findall('.//ros2_control/joint'):
            result[joint.get('name', '')] = {
                item.get('name', '') for item in joint.findall('command_interface')
            }
        return result

    def poll(self, now_wall: float) -> None:
        if self.controller_future is not None and self.controller_future.done():
            try:
                response = self.controller_future.result()
                self.controller_states = {
                    item.name: item.state for item in response.controller
                }
                self.controller_status_known = True
            except Exception:
                self.controller_status_known = False
            self.controller_future = None
        if self.hardware_future is not None and self.hardware_future.done():
            try:
                response = self.hardware_future.result()
                self.live_commands = {
                    item.name: (bool(item.is_available), bool(item.is_claimed))
                    for item in response.command_interfaces
                }
                self.hardware_status_known = True
            except Exception:
                self.hardware_status_known = False
            self.hardware_future = None
        if now_wall - self.last_poll_wall < self.config.controller_poll_period_sec:
            return
        if self.controller_future is None and self.controller_client.service_is_ready():
            self.controller_future = self.controller_client.call_async(
                ListControllers.Request()
            )
        if self.hardware_future is None and self.hardware_client.service_is_ready():
            self.hardware_future = self.hardware_client.call_async(
                ListHardwareInterfaces.Request()
            )
        self.last_poll_wall = now_wall

    def configured_effort_missing(self) -> list[str]:
        return [
            f'{name}/effort' for name in ACTUATED_JOINT_NAMES
            if 'effort' not in self.configured_commands.get(name, set())
        ]

    def live_effort_missing(self) -> list[str]:
        missing: list[str] = []
        for name in ACTUATED_JOINT_NAMES:
            interface = f'{name}/effort'
            available, _ = self.live_commands.get(interface, (False, False))
            if not available:
                missing.append(interface)
        return missing

    def effort_available(self) -> bool:
        if self.configured_effort_missing():
            return False
        if not self.hardware_status_known or self.live_effort_missing():
            return False
        return self.controller_states.get(self.config.effort_controller_name) == 'active'

    def hybrid_available(self) -> bool:
        configured = all(
            'position' in self.configured_commands.get(name, set())
            for name in LEG_JOINT_NAMES
        ) and all(
            'velocity' in self.configured_commands.get(name, set())
            for name in WHEEL_JOINT_NAMES
        )
        if not configured or not self.controller_status_known:
            return False
        return (
            self.controller_states.get(self.config.leg_controller_name) == 'active'
            and self.controller_states.get(self.config.wheel_controller_name) == 'active'
        )

    def topic_owned_exclusively(self, topics: Sequence[str]) -> bool:
        own_name = self.node.get_name().lstrip('/')
        own_namespace = self.node.get_namespace().rstrip('/') or '/'
        for topic in topics:
            for info in self.node.get_publishers_info_by_topic(topic):
                if not (
                    info.node_name.lstrip('/') == own_name
                    and (info.node_namespace.rstrip('/') or '/') == own_namespace
                ):
                    return False
        return True

    def command_subscribers_ready(self, topics: Sequence[str]) -> bool:
        return all(self.node.get_subscriptions_info_by_topic(topic) for topic in topics)

    def report(self) -> dict[str, Any]:
        return {
            'configured_command_interfaces': {
                name: sorted(value) for name, value in self.configured_commands.items()
                if name in ACTUATED_JOINT_NAMES
            },
            'live_command_interfaces': {
                name: {'available': value[0], 'claimed': value[1]}
                for name, value in self.live_commands.items()
                if name.split('/')[0] in ACTUATED_JOINT_NAMES
            },
            'controller_states': self.controller_states,
            'configured_effort_missing': self.configured_effort_missing(),
            'live_effort_missing': self.live_effort_missing(),
        }


class EffortCommandAdapter:
    """Rate-limited direct torque publisher for a verified effort backend."""

    def __init__(self, node: Node, robot: RobotModel, config: WbcConfig) -> None:
        self.robot = robot
        self.config = config
        self.publisher = node.create_publisher(
            Float64MultiArray, config.effort_command_topic, 10,
        )
        self.last_command = np.zeros(16)

    def command(self, tau: np.ndarray, dt: float) -> np.ndarray:
        limits = np.asarray([
            self.robot.effort_limits[index] * (
                self.config.wheel_torque_scale
                if name in WHEEL_JOINT_NAMES else self.config.leg_torque_scale
            )
            for index, name in enumerate(ACTUATED_JOINT_NAMES)
        ])
        desired = np.clip(tau, -limits, limits)
        step = self.config.torque_rate_limit_nmps * max(dt, 1.0e-4)
        desired = np.clip(desired, self.last_command - step, self.last_command + step)
        if not finite(desired):
            raise ValueError('NONFINITE_EFFORT_COMMAND')
        self.last_command = desired.copy()
        message = Float64MultiArray()
        message.data = desired.tolist()
        self.publisher.publish(message)
        return desired

    def stop(self) -> None:
        self.last_command[:] = 0.0
        message = Float64MultiArray()
        message.data = self.last_command.tolist()
        self.publisher.publish(message)

    def reset(self) -> None:
        self.last_command[:] = 0.0


@dataclass(frozen=True)
class HybridCommand:
    """Complete atomic hybrid adapter result."""

    leg_positions: np.ndarray
    wheel_velocities: np.ndarray


class HybridPositionVelocityAdapter:
    """Convert WBC acceleration/torque intent to stock safe interfaces."""

    def __init__(
        self,
        node: Node,
        robot: RobotModel,
        config: WbcConfig,
        publishers_enabled: bool,
    ) -> None:
        self.robot = robot
        self.config = config
        self.leg_publisher = (
            node.create_publisher(Float64MultiArray, config.leg_command_topic, 10)
            if publishers_enabled else None
        )
        self.wheel_publisher = (
            node.create_publisher(Float64MultiArray, config.wheel_command_topic, 10)
            if publishers_enabled else None
        )
        self.last_leg_command: np.ndarray | None = None
        self.last_wheel_command = np.zeros(4)

    def propose(
        self,
        state: EstimatedState,
        result: QpResult,
        effective_speed: float,
        dt: float,
    ) -> HybridCommand:
        dt = clamp(dt, 1.0e-4, 0.05)
        positions = np.asarray([
            state.q[self.robot.q_indices[name]] for name in LEG_JOINT_NAMES
        ])
        velocities = np.asarray([
            state.v[self.robot.v_indices[name]] for name in LEG_JOINT_NAMES
        ])
        accelerations = np.asarray([
            result.qdd[self.robot.v_indices[name]] for name in LEG_JOINT_NAMES
        ])
        torque_by_name = {
            name: result.tau[index] for index, name in enumerate(ACTUATED_JOINT_NAMES)
        }
        torque_correction = np.asarray([
            torque_by_name[name] - 0.20 * state.joint_efforts[
                ACTUATED_JOINT_NAMES.index(name)
            ]
            for name in LEG_JOINT_NAMES
        ]) / self.config.virtual_leg_stiffness_nm_rad
        raw = positions + velocities * dt + 0.5 * accelerations * dt * dt
        raw += torque_correction
        reference = positions if self.last_leg_command is None else self.last_leg_command
        leg = np.clip(
            raw,
            reference - self.config.max_leg_position_step_rad,
            reference + self.config.max_leg_position_step_rad,
        )
        leg = np.clip(
            leg,
            self.robot.position_lower + self.config.joint_limit_margin_rad,
            self.robot.position_upper - self.config.joint_limit_margin_rad,
        )
        wheel = np.zeros(4)
        target_speed = max(0.0, effective_speed) / self.robot.wheel_radius
        if effective_speed <= 1.0e-6:
            for index in range(4):
                wheel[index] = rate_limit(
                    self.last_wheel_command[index], 0.0,
                    self.config.max_wheel_accel_radps2, dt,
                )
        else:
            for index, leg_name in enumerate(LEGS):
                name = f'{leg_name}_foot_joint'
                acceleration = result.qdd[self.robot.v_indices[name]]
                integrated = self.last_wheel_command[index] + acceleration * dt
                torque = result.tau[ACTUATED_JOINT_NAMES.index(name)]
                torque_correction = clamp(
                    self.config.wheel_torque_velocity_gain_radps_per_nm * torque,
                    -self.config.maximum_torque_velocity_correction_radps,
                    self.config.maximum_torque_velocity_correction_radps,
                )
                dynamic_target = max(0.0, target_speed + torque_correction)
                blended = 0.65 * integrated + 0.35 * dynamic_target
                wheel[index] = rate_limit(
                    self.last_wheel_command[index], max(0.0, blended),
                    self.config.max_wheel_accel_radps2, dt,
                )
        wheel = np.clip(wheel, 0.0, self.config.maximum_wheel_speed_radps)
        if not finite(leg, wheel):
            raise ValueError('NONFINITE_HYBRID_COMMAND')
        self.last_leg_command = leg.copy()
        self.last_wheel_command = wheel.copy()
        return HybridCommand(leg_positions=leg, wheel_velocities=wheel)

    def publish(self, command: HybridCommand) -> None:
        if self.leg_publisher is None or self.wheel_publisher is None:
            return
        leg_message = Float64MultiArray()
        leg_message.data = command.leg_positions.tolist()
        wheel_message = Float64MultiArray()
        wheel_message.data = command.wheel_velocities.tolist()
        self.leg_publisher.publish(leg_message)
        self.wheel_publisher.publish(wheel_message)

    def stop(self, hold_positions: np.ndarray | None = None) -> None:
        self.last_wheel_command[:] = 0.0
        if self.wheel_publisher is not None:
            wheel = Float64MultiArray()
            wheel.data = [0.0] * 4
            self.wheel_publisher.publish(wheel)
        if hold_positions is not None and finite(hold_positions):
            self.last_leg_command = hold_positions.copy()
            if self.leg_publisher is not None:
                leg = Float64MultiArray()
                leg.data = hold_positions.tolist()
                self.leg_publisher.publish(leg)

    def reset(self) -> None:
        self.last_leg_command = None
        self.last_wheel_command[:] = 0.0


class RunLogger:
    """Unique per-run CSV plus evidence-based summary JSON."""

    FIELDS = (
        'wall_time_utc', 'ros_time_sec', 'state', 'backend', 'enabled',
        'contact_phase', 'slope_deg', 'base_position', 'base_quaternion',
        'base_linear_velocity_body', 'imu_angular_velocity',
        'imu_linear_acceleration', 'com_position', 'com_velocity',
        'desired_com_acceleration', 'pitch_deg', 'pitch_target_deg',
        'pitch_error_deg', 'roll_deg', 'q', 'v', 'qdd', 'desired_torque',
        'published_leg_command', 'published_wheel_command', 'contact_forces_world',
        'contact_force_components', 'friction_utilization', 'slip',
        'qp_status', 'solver_runtime_ms', 'solver_iterations', 'solver_objective',
        'primal_residual', 'dual_residual', 'longitudinal_margin_m',
        'lateral_margin_m', 'requested_speed_mps', 'effective_speed_mps',
        'safety_reason', 'result', 'state_update_ms', 'dynamics_ms',
        'qp_assembly_and_solve_ms', 'total_cycle_ms', 'deadline_missed',
    )

    def __init__(self, config: WbcConfig, backend_label: str) -> None:
        directory = Path(config.output_directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        requested = config.run_id.strip() or 'go2w_wbc'
        safe = ''.join(char if char.isalnum() or char in '-_' else '_' for char in requested)
        self.run_id = f'{safe}_{timestamp}'
        self.csv_path = directory / f'{self.run_id}.csv'
        self.json_path = directory / f'{self.run_id}_summary.json'
        self.file = self.csv_path.open('x', encoding='utf-8', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.config = config
        self.backend_label = backend_label
        self.rows = 0
        self.solver_attempts = 0
        self.solver_successes = 0
        self.solver_failures = 0
        self.solver_times: list[float] = []
        self.iterations: list[int] = []
        self.pitch_values: list[float] = []
        self.roll_values: list[float] = []
        self.slips: list[float] = []
        self.minimum_margin = math.inf
        self.maximum_torque_utilization = 0.0
        self.maximum_friction_utilization = 0.0
        self.deadline_misses = 0
        self.finalized = False
        self.last_flush_wall = time.monotonic()

    @staticmethod
    def encoded(value: Any) -> str:
        if isinstance(value, np.ndarray):
            value = value.tolist()
        return json.dumps(value, separators=(',', ':'), allow_nan=False)

    def write(self, row: Mapping[str, Any]) -> None:
        serializable = dict(row)
        for name in (
            'base_position', 'base_quaternion', 'base_linear_velocity_body',
            'imu_angular_velocity', 'imu_linear_acceleration', 'com_position',
            'com_velocity', 'desired_com_acceleration', 'q', 'v', 'qdd',
            'desired_torque', 'published_leg_command', 'published_wheel_command',
            'contact_forces_world', 'contact_force_components',
            'friction_utilization', 'slip',
        ):
            serializable[name] = self.encoded(serializable.get(name, []))
        self.writer.writerow({name: serializable.get(name, '') for name in self.FIELDS})
        self.rows += 1
        now = time.monotonic()
        if now - self.last_flush_wall >= self.config.log_flush_period_sec:
            self.file.flush()
            self.last_flush_wall = now

    def observe(
        self,
        result: QpResult,
        pitch_deg: float,
        roll_deg: float,
        margin: float,
        friction: Sequence[float],
        slips: Sequence[float],
        torque_utilization: float,
        deadline_missed: bool,
    ) -> None:
        if result.status != 'MONITOR_NO_QP':
            self.solver_attempts += 1
            if result.success:
                self.solver_successes += 1
            else:
                self.solver_failures += 1
            self.solver_times.append(1000.0 * result.runtime_sec)
            self.iterations.append(result.iterations)
        self.pitch_values.append(pitch_deg)
        self.roll_values.append(roll_deg)
        self.slips.extend(abs(float(value)) for value in slips)
        self.minimum_margin = min(self.minimum_margin, margin)
        self.maximum_torque_utilization = max(
            self.maximum_torque_utilization, torque_utilization,
        )
        self.maximum_friction_utilization = max(
            self.maximum_friction_utilization,
            max((float(value) for value in friction), default=0.0),
        )
        self.deadline_misses += int(deadline_missed)

    def finalize(
        self,
        result: str,
        reason: str,
        interface_report: Mapping[str, Any],
        climb_top_result: bool,
        no_topple: bool,
        slope_result_deg: float,
    ) -> None:
        if self.finalized:
            return

        def mean(values: Sequence[float]) -> float | None:
            return (sum(values) / len(values)) if values else None

        summary = {
            'run_id': self.run_id,
            'backend': self.config.backend,
            'actuation_description': self.backend_label,
            'true_effort_wbc': self.config.backend == 'effort_wbc',
            'interface_availability': interface_report,
            'slope_result_deg': slope_result_deg,
            'climb_top_result': climb_top_result,
            'no_topple_result': no_topple,
            'mean_pitch_deg': mean(self.pitch_values),
            'max_abs_pitch_deg': max((abs(v) for v in self.pitch_values), default=None),
            'mean_roll_deg': mean(self.roll_values),
            'max_abs_roll_deg': max((abs(v) for v in self.roll_values), default=None),
            'minimum_com_stability_margin_m': (
                self.minimum_margin if math.isfinite(self.minimum_margin) else None
            ),
            'maximum_torque_utilization': self.maximum_torque_utilization,
            'maximum_friction_utilization': self.maximum_friction_utilization,
            'mean_abs_slip': mean(self.slips),
            'peak_abs_slip': max(self.slips, default=None),
            'solver_success_rate': (
                self.solver_successes / self.solver_attempts
                if self.solver_attempts else None
            ),
            'mean_solver_time_ms': mean(self.solver_times),
            'max_solver_time_ms': max(self.solver_times, default=None),
            'mean_solver_iterations': mean(self.iterations),
            'deadline_misses': self.deadline_misses,
            'qp_failures': self.solver_failures,
            'csv_path': str(self.csv_path),
            'result': result,
            'final_reason': reason,
        }
        with self.json_path.open('x', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
        self.file.flush()
        self.file.close()
        self.finalized = True


class Go2WInverseDynamicsWbcNode(Node):
    """ROS orchestration, safety gating, monitoring, and command ownership."""

    def __init__(self) -> None:
        super().__init__('go2w_inverse_dynamics_wbc_climb')
        self.set_parameters([
            Parameter('use_sim_time', value=True),
        ])
        self.config = WbcConfig.from_node(self)
        source_root = Path(__file__).resolve().parents[2]
        urdf = self.config.urdf_path or str(
            source_root / 'unitree_go2w_description/urdf/go2w_description.urdf'
        )
        control_xacro = self.config.ros2_control_xacro_path or str(
            source_root / 'unitree_go2w_description/urdf/go2w_gazebo_control.urdf.xacro'
        )
        self.robot = RobotModel(urdf)
        self.estimator = StateEstimator(self.robot, self.config)
        self.contacts = ContactEstimator(self.config, self.robot.wheel_radius)
        self.planner = SlopePosturePlanner(self.config)
        self.qp = WholeBodyQp(self.robot, self.config)
        self.interfaces = CommandInterfaceInspector(self, self.config, control_xacro)
        self.effective_mode = self.config.mode
        self.backend_failure_reason = ''
        if self.config.backend == 'effort_wbc':
            missing = self.interfaces.configured_effort_missing()
            if missing:
                self.backend_failure_reason = 'EFFORT_INTERFACES_UNAVAILABLE:' + ','.join(missing)
                if self.config.mode == 'active':
                    self.effective_mode = 'shadow'
        backend_label = (
            'true effort WBC'
            if self.config.backend == 'effort_wbc' and not self.backend_failure_reason
            else 'position/velocity-backed WBC approximation'
            if self.config.backend == 'hybrid_position_velocity'
            else 'shadow WBC (no actuation)'
        )
        self.logger = RunLogger(self.config, backend_label)
        self.effort_adapter: EffortCommandAdapter | None = None
        self.hybrid_adapter: HybridPositionVelocityAdapter | None = None
        if self.config.mode == 'active' and self.config.backend == 'effort_wbc':
            self.effort_adapter = EffortCommandAdapter(self, self.robot, self.config)
        elif self.config.backend in {'shadow', 'hybrid_position_velocity'}:
            self.hybrid_adapter = HybridPositionVelocityAdapter(
                self, self.robot, self.config,
                publishers_enabled=(
                    self.config.mode == 'active'
                    and self.config.backend == 'hybrid_position_velocity'
                ),
            )
        self.enabled = False
        self.auto_start_attempted = False
        self.last_auto_start_wall = -math.inf
        self.state = 'WAITING_FOR_DATA'
        self.safety_reason = self.backend_failure_reason
        self.final_result = 'NOT_RUN'
        self.last_state: EstimatedState | None = None
        self.last_dynamics: DynamicsSnapshot | None = None
        self.last_contacts: dict[str, ContactFrame] = {}
        self.last_target: PostureTarget | None = None
        self.last_qp_result = self.qp.failed_result('NOT_SOLVED')
        self.last_safe_result: QpResult | None = None
        self.last_leg_command = np.zeros(12)
        self.last_wheel_command = np.zeros(4)
        self.consecutive_solver_failures = 0
        self.consecutive_deadline_misses = 0
        self.run_start_ros_sec: float | None = None
        self.last_cycle_ros_sec: float | None = None
        self.last_control_wall = time.monotonic()
        self.last_status_wall = -math.inf
        self.last_phase = ContactPhase.FLAT_APPROACH
        self.maximum_slope_seen = 0.0
        self.maximum_progress_seen = 0.0
        self.initial_progress = 0.0
        self.mixed_contact_start_ros_sec: float | None = None
        self.mixed_contact_checkpoint_ros_sec: float | None = None
        self.mixed_contact_checkpoint_progress_m = 0.0
        self.bridge_process: subprocess.Popen[Any] | None = None
        self.bridge_attempts = 0
        self.bridge_start_wall = time.monotonic() + 1.0
        self.stop_requested = False
        self.exit_requested = False
        self.startup_hold_leg = np.zeros(12)
        self.settle_completed = False
        self._create_ros_interfaces()
        self.timer = self.create_timer(
            1.0 / self.config.control_rate_hz, self._control_cycle,
        )
        self.get_logger().info(
            f'configured mode={self.config.mode} effective_mode={self.effective_mode} '
            f'backend={self.config.backend} enabled=false '
            f'rate={self.config.control_rate_hz:.1f}Hz '
            f'model(nq={self.robot.nq},nv={self.robot.nv},mass={self.robot.mass:.3f}kg,'
            f'wheel_radius={self.robot.wheel_radius:.3f}m)'
        )
        if self.backend_failure_reason:
            self.get_logger().error(self.backend_failure_reason)
        elif self.config.backend == 'hybrid_position_velocity':
            self.get_logger().warning(
                'active backend is a position/velocity-backed WBC approximation'
            )

    def _create_ros_interfaces(self) -> None:
        self.create_subscription(Imu, self.config.imu_topic, self.estimator.on_imu, 20)
        self.create_subscription(
            JointState, self.config.joint_state_topic,
            self.estimator.on_joint_state, 20,
        )
        self.create_subscription(
            Odometry, self.config.odometry_topic,
            self.estimator.on_odometry, 20,
        )
        self.create_subscription(
            PoseArray, self.config.pose_array_topic,
            self.estimator.on_pose_array, 20,
        )
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_service(Trigger, '~/reset', self._on_reset)
        prefix = '/go2w/wbc'
        self.string_publishers = {
            name: self.create_publisher(String, f'{prefix}/{topic}', 10)
            for name, topic in {
                'state': 'state', 'backend': 'backend',
                'solver_status': 'solver/status',
                'phase': 'terrain/contact_phase',
            }.items()
        }
        self.bool_publishers = {
            name: self.create_publisher(Bool, f'{prefix}/{topic}', 10)
            for name, topic in {
                'enabled': 'enabled',
                'deadline': 'solver/deadline_missed',
            }.items()
        }
        self.float_publishers = {
            name: self.create_publisher(Float64, f'{prefix}/{topic}', 10)
            for name, topic in {
                'runtime': 'solver/runtime_ms',
                'objective': 'solver/objective',
                'primal': 'solver/primal_residual',
                'dual': 'solver/dual_residual',
                'slope': 'terrain/slope_deg',
                'roll': 'body/roll_deg', 'pitch': 'body/pitch_deg',
                'pitch_target': 'body/pitch_target_deg',
                'long_margin': 'stability/longitudinal_margin_m',
                'lat_margin': 'stability/lateral_margin_m',
                'speed_requested': 'speed/requested_mps',
                'speed_effective': 'speed/effective_mps',
                **{
                    f'friction_{leg}': (
                        f'contact/{leg.lower()}/friction_utilization'
                    )
                    for leg in LEGS
                },
                **{f'slip_{leg}': f'wheel/{leg.lower()}/slip' for leg in LEGS},
            }.items()
        }
        self.iteration_publisher = self.create_publisher(
            UInt32, f'{prefix}/solver/iterations', 10,
        )
        self.vector_publishers = {
            name: self.create_publisher(Vector3, f'{prefix}/{topic}', 10)
            for name, topic in {
                'angular_velocity': 'body/angular_velocity',
                'com_position': 'com/position', 'com_velocity': 'com/velocity',
                'com_desired_acceleration': 'com/desired_acceleration',
                **{f'force_{leg}': f'contact/{leg.lower()}/force' for leg in LEGS},
            }.items()
        }
        self.torque_publisher = self.create_publisher(
            JointState, f'{prefix}/torque/desired_joint_state', 10,
        )
        self.qdd_publisher = self.create_publisher(
            JointState, f'{prefix}/qdd/desired_joint_state', 10,
        )
        self.proposed_joint_publisher = self.create_publisher(
            JointState, f'{prefix}/command/proposed_joint_state', 10,
        )
        self.proposed_wheel_publisher = self.create_publisher(
            Float64MultiArray, f'{prefix}/command/proposed_wheel_velocity', 10,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, f'{prefix}/diagnostics', 10,
        )

    def _active_topics(self) -> tuple[str, ...]:
        if self.config.backend == 'hybrid_position_velocity':
            return (self.config.leg_command_topic, self.config.wheel_command_topic)
        if self.config.backend == 'effort_wbc':
            return (self.config.effort_command_topic,)
        return ()

    def _start_validation(self) -> str:
        now = time.monotonic()
        reason = self.estimator.readiness_reason(now)
        if reason:
            return reason
        if self.effective_mode == 'monitor':
            return 'MONITOR_MODE_HAS_NO_QP_ACTUATION'
        if self.backend_failure_reason and self.effective_mode != 'shadow':
            return self.backend_failure_reason
        if self.config.mode == 'active':
            available = (
                self.interfaces.hybrid_available()
                if self.config.backend == 'hybrid_position_velocity'
                else self.interfaces.effort_available()
            )
            if not available:
                return 'REQUIRED_COMMAND_BACKEND_UNAVAILABLE'
            topics = self._active_topics()
            if not self.interfaces.command_subscribers_ready(topics):
                return 'REQUIRED_CONTROLLER_COMMAND_TOPIC_INACTIVE'
            if not self.interfaces.topic_owned_exclusively(topics):
                return 'CONTROLLER_CONFLICT'
        try:
            state = self.estimator.snapshot(now)
            dynamics = self.robot.compute(state)
        except ValueError as error:
            return str(error)
        if not finite(dynamics.M, dynamics.h):
            return 'NONFINITE_DYNAMICS'
        return ''

    def _enable(self) -> tuple[bool, str]:
        reason = self._start_validation()
        if reason:
            self.safety_reason = reason
            return False, reason
        state = self.estimator.snapshot(time.monotonic())
        self.contacts.reset(state)
        self.planner.reset(state)
        self.qp.reset()
        if self.effort_adapter:
            self.effort_adapter.reset()
        if self.hybrid_adapter:
            self.hybrid_adapter.reset()
        self.enabled = True
        self.stop_requested = False
        self.state = 'RUNNING_SHADOW' if self.effective_mode == 'shadow' else 'RUNNING_ACTIVE'
        self.safety_reason = ''
        self.final_result = 'RUNNING'
        self.run_start_ros_sec = self._ros_now()
        self.startup_hold_leg = np.asarray([
            state.q[self.robot.q_indices[name]] for name in LEG_JOINT_NAMES
        ])
        self.settle_completed = False
        self.last_cycle_ros_sec = None
        self.consecutive_solver_failures = 0
        self.consecutive_deadline_misses = 0
        self.initial_progress = 0.0
        self.maximum_progress_seen = 0.0
        self.mixed_contact_start_ros_sec = None
        self.mixed_contact_checkpoint_ros_sec = None
        self.mixed_contact_checkpoint_progress_m = 0.0
        self.get_logger().info(f'WBC enabled: {self.state}')
        return True, self.state

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.enabled:
            response.success = True
            response.message = 'already enabled'
            return response
        response.success, response.message = self._enable()
        if response.success and self.backend_failure_reason:
            response.success = False
            response.message = (
                f'{self.backend_failure_reason}; shadow computation is running '
                'without actuator commands'
            )
        return response

    def _on_stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.stop_requested = True
        self.safety_reason = 'USER_STOP'
        response.success = True
        response.message = 'safe stop requested'
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._publish_safe_stop()
        self.enabled = False
        self.stop_requested = False
        self.qp.reset()
        if self.effort_adapter:
            self.effort_adapter.reset()
        if self.hybrid_adapter:
            self.hybrid_adapter.reset()
        self.estimator.last_error = ''
        self.contacts.traction.reset()
        self.state = 'WAITING_FOR_DATA'
        self.safety_reason = self.backend_failure_reason
        self.final_result = 'RESET'
        self.last_safe_result = None
        self.settle_completed = False
        self.consecutive_solver_failures = 0
        self.mixed_contact_start_ros_sec = None
        self.mixed_contact_checkpoint_ros_sec = None
        response.success = True
        response.message = 'estimators, adapters, warm start, and fault state reset'
        return response

    def _manage_pose_bridge(self, now_wall: float) -> None:
        if not self.config.manage_pose_bridge:
            return
        odometry = self.estimator.poses.get('odometry')
        if odometry and StateEstimator._fresh(
            odometry['wall'], self.config.pose_timeout_sec, now_wall,
        ):
            self._stop_pose_bridge()
            return
        if self.bridge_process is not None and self.bridge_process.poll() is not None:
            self.bridge_process.wait()
            self.bridge_process = None
        if (
            self.bridge_process is None and self.bridge_attempts < 2
            and now_wall >= self.bridge_start_wall
        ):
            mapping = (
                f'{self.config.gazebo_pose_topic}'
                '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'
            )
            command = [
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', mapping,
                '--ros-args', '-r',
                f'{self.config.gazebo_pose_topic}:={self.config.pose_array_topic}',
            ]
            try:
                self.bridge_process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL, start_new_session=True,
                )
                self.bridge_attempts += 1
                self.bridge_start_wall = now_wall + 0.5
                self.get_logger().info('started managed Gazebo pose fallback bridge')
            except OSError as error:
                self.bridge_attempts = 2
                self.get_logger().warning(f'pose fallback bridge unavailable: {error}')

    def _stop_pose_bridge(self) -> None:
        process = self.bridge_process
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.bridge_process = None

    def _effective_speed(
        self,
        terrain: TerrainEstimate,
        margin: float,
        slips: Sequence[float],
    ) -> float:
        phase_scale = {
            ContactPhase.FLAT_APPROACH: 1.0,
            ContactPhase.ENTRY_DETECTED: 0.60,
            ContactPhase.FRONT_AXLE_ON_RAMP: 0.45,
            ContactPhase.MIXED_CONTACT_ENTRY: 0.38,
            ContactPhase.FULL_SLOPE: 0.72,
            ContactPhase.CREST_DETECTED: 0.45,
            ContactPhase.FRONT_AXLE_ON_TOP: 0.38,
            ContactPhase.MIXED_CONTACT_EXIT: 0.35,
            ContactPhase.TOP_PLATFORM: 0.0 if self.config.shutdown_after_top else 0.30,
        }[terrain.contact_phase]
        slope = clamp(
            max(terrain.fast_slope_deg, terrain.stable_slope_deg),
            0.0, self.config.maximum_slope_deg,
        )
        slope_scale = clamp(1.0 - 0.008 * slope, 0.55, 1.0)
        slip_peak = max((abs(float(value)) for value in slips), default=0.0)
        slip_scale = clamp(
            1.0 - slip_peak / max(
                2.0 * self.config.maximum_slip_ratio, 1.0e-6,
            ),
            0.35, 1.0,
        )
        margin_scale = clamp((margin - 0.005) / 0.045, 0.20, 1.0)
        speed = self.config.commanded_speed_mps * min(
            phase_scale, slope_scale, slip_scale, margin_scale,
        )
        if terrain.contact_phase in {
            ContactPhase.FRONT_AXLE_ON_RAMP,
            ContactPhase.MIXED_CONTACT_ENTRY,
            ContactPhase.FRONT_AXLE_ON_TOP,
            ContactPhase.MIXED_CONTACT_EXIT,
        } and margin >= self.config.minimum_stability_margin_m:
            speed = max(
                speed,
                min(self.config.commanded_speed_mps,
                    self.config.mixed_contact_minimum_speed_mps),
            )
        if self.stop_requested or self.consecutive_solver_failures:
            speed *= max(0.0, 1.0 - 0.25 * self.consecutive_solver_failures)
        return clamp(speed, 0.0, self.config.commanded_speed_mps)

    def _solution_metrics(
        self, result: QpResult,
    ) -> tuple[np.ndarray, float, float]:
        friction = np.zeros(4)
        if not result.success:
            return friction, 0.0, 0.0
        for wheel in range(4):
            longitudinal, lateral, normal = result.force_components[3 * wheel:3 * wheel + 3]
            if normal > 1.0e-8:
                friction[wheel] = max(
                    abs(longitudinal) / (self.config.friction_mu_longitudinal * normal),
                    abs(lateral) / (self.config.friction_mu_lateral * normal),
                )
            else:
                friction[wheel] = MAX_BOUND
        torque_limits = np.asarray([
            self.robot.effort_limits[index] * (
                self.config.wheel_torque_scale
                if name in WHEEL_JOINT_NAMES else self.config.leg_torque_scale
            )
            for index, name in enumerate(ACTUATED_JOINT_NAMES)
        ])
        torque_utilization = float(np.max(np.abs(result.tau) / torque_limits))
        return friction, torque_utilization, float(np.max(friction))

    def _mixed_transition_reason(
        self, terrain: TerrainEstimate, now_ros: float,
    ) -> str:
        """Bound the anti-deadlock crawl by elapsed time and measured progress."""
        mixed = terrain.contact_phase in {
            ContactPhase.FRONT_AXLE_ON_RAMP,
            ContactPhase.MIXED_CONTACT_ENTRY,
        }
        if not mixed:
            self.mixed_contact_start_ros_sec = None
            self.mixed_contact_checkpoint_ros_sec = None
            return ''
        if self.mixed_contact_start_ros_sec is None:
            self.mixed_contact_start_ros_sec = now_ros
            self.mixed_contact_checkpoint_ros_sec = now_ros
            self.mixed_contact_checkpoint_progress_m = terrain.forward_progress_m
            return ''
        if (
            now_ros - self.mixed_contact_start_ros_sec
            > self.config.mixed_contact_escape_max_duration_sec
        ):
            return 'MIXED_CONTACT_TIMEOUT'
        checkpoint = self.mixed_contact_checkpoint_ros_sec
        if (
            checkpoint is not None
            and now_ros - checkpoint
            >= self.config.mixed_contact_progress_timeout_sec
        ):
            progress = (
                terrain.forward_progress_m
                - self.mixed_contact_checkpoint_progress_m
            )
            self.mixed_contact_checkpoint_ros_sec = now_ros
            self.mixed_contact_checkpoint_progress_m = terrain.forward_progress_m
            if progress < self.config.mixed_contact_min_progress_m:
                return 'MIXED_CONTACT_NO_PROGRESS'
        return ''

    def _safety_check(
        self,
        state: EstimatedState,
        target: PostureTarget,
        slips: Sequence[float],
        result: QpResult,
        friction_peak: float,
        torque_utilization: float,
        traction_state: str,
        traction_reason: str,
        now_ros: float,
    ) -> str:
        roll, pitch, _ = quaternion_to_rpy(state.base_quaternion)
        transition_reason = self._mixed_transition_reason(
            self.contacts.terrain, now_ros,
        )
        if transition_reason:
            return transition_reason
        if abs(math.degrees(roll)) > self.config.maximum_roll_deg:
            return 'EXCESSIVE_ROLL'
        if abs(math.degrees(pitch)) > self.config.maximum_pitch_deg:
            return 'EXCESSIVE_PITCH'
        if (
            min(target.longitudinal_margin_m, target.lateral_margin_m)
            < self.config.minimum_stability_margin_m
        ):
            return 'NEGATIVE_STABILITY_MARGIN'
        mixed_escape = (
            self.contacts.terrain.contact_phase in {
                ContactPhase.FRONT_AXLE_ON_RAMP,
                ContactPhase.MIXED_CONTACT_ENTRY,
            }
            and min(
                target.longitudinal_margin_m,
                target.lateral_margin_m,
            ) > max(0.03, self.config.minimum_stability_margin_m)
            and friction_peak < 0.75
        )
        if traction_state in {'STOP', 'INFEASIBLE'} and not mixed_escape:
            return traction_reason.upper() or 'EXCESSIVE_SLIP'
        if result.success:
            constraint_tolerance = max(
                0.010, 10.0 * self.config.solver_eps_abs,
            )
            normals = result.force_components[2::3]
            if np.any(
                normals
                < self.config.normal_force_min_n - constraint_tolerance
            ):
                return 'NEGATIVE_NORMAL_FORCE'
            friction_ratio_tolerance = constraint_tolerance / max(
                min(
                    self.config.friction_mu_longitudinal,
                    self.config.friction_mu_lateral,
                ) * self.config.normal_force_min_n,
                1.0,
            )
            if friction_peak > 1.001 + friction_ratio_tolerance:
                return 'FRICTION_CONE_VIOLATION'
            if torque_utilization > 1.001:
                return 'TORQUE_BOUND_VIOLATION'
        if self.run_start_ros_sec is not None:
            if now_ros - self.run_start_ros_sec > self.config.maximum_runtime_sec:
                return 'MAXIMUM_RUNTIME'
        if (
            self.contacts.terrain.contact_phase != ContactPhase.TOP_PLATFORM
            and self.final_result != 'CLIMB_SUCCESS'
            and self.contacts.terrain.forward_progress_m
            < self.maximum_progress_seen - self.config.maximum_backward_progress_m
        ):
            return 'BACKWARD_MOTION'
        return ''

    def _fault(self, reason: str, result: str = 'SAFE_INFEASIBLE') -> None:
        if not self.enabled:
            self.safety_reason = reason
            return
        self.enabled = False
        self.state = 'WBC_SOLVER_FAULT' if reason.startswith('WBC_SOLVER') else result
        self.safety_reason = reason
        self.final_result = result
        self._publish_safe_stop()
        self.get_logger().error(f'{self.state}: {reason}')

    def _publish_safe_stop(self) -> None:
        hold = None
        if self.last_state is not None:
            hold = np.asarray([
                self.last_state.q[self.robot.q_indices[name]]
                for name in LEG_JOINT_NAMES
            ])
        if self.hybrid_adapter:
            self.hybrid_adapter.stop(hold)
        if self.effort_adapter:
            self.effort_adapter.stop()
        self.last_wheel_command = np.zeros(4)
        if hold is not None:
            self.last_leg_command = hold

    @staticmethod
    def _vector_message(values: Sequence[float]) -> Vector3:
        message = Vector3()
        message.x, message.y, message.z = (float(value) for value in values)
        return message

    def _publish_monitoring(
        self,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        target: PostureTarget,
        result: QpResult,
        terrain: TerrainEstimate,
        slips: Sequence[float],
        friction: Sequence[float],
        effective_speed: float,
        deadline_missed: bool,
    ) -> None:
        roll, pitch, _ = quaternion_to_rpy(state.base_quaternion)
        strings = {
            'state': self.state,
            'backend': (
                self.config.backend if not self.backend_failure_reason
                else f'{self.config.backend}:unavailable_shadow'
            ),
            'solver_status': result.status,
            'phase': terrain.contact_phase.value,
        }
        for name, value in strings.items():
            message = String()
            message.data = value
            self.string_publishers[name].publish(message)
        for name, value in {'enabled': self.enabled, 'deadline': deadline_missed}.items():
            message = Bool()
            message.data = bool(value)
            self.bool_publishers[name].publish(message)
        floats = {
            'runtime': 1000.0 * result.runtime_sec,
            'objective': result.objective, 'primal': result.primal_residual,
            'dual': result.dual_residual,
            'slope': max(terrain.fast_slope_deg, terrain.stable_slope_deg),
            'roll': math.degrees(roll), 'pitch': math.degrees(pitch),
            'pitch_target': math.degrees(target.feasible_pitch_rad),
            'long_margin': target.longitudinal_margin_m,
            'lat_margin': target.lateral_margin_m,
            'speed_requested': self.config.commanded_speed_mps,
            'speed_effective': effective_speed,
            **{f'friction_{leg}': friction[index] for index, leg in enumerate(LEGS)},
            **{f'slip_{leg}': slips[index] for index, leg in enumerate(LEGS)},
        }
        for name, value in floats.items():
            if math.isfinite(float(value)):
                message = Float64()
                message.data = float(value)
                self.float_publishers[name].publish(message)
        iteration = UInt32()
        iteration.data = max(0, result.iterations)
        self.iteration_publisher.publish(iteration)
        vectors = {
            'angular_velocity': state.base_angular_velocity_body,
            'com_position': dynamics.com_position,
            'com_velocity': dynamics.com_velocity,
            'com_desired_acceleration': target.desired_com_acceleration,
            **{f'force_{leg}': result.world_forces[index] for index, leg in enumerate(LEGS)},
        }
        for name, value in vectors.items():
            if finite(value):
                self.vector_publishers[name].publish(self._vector_message(value))
        stamp = self.get_clock().now().to_msg()
        torque = JointState()
        torque.header.stamp = stamp
        torque.name = list(ACTUATED_JOINT_NAMES)
        torque.effort = result.tau.tolist()
        self.torque_publisher.publish(torque)
        qdd = JointState()
        qdd.header.stamp = stamp
        qdd.name = list(ACTUATED_JOINT_NAMES)
        # JointState has no acceleration field; velocity carries qdd by topic contract.
        qdd.velocity = [
            float(result.qdd[self.robot.v_indices[name]])
            for name in ACTUATED_JOINT_NAMES
        ]
        self.qdd_publisher.publish(qdd)
        proposed = JointState()
        proposed.header.stamp = stamp
        proposed.name = list(LEG_JOINT_NAMES)
        proposed.position = self.last_leg_command.tolist()
        self.proposed_joint_publisher.publish(proposed)
        wheels = Float64MultiArray()
        wheels.data = self.last_wheel_command.tolist()
        self.proposed_wheel_publisher.publish(wheels)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'go2w_inverse_dynamics_wbc_climb'
        status.hardware_id = 'go2w'
        status.level = (
            DiagnosticStatus.ERROR if self.state in {'FAULT', 'WBC_SOLVER_FAULT'}
            else DiagnosticStatus.WARN if self.safety_reason else DiagnosticStatus.OK
        )
        status.message = self.safety_reason or self.state
        status.values = [
            KeyValue(key='mode', value=self.effective_mode),
            KeyValue(key='backend', value=self.config.backend),
            KeyValue(key='pose_source', value=state.pose_source),
            KeyValue(key='contact_phase', value=terrain.contact_phase.value),
            KeyValue(key='solver_status', value=result.status),
            KeyValue(key='run_id', value=self.logger.run_id),
        ]
        diagnostics.status = [status]
        self.diagnostics_publisher.publish(diagnostics)

    def _log_cycle(
        self,
        now_ros: float,
        state: EstimatedState,
        dynamics: DynamicsSnapshot,
        target: PostureTarget,
        result: QpResult,
        terrain: TerrainEstimate,
        slips: Sequence[float],
        friction: Sequence[float],
        effective_speed: float,
        timings: Mapping[str, float],
        deadline_missed: bool,
    ) -> None:
        roll, pitch, _ = quaternion_to_rpy(state.base_quaternion)
        self.logger.write({
            'wall_time_utc': datetime.now(timezone.utc).isoformat(),
            'ros_time_sec': now_ros, 'state': self.state,
            'backend': self.config.backend, 'enabled': self.enabled,
            'contact_phase': terrain.contact_phase.value,
            'slope_deg': max(terrain.fast_slope_deg, terrain.stable_slope_deg),
            'base_position': state.base_position,
            'base_quaternion': state.base_quaternion,
            'base_linear_velocity_body': state.base_linear_velocity_body,
            'imu_angular_velocity': state.base_angular_velocity_body,
            'imu_linear_acceleration': self.estimator.imu_linear_acceleration,
            'com_position': dynamics.com_position,
            'com_velocity': dynamics.com_velocity,
            'desired_com_acceleration': target.desired_com_acceleration,
            'pitch_deg': math.degrees(pitch),
            'pitch_target_deg': math.degrees(target.feasible_pitch_rad),
            'pitch_error_deg': math.degrees(target.feasible_pitch_rad - pitch),
            'roll_deg': math.degrees(roll), 'q': state.q, 'v': state.v,
            'qdd': result.qdd, 'desired_torque': result.tau,
            'published_leg_command': self.last_leg_command,
            'published_wheel_command': self.last_wheel_command,
            'contact_forces_world': result.world_forces,
            'contact_force_components': result.force_components,
            'friction_utilization': friction, 'slip': slips,
            'qp_status': result.status,
            'solver_runtime_ms': 1000.0 * result.runtime_sec,
            'solver_iterations': result.iterations,
            'solver_objective': result.objective,
            'primal_residual': result.primal_residual,
            'dual_residual': result.dual_residual,
            'longitudinal_margin_m': target.longitudinal_margin_m,
            'lateral_margin_m': target.lateral_margin_m,
            'requested_speed_mps': self.config.commanded_speed_mps,
            'effective_speed_mps': effective_speed,
            'safety_reason': self.safety_reason, 'result': self.final_result,
            'state_update_ms': timings.get('state', 0.0),
            'dynamics_ms': timings.get('dynamics', 0.0),
            'qp_assembly_and_solve_ms': timings.get('qp', 0.0),
            'total_cycle_ms': timings.get('total', 0.0),
            'deadline_missed': deadline_missed,
        })

    def _ros_now(self) -> float:
        nanoseconds = self.get_clock().now().nanoseconds
        return 1.0e-9 * float(nanoseconds)

    def _control_cycle(self) -> None:
        cycle_start = time.perf_counter()
        now_wall = time.monotonic()
        now_ros = self._ros_now()
        self.interfaces.poll(now_wall)
        self._manage_pose_bridge(now_wall)
        readiness = self.estimator.readiness_reason(now_wall)
        monitor_cycle = self.effective_mode == 'monitor' and not readiness
        if not self.enabled and not monitor_cycle:
            if self.state not in {'SAFE_INFEASIBLE', 'FAULT', 'WBC_SOLVER_FAULT'}:
                self.state = 'READY_DISABLED' if not readiness else 'WAITING_FOR_DATA'
            if (
                self.config.auto_start and not readiness
                and not self.auto_start_attempted
                and now_wall - self.last_auto_start_wall >= 0.50
            ):
                self.last_auto_start_wall = now_wall
                success, reason = self._enable()
                self.auto_start_attempted = success or bool(
                    self.backend_failure_reason
                )
                if not success:
                    self.get_logger().warning(f'auto_start refused: {reason}')
            if not self.enabled:
                self._publish_idle_diagnostics(readiness)
                return
        if monitor_cycle:
            self.state = 'MONITORING'
        if self.enabled and self.effective_mode == 'active':
            available = (
                self.interfaces.hybrid_available()
                if self.config.backend == 'hybrid_position_velocity'
                else self.interfaces.effort_available()
            )
            topics = self._active_topics()
            if not available:
                self._fault('REQUIRED_CONTROLLER_INACTIVE', 'FAULT')
                return
            if not self.interfaces.command_subscribers_ready(topics):
                self._fault('COMMAND_SUBSCRIPTION_DISAPPEARED', 'FAULT')
                return
            if not self.interfaces.topic_owned_exclusively(topics):
                self._fault('CONTROLLER_CONFLICT', 'FAULT')
                return
        if readiness:
            self._fault(readiness, 'FAULT')
            return
        if self.last_cycle_ros_sec is not None and now_ros < self.last_cycle_ros_sec - 1.0e-9:
            self._fault('TIME_REVERSAL', 'FAULT')
            return
        dt = (
            1.0 / self.config.control_rate_hz if self.last_cycle_ros_sec is None
            else clamp(now_ros - self.last_cycle_ros_sec, 1.0e-4, 0.05)
        )
        self.last_cycle_ros_sec = now_ros
        state_start = time.perf_counter()
        try:
            state = self.estimator.snapshot(now_wall)
        except ValueError as error:
            self._fault(str(error), 'FAULT')
            return
        state_ms = 1000.0 * (time.perf_counter() - state_start)
        dynamics_start = time.perf_counter()
        try:
            dynamics = self.robot.compute(state)
            terrain = self.contacts.update_terrain(
                state, dynamics, self.estimator.imu_linear_acceleration,
            )
            frames = self.contacts.frames(state, dynamics, terrain)
        except (ValueError, RuntimeError) as error:
            self._fault(str(error), 'FAULT')
            return
        dynamics_ms = 1000.0 * (time.perf_counter() - dynamics_start)
        settling = (
            self.enabled
            and self.run_start_ros_sec is not None
            and now_ros - self.run_start_ros_sec < self.config.startup_settle_sec
        )
        if settling:
            terrain = TerrainEstimate(
                contact_phase=ContactPhase.FLAT_APPROACH,
                valid=True, stale=False,
            )
            frames = self.contacts.frames(state, dynamics, terrain)
            self.state = 'STARTUP_SETTLE'
        elif self.enabled and not self.settle_completed:
            self.contacts.reset(state)
            terrain = TerrainEstimate(
                contact_phase=ContactPhase.FLAT_APPROACH,
                valid=True, stale=False,
            )
            frames = self.contacts.frames(state, dynamics, terrain)
            self.settle_completed = True
            self.state = (
                'RUNNING_SHADOW'
                if self.effective_mode == 'shadow' else 'RUNNING_ACTIVE'
            )
        self.maximum_slope_seen = max(
            self.maximum_slope_seen, terrain.fast_slope_deg, terrain.stable_slope_deg,
        )
        self.maximum_progress_seen = max(
            self.maximum_progress_seen, terrain.forward_progress_m,
        )
        if terrain.contact_phase != self.last_phase:
            self.get_logger().info(
                f'contact phase: {self.last_phase.value} -> {terrain.contact_phase.value}'
            )
            if (
                self.last_phase in {
                    ContactPhase.FRONT_AXLE_ON_RAMP,
                    ContactPhase.MIXED_CONTACT_ENTRY,
                }
                and terrain.contact_phase == ContactPhase.FULL_SLOPE
            ):
                # Wheel-center slip is discontinuous while the rear axle moves
                # between planes. Start a fresh sustained-slip episode only
                # after all four rolling constraints share the ramp plane.
                self.contacts.traction.reset()
            self.last_phase = terrain.contact_phase
        preliminary = self.planner.plan(
            state, dynamics, frames, terrain, self.config.commanded_speed_mps,
        )
        preliminary_slips = self.contacts.slip_ratios(
            state, frames, dynamics, self.config.commanded_speed_mps,
        )
        margin = min(
            preliminary.longitudinal_margin_m, preliminary.lateral_margin_m,
        )
        effective_speed = self._effective_speed(terrain, margin, preliminary_slips)
        if settling:
            effective_speed = 0.0
        target = self.planner.plan(
            state, dynamics, frames, terrain, effective_speed,
        )
        if self.effective_mode == 'monitor':
            result = self.qp.failed_result('MONITOR_NO_QP')
        else:
            qp_start = time.perf_counter()
            result = self.qp.solve(
                state, dynamics, frames, target, effective_speed, dt,
            )
            qp_ms = 1000.0 * (time.perf_counter() - qp_start)
        if self.effective_mode == 'monitor':
            qp_ms = 0.0
        if self.effective_mode == 'monitor':
            self.consecutive_solver_failures = 0
        elif result.success:
            self.consecutive_solver_failures = 0
            self.last_safe_result = result
        else:
            self.consecutive_solver_failures += 1
            fallback = complete_solution_or_fallback(
                result, self.last_safe_result,
                self.consecutive_solver_failures,
                self.config.maximum_consecutive_solver_failures,
            )
            if fallback is None:
                self._fault('WBC_SOLVER_FAULT:' + result.status, 'FAULT')
                return
            result = fallback
        friction, torque_utilization, friction_peak = self._solution_metrics(result)
        slips = preliminary_slips
        _, _, yaw = quaternion_to_rpy(state.base_quaternion)
        heading = np.asarray((math.cos(yaw), math.sin(yaw), 0.0))
        traction = self.contacts.traction.evaluate(
            now_ros,
            [
                state.v[self.robot.v_indices[name]]
                for name in WHEEL_JOINT_NAMES
            ],
            float(dynamics.com_velocity @ heading),
            effective_speed,
        )
        safety = self._safety_check(
            state, target, slips, result, friction_peak,
            torque_utilization, traction.state.value,
            traction.stop_reason, now_ros,
        )
        if safety:
            self._fault(safety)
            return
        if self.stop_requested:
            effective_speed = 0.0
        if self.config.mode == 'active' and self.effective_mode == 'active':
            try:
                if self.hybrid_adapter is not None:
                    if settling:
                        command = HybridCommand(
                            leg_positions=self.startup_hold_leg.copy(),
                            wheel_velocities=np.zeros(4),
                        )
                        self.hybrid_adapter.last_leg_command = (
                            self.startup_hold_leg.copy()
                        )
                        self.hybrid_adapter.last_wheel_command[:] = 0.0
                    else:
                        command = self.hybrid_adapter.propose(
                            state, result, effective_speed, dt,
                        )
                    self.last_leg_command = command.leg_positions
                    self.last_wheel_command = command.wheel_velocities
                    self.hybrid_adapter.publish(command)
                elif self.effort_adapter is not None:
                    self.effort_adapter.command(result.tau, dt)
            except ValueError as error:
                self._fault(str(error), 'FAULT')
                return
        elif self.hybrid_adapter is not None and self.effective_mode != 'monitor':
            command = self.hybrid_adapter.propose(
                state, result, effective_speed, dt,
            )
            self.last_leg_command = command.leg_positions
            self.last_wheel_command = command.wheel_velocities
        if self.stop_requested and np.max(np.abs(self.last_wheel_command)) <= 1.0e-3:
            self.enabled = False
            if self.final_result == 'CLIMB_SUCCESS':
                self.state = 'TOP_PLATFORM'
            else:
                self.state = 'STOPPED_SAFE'
                self.final_result = 'STOPPED'
            self._publish_safe_stop()
            if self.config.shutdown_after_top:
                self.exit_requested = True
        if terrain.contact_phase == ContactPhase.TOP_PLATFORM:
            self.state = 'TOP_PLATFORM'
            self.final_result = 'CLIMB_SUCCESS'
            if self.config.shutdown_after_top:
                self.stop_requested = True
        total_ms = 1000.0 * (time.perf_counter() - cycle_start)
        deadline_missed = total_ms > 1000.0 / self.config.control_rate_hz
        self.consecutive_deadline_misses = (
            self.consecutive_deadline_misses + 1 if deadline_missed else 0
        )
        if self.consecutive_deadline_misses >= self.config.maximum_deadline_misses:
            self._fault('REPEATED_QP_DEADLINE_MISS', 'FAULT')
            return
        self.last_state = state
        self.last_dynamics = dynamics
        self.last_contacts = frames
        self.last_target = target
        self.last_qp_result = result
        self._publish_monitoring(
            state, dynamics, target, result, terrain, slips,
            friction, effective_speed, deadline_missed,
        )
        timings = {
            'state': state_ms, 'dynamics': dynamics_ms,
            'qp': qp_ms, 'total': total_ms,
        }
        self._log_cycle(
            now_ros, state, dynamics, target, result, terrain,
            slips, friction, effective_speed, timings, deadline_missed,
        )
        roll, pitch, _ = quaternion_to_rpy(state.base_quaternion)
        self.logger.observe(
            result, math.degrees(pitch), math.degrees(roll),
            min(target.longitudinal_margin_m, target.lateral_margin_m),
            friction, slips, torque_utilization, deadline_missed,
        )
        if now_wall - self.last_status_wall >= self.config.status_period_sec:
            self.get_logger().info(
                f'state={self.state} phase={terrain.contact_phase.value} '
                f'slope={max(terrain.fast_slope_deg, terrain.stable_slope_deg):.1f}deg '
                f'speed={effective_speed:.3f}m/s margin={margin:.3f}m '
                f'qp={result.status} {1000.0 * result.runtime_sec:.2f}ms'
            )
            self.last_status_wall = now_wall

    def _publish_idle_diagnostics(self, readiness: str) -> None:
        now_wall = time.monotonic()
        if now_wall - self.last_status_wall < self.config.status_period_sec:
            return
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'go2w_inverse_dynamics_wbc_climb'
        status.hardware_id = 'go2w'
        status.level = DiagnosticStatus.WARN
        status.message = self.backend_failure_reason or readiness or self.state
        status.values = [
            KeyValue(key='mode', value=self.effective_mode),
            KeyValue(key='backend', value=self.config.backend),
            KeyValue(key='enabled', value='false'),
        ]
        diagnostics.status = [status]
        self.diagnostics_publisher.publish(diagnostics)
        for name, value in {
            'state': self.state,
            'backend': self.config.backend,
            'solver_status': 'DISABLED',
            'phase': self.contacts.terrain.contact_phase.value,
        }.items():
            message = String()
            message.data = value
            self.string_publishers[name].publish(message)
        enabled = Bool()
        enabled.data = False
        self.bool_publishers['enabled'].publish(enabled)
        self.last_status_wall = now_wall

    def shutdown_safely(self, result: str = 'INTERRUPTED') -> None:
        self._publish_safe_stop()
        self._stop_pose_bridge()
        top = self.contacts.terrain.contact_phase == ContactPhase.TOP_PLATFORM
        no_topple = self.safety_reason not in {'EXCESSIVE_ROLL', 'EXCESSIVE_PITCH'}
        final_result = (
            result if self.final_result in {'NOT_RUN', 'RUNNING'}
            else self.final_result
        )
        final_reason = self.safety_reason or (
            final_result if final_result == 'CLIMB_SUCCESS' else result
        )
        self.logger.finalize(
            final_result,
            final_reason,
            self.interfaces.report(), top, no_topple, self.maximum_slope_seen,
        )
        self.get_logger().info(
            f'final result={final_result} reason={final_reason} '
            f'csv={self.logger.csv_path} json={self.logger.json_path}'
        )


def main(args: Sequence[str] | None = None) -> None:
    """Run as a direct Python ROS 2 executable."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Go2WInverseDynamicsWbcNode | None = None
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = Go2WInverseDynamicsWbcNode()
        while rclpy.ok() and not stop_requested and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.10)
    except (ValueError, FileNotFoundError, ImportError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'go2w_inverse_dynamics_wbc_climb: {error}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.shutdown_safely()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
