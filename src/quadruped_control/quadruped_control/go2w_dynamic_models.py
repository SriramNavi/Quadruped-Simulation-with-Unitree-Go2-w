"""Typed, ROS-independent models for adaptive Go2-W ramp climbing."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Sequence


LEGS = ('FL', 'FR', 'RL', 'RR')


class StringEnum(str, Enum):
    """String enum that serializes cleanly in ROS messages and logs."""

    def __str__(self) -> str:
        """Return the serialized enum value."""
        return self.value


class ControllerMode(StringEnum):
    """Available command-authority modes."""

    MONITOR = 'monitor'
    SHADOW = 'shadow'
    ACTIVE = 'active'


class ClimbState(StringEnum):
    """High-level adaptive climb states."""

    WAITING_FOR_DATA = 'WAITING_FOR_DATA'
    READY = 'READY'
    STARTUP_SETTLE = 'STARTUP_SETTLE'
    FLAT_DRIVE = 'FLAT_DRIVE'
    RAMP_APPROACH = 'RAMP_APPROACH'
    MIXED_CONTACT_ENTRY = 'MIXED_CONTACT_ENTRY'
    CLIMB = 'CLIMB'
    MIXED_CONTACT_EXIT = 'MIXED_CONTACT_EXIT'
    TOP_TRANSITION = 'TOP_TRANSITION'
    TOP_HOLD = 'TOP_HOLD'
    COMPLETE = 'COMPLETE'
    SAFE_INFEASIBLE = 'SAFE_INFEASIBLE'
    FAULT = 'FAULT'
    TIMEOUT = 'TIMEOUT'


class ContactPhase(StringEnum):
    """Estimated wheel-contact transition phases."""

    FLAT_APPROACH = 'FLAT_APPROACH'
    ENTRY_DETECTED = 'ENTRY_DETECTED'
    FRONT_AXLE_ON_RAMP = 'FRONT_AXLE_ON_RAMP'
    MIXED_CONTACT_ENTRY = 'MIXED_CONTACT_ENTRY'
    FULL_SLOPE = 'FULL_SLOPE'
    CREST_DETECTED = 'CREST_DETECTED'
    FRONT_AXLE_ON_TOP = 'FRONT_AXLE_ON_TOP'
    MIXED_CONTACT_EXIT = 'MIXED_CONTACT_EXIT'
    TOP_PLATFORM = 'TOP_PLATFORM'


class SafetyState(StringEnum):
    """Shared safety decision severity."""

    SAFE = 'SAFE'
    CAUTION = 'CAUTION'
    STOP = 'STOP'
    INFEASIBLE = 'INFEASIBLE'


@dataclass(frozen=True)
class SensorSnapshot:
    """Synchronized finite sensor values used by pure control logic."""

    stamp_sec: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    roll_rate_radps: float
    pitch_rate_radps: float
    yaw_rate_radps: float
    vertical_accel_mps2: float
    base_x_m: float
    base_y_m: float
    base_z_m: float
    base_vx_mps: float
    base_vy_mps: float
    base_vz_mps: float
    joint_positions: Mapping[str, float] = field(default_factory=dict)
    wheel_velocities_radps: Sequence[float] = field(default_factory=tuple)


@dataclass(frozen=True)
class TerrainEstimate:
    """Slope, confidence, progress, and contact-phase estimate."""

    forward_progress_m: float = 0.0
    height_gain_m: float = 0.0
    raw_slope_deg: float = 0.0
    fast_slope_deg: float = 0.0
    stable_slope_deg: float = 0.0
    slope_confidence: float = 0.0
    slope_rate_degps: float = 0.0
    transition_probability: float = 0.0
    contact_phase: ContactPhase = ContactPhase.FLAT_APPROACH
    valid: bool = False
    stale: bool = True


@dataclass(frozen=True)
class AttitudeCommand:
    """Separated adaptive attitude targets and control terms."""

    desired_level_pitch_deg: float = 0.0
    feasible_target_pitch_deg: float = 0.0
    measured_pitch_deg: float = 0.0
    feedforward_deg: float = 0.0
    feedback_deg: float = 0.0
    final_correction_deg: float = 0.0
    desired_roll_deg: float = 0.0
    measured_roll_deg: float = 0.0
    roll_correction_deg: float = 0.0
    pitch_error_deg: float = 0.0
    p_term_deg: float = 0.0
    i_term_deg: float = 0.0
    d_term_deg: float = 0.0
    saturated: bool = False


@dataclass(frozen=True)
class FeasibilityResult:
    """Current all-four-leg feasible correction envelope."""

    valid: bool = False
    min_correction_deg: float = 0.0
    max_correction_deg: float = 0.0
    max_front_offset_m: float = 0.0
    max_rear_offset_m: float = 0.0
    max_roll_correction_deg: float = 0.0
    joint_margin_rad: float = 0.0
    workspace_margin_m: float = 0.0
    workspace_usage: float = 0.0
    limiting_leg: str = ''
    limiting_joint: str = ''
    limiting_reason: str = ''
    recomputed: bool = False


@dataclass(frozen=True)
class StabilityResult:
    """Approximate quasi-static support-margin decision."""

    longitudinal_margin_m: float = 0.0
    lateral_margin_m: float = 0.0
    minimum_margin_m: float = 0.0
    projected_com_x_m: float = 0.0
    projected_com_y_m: float = 0.0
    state: SafetyState = SafetyState.STOP
    stop_reason: str = ''
    approximation: str = 'base_inertial_com_quasi_static'


@dataclass(frozen=True)
class TractionResult:
    """Per-wheel slip observables and traction decision."""

    slips: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    average_abs_slip: float = 0.0
    peak_abs_slip: float = 0.0
    front_average_abs_slip: float = 0.0
    rear_average_abs_slip: float = 0.0
    confidence: float = 0.0
    state: SafetyState = SafetyState.SAFE
    stop_reason: str = ''


@dataclass(frozen=True)
class MotionCommand:
    """Scheduled body speed, wheel commands, and scale factors."""

    requested_speed_mps: float = 0.0
    effective_speed_mps: float = 0.0
    wheel_linear_mps: tuple[float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0,
    )
    wheel_radps: tuple[float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0,
    )
    slope_scale: float = 1.0
    transition_scale: float = 1.0
    pitch_scale: float = 1.0
    stability_scale: float = 1.0
    traction_scale: float = 1.0
    workspace_scale: float = 1.0
    paused: bool = False
    stop_reason: str = ''


@dataclass(frozen=True)
class JointCommandResult:
    """Atomic all-four-leg IK command result."""

    valid: bool
    positions: tuple[float, ...] = ()
    solutions: Mapping[str, object] = field(default_factory=dict)
    applied_pitch_correction_deg: float = 0.0
    applied_roll_correction_deg: float = 0.0
    front_offset_m: float = 0.0
    rear_offset_m: float = 0.0
    left_offset_m: float = 0.0
    right_offset_m: float = 0.0
    scale: float = 0.0
    joint_margin_rad: float = 0.0
    limiting_leg: str = ''
    limiting_joint: str = ''
    limiting_reason: str = ''


@dataclass
class RunMetrics:
    """Accumulated finite metrics used in final run summaries."""

    pitch_errors_deg: list[float] = field(default_factory=list)
    pitch_values_deg: list[float] = field(default_factory=list)
    roll_values_deg: list[float] = field(default_factory=list)
    speeds_mps: list[float] = field(default_factory=list)
    slips: list[float] = field(default_factory=list)
    phase_durations_sec: dict[str, float] = field(default_factory=dict)
    minimum_stability_margin_m: float = math.inf
    peak_abs_slip: float = 0.0
    maximum_workspace_usage: float = 0.0
    maximum_joint_limit_usage: float = 0.0
    entry_minimum_speed_mps: float = math.inf
    climb_speeds_mps: list[float] = field(default_factory=list)

    @staticmethod
    def mean(values: Sequence[float]) -> float | None:
        """Return the arithmetic mean or None for an empty sequence."""
        return sum(values) / len(values) if values else None
