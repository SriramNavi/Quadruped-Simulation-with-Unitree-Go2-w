"""Conservative quasi-static stability and wheel-traction supervision."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

from quadruped_control.go2w_dynamic_models import (
    ContactPhase,
    LEGS,
    SafetyState,
    StabilityResult,
    TractionResult,
)


@dataclass(frozen=True)
class StabilityConfig:
    """URDF base-COM approximation and conservative safety thresholds."""

    base_com_x_m: float = 0.021112
    base_com_y_m: float = 0.0
    base_com_z_m: float = -0.005366
    wheel_radius_m: float = 0.086
    caution_margin_m: float = 0.045
    stop_margin_m: float = 0.015
    mixed_contact_inset_m: float = 0.012
    maximum_pitch_rate_radps: float = 1.8
    maximum_roll_rate_radps: float = 1.8

    def validate(self) -> None:
        """Reject inconsistent stability settings."""
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError('stability configuration must be finite')
        if self.stop_margin_m < 0.0:
            raise ValueError('stability stop margin must be non-negative')
        if self.caution_margin_m <= self.stop_margin_m:
            raise ValueError('caution margin must exceed stop margin')


class StabilitySupervisor:
    """Project the URDF base inertial COM into the current support region.

    This is deliberately named an approximate base-COM quasi-static model. It
    does not estimate contact forces or the mass-weighted articulated-body COM.
    """

    _MIXED_PHASES = {
        ContactPhase.FRONT_AXLE_ON_RAMP,
        ContactPhase.MIXED_CONTACT_ENTRY,
        ContactPhase.FRONT_AXLE_ON_TOP,
        ContactPhase.MIXED_CONTACT_EXIT,
    }

    def __init__(self, config: StabilityConfig) -> None:
        """Store validated stability geometry and thresholds."""
        config.validate()
        self.config = config

    def evaluate(
        self,
        foot_positions: Mapping[str, Sequence[float]],
        roll_rad: float,
        pitch_rad: float,
        roll_rate_radps: float,
        pitch_rate_radps: float,
        phase: ContactPhase,
    ) -> StabilityResult:
        """Evaluate projected base-COM margins against wheel contacts."""
        if set(foot_positions) != set(LEGS):
            return StabilityResult(
                state=SafetyState.INFEASIBLE,
                stop_reason='incomplete_support_geometry',
            )
        contacts: list[tuple[float, float, float]] = []
        for leg in LEGS:
            point = tuple(float(value) for value in foot_positions[leg])
            if (
                len(point) != 3
                or not all(math.isfinite(value) for value in point)
            ):
                return StabilityResult(
                    state=SafetyState.INFEASIBLE,
                    stop_reason='invalid_support_geometry',
                )
            hip_x = 0.1934 if leg.startswith('F') else -0.1934
            hip_y = 0.0465 if leg.endswith('L') else -0.0465
            contacts.append((
                hip_x + point[0],
                hip_y + point[1],
                point[2] - self.config.wheel_radius_m,
            ))
        min_x = min(point[0] for point in contacts)
        max_x = max(point[0] for point in contacts)
        min_y = min(point[1] for point in contacts)
        max_y = max(point[1] for point in contacts)
        if phase in self._MIXED_PHASES:
            inset = self.config.mixed_contact_inset_m
            min_x += inset
            max_x -= inset

        # World-down expressed in body coordinates for yaw-independent RPY.
        gx = math.sin(pitch_rad)
        gy = -math.sin(roll_rad) * math.cos(pitch_rad)
        gz = -math.cos(roll_rad) * math.cos(pitch_rad)
        support_z = sum(point[2] for point in contacts) / len(contacts)
        if (
            not all(math.isfinite(v) for v in (gx, gy, gz))
            or abs(gz) < 1.0e-5
        ):
            return StabilityResult(
                state=SafetyState.INFEASIBLE,
                stop_reason='invalid_gravity_projection',
            )
        distance = (support_z - self.config.base_com_z_m) / gz
        projected_x = self.config.base_com_x_m + distance * gx
        projected_y = self.config.base_com_y_m + distance * gy
        longitudinal = min(projected_x - min_x, max_x - projected_x)
        lateral = min(projected_y - min_y, max_y - projected_y)
        minimum = min(longitudinal, lateral)
        stop_reason = ''
        if minimum < 0.0:
            state = SafetyState.INFEASIBLE
            stop_reason = 'projected_com_outside_support'
        elif minimum <= self.config.stop_margin_m:
            state = SafetyState.STOP
            stop_reason = 'stability_margin_below_stop'
        elif (
            abs(pitch_rate_radps) > self.config.maximum_pitch_rate_radps
            or abs(roll_rate_radps) > self.config.maximum_roll_rate_radps
        ):
            state = SafetyState.STOP
            stop_reason = 'excessive_attitude_rate'
        elif minimum <= self.config.caution_margin_m:
            state = SafetyState.CAUTION
            stop_reason = 'stability_margin_caution'
        else:
            state = SafetyState.SAFE
        return StabilityResult(
            longitudinal_margin_m=longitudinal,
            lateral_margin_m=lateral,
            minimum_margin_m=minimum,
            projected_com_x_m=projected_x,
            projected_com_y_m=projected_y,
            state=state,
            stop_reason=stop_reason,
        )


@dataclass(frozen=True)
class TractionConfig:
    """Wheel normalization, observability, and sustained-slip settings."""

    wheel_radius_m: float = 0.086
    wheel_forward_signs: tuple[float, float, float, float] = (
        1.0, 1.0, 1.0, 1.0,
    )
    slip_epsilon_mps: float = 0.025
    caution_threshold: float = 0.30
    stop_threshold: float = 0.55
    slip_hold_sec: float = 0.40
    infeasible_hold_sec: float = 1.50

    def validate(self) -> None:
        """Reject invalid traction geometry or thresholds."""
        if (
            not math.isfinite(self.wheel_radius_m)
            or self.wheel_radius_m <= 0.0
        ):
            raise ValueError('wheel radius must be positive')
        if any(
            value not in (-1.0, 1.0)
            for value in self.wheel_forward_signs
        ):
            raise ValueError('wheel forward signs must be -1 or 1')
        if not 0.0 < self.caution_threshold < self.stop_threshold:
            raise ValueError('invalid slip thresholds')
        if self.slip_hold_sec <= 0.0 or self.infeasible_hold_sec <= 0.0:
            raise ValueError('slip hold durations must be positive')


class TractionSupervisor:
    """Compare normalized wheel surface speed with measured base speed."""

    def __init__(self, config: TractionConfig) -> None:
        """Initialize the sustained-slip timer."""
        config.validate()
        self.config = config
        self._high_slip_duration_sec = 0.0
        self._last_stamp_sec: float | None = None

    def reset(self) -> None:
        """Clear sustained-slip history at a run boundary."""
        self._high_slip_duration_sec = 0.0
        self._last_stamp_sec = None

    def evaluate(
        self,
        stamp_sec: float,
        wheel_velocities_radps: Sequence[float],
        base_forward_speed_mps: float,
        requested_speed_mps: float,
    ) -> TractionResult:
        """Calculate finite per-wheel slip and a held safety decision."""
        if len(wheel_velocities_radps) != 4:
            return TractionResult(
                state=SafetyState.STOP,
                stop_reason='missing_wheel_velocity',
            )
        values = (
            stamp_sec, base_forward_speed_mps, requested_speed_mps,
            *wheel_velocities_radps,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return TractionResult(
                state=SafetyState.STOP,
                stop_reason='invalid_wheel_velocity',
            )
        surface = tuple(
            self.config.wheel_radius_m * float(omega) * sign
            for omega, sign in zip(
                wheel_velocities_radps, self.config.wheel_forward_signs,
            )
        )
        driven_speed = max(
            abs(requested_speed_mps), *(abs(value) for value in surface),
        )
        sample_dt = (
            0.0 if self._last_stamp_sec is None
            else max(0.0, min(0.10, stamp_sec - self._last_stamp_sec))
        )
        self._last_stamp_sec = stamp_sec
        if abs(requested_speed_mps) < self.config.slip_epsilon_mps:
            # A supervisor-requested pause is not new evidence of slip. Keep
            # the episode open so a failed retry can still escalate, but do
            # not age it into STOP/INFEASIBLE from passive body recoil while
            # the wheels are stopped.
            if self._high_slip_duration_sec > 0.0:
                return TractionResult(
                    confidence=0.0,
                    state=SafetyState.CAUTION,
                    stop_reason='traction_recovery_pause',
                )
            return TractionResult(confidence=0.0)
        observable_speed = max(abs(base_forward_speed_mps), driven_speed)
        if observable_speed < self.config.slip_epsilon_mps:
            self._high_slip_duration_sec = 0.0
            return TractionResult(confidence=0.0)
        slips = tuple(
            (value - base_forward_speed_mps) / max(
                abs(value), abs(base_forward_speed_mps),
                self.config.slip_epsilon_mps,
            )
            for value in surface
        )
        absolute = tuple(abs(value) for value in slips)
        average = sum(absolute) / 4.0
        peak = max(absolute)
        stop_metric = max(average, 0.75 * peak)
        state = SafetyState.SAFE
        reason = ''
        if stop_metric >= self.config.stop_threshold:
            self._high_slip_duration_sec += sample_dt
            if (
                self._high_slip_duration_sec
                >= self.config.infeasible_hold_sec
            ):
                state = SafetyState.INFEASIBLE
                reason = 'sustained_inadequate_traction'
            elif self._high_slip_duration_sec >= self.config.slip_hold_sec:
                state = SafetyState.STOP
                reason = 'sustained_wheel_slip'
            else:
                state = SafetyState.CAUTION
                reason = 'wheel_slip_rising'
        elif stop_metric >= self.config.caution_threshold:
            self._high_slip_duration_sec = 0.0
            state = SafetyState.CAUTION
            reason = 'wheel_slip_caution'
        else:
            self._high_slip_duration_sec = 0.0
        confidence = min(1.0, observable_speed / 0.10)
        return TractionResult(
            slips=slips,
            average_abs_slip=average,
            peak_abs_slip=peak,
            front_average_abs_slip=0.5 * (absolute[0] + absolute[1]),
            rear_average_abs_slip=0.5 * (absolute[2] + absolute[3]),
            confidence=confidence,
            state=state,
            stop_reason=reason,
        )
