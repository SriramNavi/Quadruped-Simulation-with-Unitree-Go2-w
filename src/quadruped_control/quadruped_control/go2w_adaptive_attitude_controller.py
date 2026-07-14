"""Adaptive terrain-feedforward and IMU-feedback attitude controller."""

from __future__ import annotations

import math
from dataclasses import dataclass

from quadruped_control.go2w_dynamic_models import (
    AttitudeCommand,
    ClimbState,
    FeasibilityResult,
    SafetyState,
    StabilityResult,
    TerrainEstimate,
)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to inclusive bounds."""
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class AttitudeControllerConfig:
    """Validated filtering, gain, target, and hard-cap settings."""

    desired_level_pitch_deg: float = 0.0
    desired_roll_deg: float = 0.0
    adaptive_target_enabled: bool = True
    terrain_feedforward_enabled: bool = True
    roll_control_enabled: bool = False
    feedforward_gain: float = 1.0
    pitch_kp_flat: float = 0.55
    pitch_kp_entry: float = 0.75
    pitch_kp_climb: float = 0.65
    pitch_kd_flat: float = 0.08
    pitch_kd_entry: float = 0.12
    pitch_kd_climb: float = 0.10
    pitch_ki: float = 0.0
    integral_limit_deg_sec: float = 8.0
    roll_kp: float = 0.30
    roll_kd: float = 0.06
    pitch_filter_cutoff_hz: float = 5.0
    roll_filter_cutoff_hz: float = 4.0
    absolute_correction_cap_deg: float = 25.0
    absolute_roll_cap_deg: float = 4.0

    def validate(self) -> None:
        """Reject non-finite or negative controller settings."""
        values = (
            self.desired_level_pitch_deg, self.desired_roll_deg,
            self.feedforward_gain, self.pitch_kp_flat, self.pitch_kp_entry,
            self.pitch_kp_climb, self.pitch_kd_flat, self.pitch_kd_entry,
            self.pitch_kd_climb, self.pitch_ki, self.integral_limit_deg_sec,
            self.roll_kp, self.roll_kd, self.pitch_filter_cutoff_hz,
            self.roll_filter_cutoff_hz, self.absolute_correction_cap_deg,
            self.absolute_roll_cap_deg,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('attitude configuration must be finite')
        if any(value < 0.0 for value in values[2:]):
            raise ValueError('attitude gains and limits must be non-negative')


class AdaptiveAttitudeController:
    """Generate the closest feasible pitch target and safe correction."""

    _ENTRY_STATES = {
        ClimbState.RAMP_APPROACH,
        ClimbState.MIXED_CONTACT_ENTRY,
        ClimbState.MIXED_CONTACT_EXIT,
        ClimbState.TOP_TRANSITION,
    }

    def __init__(self, config: AttitudeControllerConfig) -> None:
        """Initialize filter and integral state."""
        config.validate()
        self.config = config
        self.filtered_pitch_rad = 0.0
        self.filtered_roll_rad = 0.0
        self.raw_pitch_rad = 0.0
        self.raw_roll_rad = 0.0
        self.pitch_rate_radps = 0.0
        self.roll_rate_radps = 0.0
        self.integral_deg_sec = 0.0
        self._last_stamp: float | None = None
        self.valid = False

    @staticmethod
    def _filter_alpha(cutoff_hz: float, dt: float) -> float:
        return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)

    def update_imu(
        self,
        stamp_sec: float,
        pitch_rad: float,
        roll_rad: float,
        pitch_rate_radps: float,
        roll_rate_radps: float,
    ) -> None:
        """Filter one valid IMU attitude sample."""
        values = (
            stamp_sec, pitch_rad, roll_rad, pitch_rate_radps, roll_rate_radps,
        )
        if not all(math.isfinite(value) for value in values):
            self.valid = False
            raise ValueError('IMU attitude values must be finite')
        if self._last_stamp is not None and stamp_sec < self._last_stamp:
            self.valid = False
            raise ValueError('IMU timestamp moved backward')
        dt = None if self._last_stamp is None else stamp_sec - self._last_stamp
        if not self.valid or dt is None or dt <= 0.0 or dt > 0.10:
            self.filtered_pitch_rad = pitch_rad
            self.filtered_roll_rad = roll_rad
        else:
            pitch_alpha = self._filter_alpha(
                self.config.pitch_filter_cutoff_hz, dt,
            )
            roll_alpha = self._filter_alpha(
                self.config.roll_filter_cutoff_hz, dt,
            )
            self.filtered_pitch_rad += pitch_alpha * (
                pitch_rad - self.filtered_pitch_rad
            )
            self.filtered_roll_rad += roll_alpha * (
                roll_rad - self.filtered_roll_rad
            )
        self.raw_pitch_rad = pitch_rad
        self.raw_roll_rad = roll_rad
        self.pitch_rate_radps = pitch_rate_radps
        self.roll_rate_radps = roll_rate_radps
        self._last_stamp = stamp_sec
        self.valid = True

    def reset_integral(self) -> None:
        """Clear integral state at run boundaries."""
        self.integral_deg_sec = 0.0

    def _gains(self, state: ClimbState) -> tuple[float, float]:
        if state in self._ENTRY_STATES:
            return self.config.pitch_kp_entry, self.config.pitch_kd_entry
        if state == ClimbState.CLIMB:
            return self.config.pitch_kp_climb, self.config.pitch_kd_climb
        return self.config.pitch_kp_flat, self.config.pitch_kd_flat

    def compute(
        self,
        terrain: TerrainEstimate,
        feasibility: FeasibilityResult,
        stability: StabilityResult,
        state: ClimbState,
        dt_sec: float,
    ) -> AttitudeCommand:
        """Compute same-cycle feedforward plus residual feedback."""
        if not self.valid:
            return AttitudeCommand()
        cfg = self.config
        measured_pitch = math.degrees(self.filtered_pitch_rad)
        measured_roll = math.degrees(self.filtered_roll_rad)
        positive_slope = max(0.0, terrain.fast_slope_deg)
        if state == ClimbState.CLIMB and terrain.valid:
            positive_slope = max(0.0, terrain.stable_slope_deg)
        confidence = terrain.slope_confidence if terrain.valid else 0.0
        confidence_weighted_slope = positive_slope * confidence

        positive_capability = max(0.0, feasibility.max_correction_deg)
        negative_capability = min(0.0, feasibility.min_correction_deg)
        safe_positive_capability = min(
            positive_capability, cfg.absolute_correction_cap_deg,
        )
        required_correction = max(
            0.0, confidence_weighted_slope - cfg.desired_level_pitch_deg,
        )
        feasible_terrain_correction = min(
            required_correction, safe_positive_capability,
        )
        if cfg.adaptive_target_enabled:
            uncompensated = max(
                0.0, confidence_weighted_slope - feasible_terrain_correction,
            )
            feasible_target = cfg.desired_level_pitch_deg - uncompensated
        else:
            feasible_target = cfg.desired_level_pitch_deg
        feedforward = (
            cfg.feedforward_gain * feasible_terrain_correction
            if cfg.terrain_feedforward_enabled else 0.0
        )

        pitch_error = feasible_target - measured_pitch
        kp, kd = self._gains(state)
        slope_gain = clamp(1.0 - 0.004 * positive_slope, 0.75, 1.0)
        stability_gain = (
            0.75 if stability.state == SafetyState.CAUTION else 1.0
        )
        kp *= slope_gain * stability_gain
        kd *= clamp(1.0 + 0.003 * positive_slope, 1.0, 1.15)
        p_term = kp * pitch_error
        d_term = -kd * math.degrees(self.pitch_rate_radps)
        integral_candidate = clamp(
            self.integral_deg_sec + pitch_error * max(0.0, dt_sec),
            -cfg.integral_limit_deg_sec,
            cfg.integral_limit_deg_sec,
        )
        i_candidate = cfg.pitch_ki * integral_candidate
        unconstrained = feedforward + p_term + i_candidate + d_term
        lower = max(negative_capability, -cfg.absolute_correction_cap_deg)
        upper = safe_positive_capability
        final = clamp(unconstrained, lower, upper)
        saturated = not math.isclose(final, unconstrained, abs_tol=1.0e-9)
        if cfg.pitch_ki > 0.0 and (
            not saturated or pitch_error * unconstrained < 0.0
        ):
            self.integral_deg_sec = integral_candidate
        elif cfg.pitch_ki <= 0.0:
            self.integral_deg_sec = 0.0
        i_term = cfg.pitch_ki * self.integral_deg_sec
        feedback = p_term + i_term + d_term
        final = clamp(feedforward + feedback, lower, upper)

        roll_correction = 0.0
        if cfg.roll_control_enabled:
            roll_error = cfg.desired_roll_deg - measured_roll
            roll_correction = clamp(
                cfg.roll_kp * roll_error
                - cfg.roll_kd * math.degrees(self.roll_rate_radps),
                -min(
                    cfg.absolute_roll_cap_deg,
                    feasibility.max_roll_correction_deg,
                ),
                min(
                    cfg.absolute_roll_cap_deg,
                    feasibility.max_roll_correction_deg,
                ),
            )
            if stability.state != SafetyState.SAFE:
                roll_correction *= 0.5

        return AttitudeCommand(
            desired_level_pitch_deg=cfg.desired_level_pitch_deg,
            feasible_target_pitch_deg=feasible_target,
            measured_pitch_deg=measured_pitch,
            feedforward_deg=feedforward,
            feedback_deg=feedback,
            final_correction_deg=final,
            desired_roll_deg=cfg.desired_roll_deg,
            measured_roll_deg=measured_roll,
            roll_correction_deg=roll_correction,
            pitch_error_deg=pitch_error,
            p_term_deg=p_term,
            i_term_deg=i_term,
            d_term_deg=d_term,
            saturated=saturated,
        )
