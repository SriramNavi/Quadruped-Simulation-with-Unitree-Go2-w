"""Adaptive forward-motion scheduling and pure climb state sequencing."""

from __future__ import annotations

import math
from dataclasses import dataclass

from quadruped_control.go2w_dynamic_models import (
    ClimbState,
    ContactPhase,
    FeasibilityResult,
    MotionCommand,
    SafetyState,
    StabilityResult,
    TerrainEstimate,
    TractionResult,
)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to inclusive bounds."""
    return max(lower, min(upper, value))


def rate_limit(current: float, target: float, rate: float, dt: float) -> float:
    """Apply a symmetric scalar rate bound."""
    change = max(0.0, rate * dt)
    return current + clamp(target - current, -change, change)


@dataclass(frozen=True)
class MotionSupervisorConfig:
    """Adaptive speed scales, steering gains, and wheel-rate limits."""

    requested_speed_mps: float = 0.10
    wheel_radius_m: float = 0.086
    wheel_accel_limit_mps2: float = 0.15
    wheel_decel_limit_mps2: float = 0.30
    yaw_kp: float = 0.08
    yaw_kd: float = 0.01
    maximum_yaw_correction_mps: float = 0.025
    entry_speed_scale: float = 0.45
    crest_speed_scale: float = 0.40
    minimum_speed_scale: float = 0.25
    slope_speed_gain: float = 0.018
    pitch_error_speed_gain: float = 0.10
    workspace_speed_gain: float = 0.35
    stability_caution_margin_m: float = 0.045
    stability_stop_margin_m: float = 0.015
    slip_caution_threshold: float = 0.30
    slip_stop_threshold: float = 0.55
    safe_pause_hold_sec: float = 0.35

    def validate(self) -> None:
        """Reject invalid scheduler gains, scales, or thresholds."""
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError(
                'motion configuration must be finite and non-negative'
            )
        for value in (
            self.entry_speed_scale, self.crest_speed_scale,
            self.minimum_speed_scale,
        ):
            if value > 1.0:
                raise ValueError('motion scales must be in [0, 1]')
        if self.stability_caution_margin_m <= self.stability_stop_margin_m:
            raise ValueError('invalid stability scaling margins')
        if self.slip_stop_threshold <= self.slip_caution_threshold:
            raise ValueError('invalid traction scaling thresholds')


class MotionSupervisor:
    """Choose a safe speed no greater than the user-requested maximum."""

    _ENTRY_PHASES = {
        ContactPhase.ENTRY_DETECTED,
        ContactPhase.FRONT_AXLE_ON_RAMP,
        ContactPhase.MIXED_CONTACT_ENTRY,
    }
    _EXIT_PHASES = {
        ContactPhase.CREST_DETECTED,
        ContactPhase.FRONT_AXLE_ON_TOP,
        ContactPhase.MIXED_CONTACT_EXIT,
    }

    def __init__(self, config: MotionSupervisorConfig) -> None:
        """Initialize speed, pause, and wheel-command state."""
        config.validate()
        self.config = config
        self._speed_mps = 0.0
        self._wheel_linear = [0.0] * 4
        self._safe_since: float | None = None
        self._paused = False

    def reset(self) -> None:
        """Clear speed and pause history at a run boundary."""
        self._speed_mps = 0.0
        self._wheel_linear = [0.0] * 4
        self._safe_since = None
        self._paused = False

    def update(
        self,
        now_sec: float,
        dt_sec: float,
        terrain: TerrainEstimate,
        pitch_error_deg: float,
        stability: StabilityResult,
        traction: TractionResult,
        feasibility: FeasibilityResult,
        heading_error_rad: float,
        yaw_rate_radps: float,
        drive_enabled: bool,
    ) -> MotionCommand:
        """Schedule and rate-limit one safe non-reversing wheel command."""
        cfg = self.config
        slope = max(0.0, terrain.stable_slope_deg)
        if terrain.contact_phase in self._ENTRY_PHASES:
            slope = max(slope, terrain.fast_slope_deg)
        slope_scale = clamp(
            1.0 / (1.0 + cfg.slope_speed_gain * slope), 0.0, 1.0,
        )
        if terrain.contact_phase in self._ENTRY_PHASES:
            transition_scale = cfg.entry_speed_scale
        elif terrain.contact_phase in self._EXIT_PHASES:
            transition_scale = cfg.crest_speed_scale
        else:
            transition_scale = 1.0
        pitch_scale = clamp(
            1.0 / (1.0 + cfg.pitch_error_speed_gain * abs(pitch_error_deg)),
            0.0, 1.0,
        )
        if stability.state in (SafetyState.STOP, SafetyState.INFEASIBLE):
            stability_scale = 0.0
        else:
            stability_scale = clamp(
                (stability.minimum_margin_m - cfg.stability_stop_margin_m)
                / (
                    cfg.stability_caution_margin_m
                    - cfg.stability_stop_margin_m
                ),
                0.0, 1.0,
            )
        slip = traction.average_abs_slip
        if traction.state in (SafetyState.STOP, SafetyState.INFEASIBLE):
            traction_scale = 0.0
        else:
            traction_scale = clamp(
                (cfg.slip_stop_threshold - slip)
                / (cfg.slip_stop_threshold - cfg.slip_caution_threshold),
                0.0, 1.0,
            )
            if traction.state == SafetyState.CAUTION:
                traction_scale = max(0.15, traction_scale)
        workspace_scale = clamp(
            1.0 - cfg.workspace_speed_gain * feasibility.workspace_usage ** 2,
            cfg.minimum_speed_scale, 1.0,
        )
        stop_reason = stability.stop_reason or traction.stop_reason
        safe_now = stability_scale > 0.0 and traction_scale > 0.0
        if not safe_now:
            self._paused = True
            self._safe_since = None
        elif self._paused:
            self._safe_since = self._safe_since or now_sec
            if now_sec - self._safe_since >= cfg.safe_pause_hold_sec:
                self._paused = False
                self._safe_since = None
        product = (
            slope_scale * transition_scale * pitch_scale
            * stability_scale * traction_scale * workspace_scale
        )
        if product > 0.0:
            product = max(cfg.minimum_speed_scale, product)
        target = cfg.requested_speed_mps * product
        if not drive_enabled or self._paused:
            target = 0.0
        rate = (
            cfg.wheel_accel_limit_mps2
            if target > self._speed_mps else cfg.wheel_decel_limit_mps2
        )
        self._speed_mps = rate_limit(self._speed_mps, target, rate, dt_sec)
        yaw_correction = clamp(
            cfg.yaw_kp * heading_error_rad - cfg.yaw_kd * yaw_rate_radps,
            -cfg.maximum_yaw_correction_mps,
            cfg.maximum_yaw_correction_mps,
        )
        yaw_correction = clamp(
            yaw_correction, -self._speed_mps, self._speed_mps,
        )
        left_target = max(0.0, self._speed_mps - yaw_correction)
        right_target = max(0.0, self._speed_mps + yaw_correction)
        targets = (left_target, right_target, left_target, right_target)
        for index, wheel_target in enumerate(targets):
            wheel_rate = (
                cfg.wheel_accel_limit_mps2
                if wheel_target > self._wheel_linear[index]
                else cfg.wheel_decel_limit_mps2
            )
            self._wheel_linear[index] = rate_limit(
                self._wheel_linear[index], wheel_target, wheel_rate, dt_sec,
            )
        wheel_linear = tuple(self._wheel_linear)
        wheel_radps = tuple(
            value / cfg.wheel_radius_m for value in wheel_linear
        )
        return MotionCommand(
            requested_speed_mps=cfg.requested_speed_mps,
            effective_speed_mps=self._speed_mps,
            wheel_linear_mps=wheel_linear,
            wheel_radps=wheel_radps,
            slope_scale=slope_scale,
            transition_scale=transition_scale,
            pitch_scale=pitch_scale,
            stability_scale=stability_scale,
            traction_scale=traction_scale,
            workspace_scale=workspace_scale,
            paused=self._paused,
            stop_reason=stop_reason,
        )


@dataclass(frozen=True)
class ClimbStateMachineConfig:
    """High-level settle, hold, and timeout durations."""

    startup_settle_sec: float = 2.0
    top_hold_sec: float = 3.0
    maximum_run_time_sec: float = 120.0


class AdaptiveClimbStateMachine:
    """Pure high-level sequencing driven by the contact-phase estimator."""

    TERMINAL = {
        ClimbState.COMPLETE,
        ClimbState.SAFE_INFEASIBLE,
        ClimbState.FAULT,
        ClimbState.TIMEOUT,
    }

    def __init__(self, config: ClimbStateMachineConfig) -> None:
        """Initialize in the data-wait state."""
        self.config = config
        self.state = ClimbState.WAITING_FOR_DATA
        self.state_enter_sec = 0.0
        self.run_start_sec: float | None = None

    def transition(self, state: ClimbState, now_sec: float) -> bool:
        """Enter a different state and report whether it changed."""
        if state == self.state:
            return False
        self.state = state
        self.state_enter_sec = now_sec
        if state == ClimbState.STARTUP_SETTLE:
            self.run_start_sec = now_sec
        return True

    def update(
        self,
        now_sec: float,
        phase: ContactPhase,
        start_requested: bool = False,
        data_ready: bool = False,
        level_start: bool = True,
        wheels_stopped: bool = False,
        posture_neutral: bool = False,
        infeasible: bool = False,
        fault: bool = False,
    ) -> ClimbState:
        """Advance the deterministic high-level state sequence."""
        if self.state in self.TERMINAL:
            return self.state
        if fault:
            self.transition(ClimbState.FAULT, now_sec)
            return self.state
        if infeasible:
            self.transition(ClimbState.SAFE_INFEASIBLE, now_sec)
            return self.state
        if (
            self.run_start_sec is not None
            and now_sec - self.run_start_sec
            > self.config.maximum_run_time_sec
        ):
            self.transition(ClimbState.TIMEOUT, now_sec)
            return self.state
        if self.state == ClimbState.WAITING_FOR_DATA and data_ready:
            self.transition(ClimbState.READY, now_sec)
        elif self.state == ClimbState.READY and start_requested:
            self.transition(ClimbState.STARTUP_SETTLE, now_sec)
        elif self.state == ClimbState.STARTUP_SETTLE:
            if (
                now_sec - self.state_enter_sec
                >= self.config.startup_settle_sec
            ):
                self.transition(
                    ClimbState.FLAT_DRIVE if level_start else ClimbState.FAULT,
                    now_sec,
                )
        elif self.state == ClimbState.FLAT_DRIVE:
            if phase in (
                ContactPhase.ENTRY_DETECTED,
                ContactPhase.FRONT_AXLE_ON_RAMP,
            ):
                self.transition(ClimbState.RAMP_APPROACH, now_sec)
        elif self.state == ClimbState.RAMP_APPROACH:
            if phase in (
                ContactPhase.FRONT_AXLE_ON_RAMP,
                ContactPhase.MIXED_CONTACT_ENTRY,
            ):
                self.transition(ClimbState.MIXED_CONTACT_ENTRY, now_sec)
        elif self.state == ClimbState.MIXED_CONTACT_ENTRY:
            if phase == ContactPhase.FULL_SLOPE:
                self.transition(ClimbState.CLIMB, now_sec)
        elif self.state == ClimbState.CLIMB:
            if phase in (
                ContactPhase.CREST_DETECTED,
                ContactPhase.FRONT_AXLE_ON_TOP,
            ):
                self.transition(ClimbState.MIXED_CONTACT_EXIT, now_sec)
        elif self.state == ClimbState.MIXED_CONTACT_EXIT:
            if phase == ContactPhase.TOP_PLATFORM:
                self.transition(ClimbState.TOP_TRANSITION, now_sec)
        elif self.state == ClimbState.TOP_TRANSITION and wheels_stopped:
            self.transition(ClimbState.TOP_HOLD, now_sec)
        elif self.state == ClimbState.TOP_HOLD:
            if (
                now_sec - self.state_enter_sec >= self.config.top_hold_sec
                and wheels_stopped and posture_neutral
            ):
                self.transition(ClimbState.COMPLETE, now_sec)
        return self.state
