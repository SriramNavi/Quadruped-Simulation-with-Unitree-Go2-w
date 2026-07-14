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
    full_slope_speed_scale: float = 0.80
    full_slope_recovery_hold_sec: float = 0.50
    full_slope_pitch_error_limit_deg: float = 3.0
    full_slope_workspace_limit: float = 0.90
    slope_speed_gain: float = 0.018
    pitch_error_speed_gain: float = 0.10
    workspace_speed_gain: float = 0.35
    stability_caution_margin_m: float = 0.045
    stability_stop_margin_m: float = 0.015
    slip_caution_threshold: float = 0.30
    slip_stop_threshold: float = 0.55
    safe_pause_hold_sec: float = 0.35
    mixed_contact_escape_enabled: bool = True
    mixed_contact_escape_speed_mps: float = 0.012
    mixed_contact_escape_margin_min_m: float = 0.005
    mixed_contact_escape_margin_max_m: float = 0.015
    mixed_contact_escape_workspace_limit: float = 0.90
    mixed_contact_escape_pitch_error_limit_deg: float = 18.0
    mixed_contact_escape_max_duration_sec: float = 2.0
    mixed_contact_escape_rearm_sec: float = 0.50
    mixed_contact_escape_progress_timeout_sec: float = 0.75
    mixed_contact_escape_min_progress_m: float = 0.003

    def validate(self) -> None:
        """Reject invalid scheduler gains, scales, or thresholds."""
        if type(self.mixed_contact_escape_enabled) is not bool:
            raise ValueError('mixed-contact escape enabled must be bool')
        values = tuple(
            value for name, value in self.__dict__.items()
            if name != 'mixed_contact_escape_enabled'
        )
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            and value >= 0.0
            for value in values
        ):
            raise ValueError(
                'motion configuration must be finite and non-negative'
            )
        for value in (
            self.entry_speed_scale, self.crest_speed_scale,
            self.minimum_speed_scale, self.full_slope_speed_scale,
        ):
            if value > 1.0:
                raise ValueError('motion scales must be in [0, 1]')
        if self.full_slope_pitch_error_limit_deg <= 0.0:
            raise ValueError(
                'full-slope pitch error limit must be greater than zero'
            )
        if not 0.0 < self.full_slope_workspace_limit <= 1.0:
            raise ValueError('full-slope workspace limit must be in (0, 1]')
        if self.stability_caution_margin_m <= self.stability_stop_margin_m:
            raise ValueError('invalid stability scaling margins')
        if self.slip_stop_threshold <= self.slip_caution_threshold:
            raise ValueError('invalid traction scaling thresholds')
        if self.mixed_contact_escape_speed_mps <= 0.0:
            raise ValueError(
                'mixed-contact escape speed must be greater than zero'
            )
        if (
            self.requested_speed_mps <= 0.0
            or self.mixed_contact_escape_speed_mps
            > self.requested_speed_mps
        ):
            raise ValueError(
                'mixed-contact escape speed must not exceed requested speed'
            )
        if self.mixed_contact_escape_margin_min_m < 0.0:
            raise ValueError(
                'mixed-contact escape minimum margin must be non-negative'
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
                'mixed-contact escape maximum margin must not exceed '
                'stability stop margin'
            )
        if not 0.0 < self.mixed_contact_escape_workspace_limit <= 1.0:
            raise ValueError(
                'mixed-contact escape workspace limit must be in (0, 1]'
            )
        if self.mixed_contact_escape_pitch_error_limit_deg <= 0.0:
            raise ValueError(
                'mixed-contact escape pitch error limit must be positive'
            )
        if self.mixed_contact_escape_max_duration_sec <= 0.0:
            raise ValueError(
                'mixed-contact escape maximum duration must be positive'
            )
        if self.mixed_contact_escape_rearm_sec < 0.0:
            raise ValueError(
                'mixed-contact escape rearm duration must be non-negative'
            )
        if self.mixed_contact_escape_progress_timeout_sec <= 0.0:
            raise ValueError(
                'mixed-contact escape progress timeout must be positive'
            )
        if self.mixed_contact_escape_min_progress_m < 0.0:
            raise ValueError(
                'mixed-contact escape minimum progress must be non-negative'
            )


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
    _MIXED_CONTACT_ESCAPE_PHASES = {
        ContactPhase.FRONT_AXLE_ON_RAMP,
        ContactPhase.MIXED_CONTACT_ENTRY,
    }

    def __init__(self, config: MotionSupervisorConfig) -> None:
        """Initialize speed, pause, and wheel-command state."""
        config.validate()
        self.config = config
        self._speed_mps = 0.0
        self._wheel_linear = [0.0] * 4
        self._safe_since: float | None = None
        self._paused = False
        self._full_slope_safe_since: float | None = None
        self._full_slope_recovered = False
        self._escape_active = False
        self._escape_start_sec: float | None = None
        self._escape_start_progress_m: float | None = None
        self._escape_last_progress_sec: float | None = None
        self._escape_rearm_until_sec: float | None = None
        self._escape_reason = ''
        self._last_update_sec: float | None = None

    def reset(self) -> None:
        """Clear speed and pause history at a run boundary."""
        self._speed_mps = 0.0
        self._wheel_linear = [0.0] * 4
        self._safe_since = None
        self._paused = False
        self._full_slope_safe_since = None
        self._full_slope_recovered = False
        self._escape_active = False
        self._escape_start_sec = None
        self._escape_start_progress_m = None
        self._escape_last_progress_sec = None
        self._escape_rearm_until_sec = None
        self._escape_reason = ''
        self._last_update_sec = None

    @property
    def full_slope_recovered(self) -> bool:
        """Report whether sustained healthy full-slope recovery is active."""
        return self._full_slope_recovered

    @property
    def mixed_contact_escape_active(self) -> bool:
        """Report whether the bounded mixed-contact crawl is active."""
        return self._escape_active

    @property
    def mixed_contact_escape_elapsed_sec(self) -> float:
        """Report elapsed time for the current escape attempt."""
        if (
            not self._escape_active
            or self._escape_start_sec is None
            or self._last_update_sec is None
        ):
            return 0.0
        return max(0.0, self._last_update_sec - self._escape_start_sec)

    @property
    def mixed_contact_escape_reason(self) -> str:
        """Report the latest escape transition reason."""
        return self._escape_reason

    def _clear_escape_timing(self) -> None:
        """Clear timing and progress checkpoints for an inactive escape."""
        self._escape_start_sec = None
        self._escape_start_progress_m = None
        self._escape_last_progress_sec = None

    def _abort_escape(self, now_sec: float, reason: str) -> None:
        """Latch a bounded escape failure into the ordinary safe pause."""
        self._escape_active = False
        self._clear_escape_timing()
        self._escape_rearm_until_sec = (
            now_sec + self.config.mixed_contact_escape_rearm_sec
        )
        self._escape_reason = reason
        self._paused = True
        self._safe_since = None

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
        previous_update_sec = self._last_update_sec
        time_valid = (
            math.isfinite(now_sec)
            and math.isfinite(dt_sec)
            and dt_sec >= 0.0
            and (
                previous_update_sec is None
                or now_sec >= previous_update_sec
            )
        )
        if time_valid:
            self._last_update_sec = now_sec
        forward_progress_m = terrain.forward_progress_m
        escape_inputs_valid = time_valid and all(
            math.isfinite(value) for value in (
                terrain.stable_slope_deg,
                terrain.fast_slope_deg,
                forward_progress_m,
                pitch_error_deg,
                stability.minimum_margin_m,
                stability.longitudinal_margin_m,
                stability.lateral_margin_m,
                traction.average_abs_slip,
                feasibility.workspace_usage,
                heading_error_rad,
                yaw_rate_radps,
            )
        )
        rearmed_now = False
        if (
            self._escape_rearm_until_sec is not None
            and time_valid
            and now_sec >= self._escape_rearm_until_sec
        ):
            self._escape_rearm_until_sec = None
            self._escape_reason = 'ESCAPE_REARMED'
            rearmed_now = True

        aborted_this_update = False
        if self._escape_active:
            abort_reason = ''
            if not escape_inputs_valid:
                abort_reason = 'ESCAPE_ABORT_INVALID_INPUT'
            elif not drive_enabled:
                abort_reason = 'ESCAPE_ABORT_DRIVE_DISABLED'
            elif stability.state == SafetyState.INFEASIBLE:
                abort_reason = 'ESCAPE_ABORT_STABILITY'
            elif (
                stability.state in (SafetyState.STOP, SafetyState.CAUTION)
                and (
                    stability.longitudinal_margin_m
                    > stability.lateral_margin_m + 1.0e-9
                    or not math.isclose(
                        stability.minimum_margin_m,
                        stability.longitudinal_margin_m,
                        rel_tol=0.0,
                        abs_tol=1.0e-9,
                    )
                )
            ):
                abort_reason = 'ESCAPE_ABORT_STABILITY'
            elif (
                stability.minimum_margin_m
                < cfg.mixed_contact_escape_margin_min_m
                or stability.minimum_margin_m <= 0.0
            ):
                abort_reason = 'ESCAPE_ABORT_MARGIN'
            elif traction.state in (
                SafetyState.STOP,
                SafetyState.INFEASIBLE,
            ) or (
                traction.average_abs_slip >= 0.15
            ) or (
                traction.peak_abs_slip >= cfg.slip_caution_threshold
            ):
                abort_reason = 'ESCAPE_ABORT_TRACTION'
            elif (
                feasibility.workspace_usage
                > cfg.mixed_contact_escape_workspace_limit
            ):
                abort_reason = 'ESCAPE_ABORT_WORKSPACE'
            elif (
                abs(pitch_error_deg)
                > cfg.mixed_contact_escape_pitch_error_limit_deg
            ):
                abort_reason = 'ESCAPE_ABORT_PITCH'
            elif terrain.contact_phase == ContactPhase.FULL_SLOPE:
                self._escape_active = False
                self._clear_escape_timing()
                self._escape_rearm_until_sec = None
                self._escape_reason = 'ESCAPE_SUCCESS_FULL_SLOPE'
            elif (
                terrain.contact_phase
                not in self._MIXED_CONTACT_ESCAPE_PHASES
            ):
                abort_reason = 'ESCAPE_ABORT_PHASE'
            elif (
                self._escape_start_sec is None
                or now_sec - self._escape_start_sec
                >= cfg.mixed_contact_escape_max_duration_sec
            ):
                abort_reason = 'ESCAPE_ABORT_TIMEOUT'
            else:
                if (
                    self._escape_start_progress_m is None
                    or self._escape_last_progress_sec is None
                ):
                    abort_reason = 'ESCAPE_ABORT_INVALID_INPUT'
                elif (
                    forward_progress_m - self._escape_start_progress_m
                    >= cfg.mixed_contact_escape_min_progress_m
                ):
                    self._escape_start_progress_m = forward_progress_m
                    self._escape_last_progress_sec = now_sec
                elif (
                    now_sec - self._escape_last_progress_sec
                    > cfg.mixed_contact_escape_progress_timeout_sec
                ):
                    abort_reason = 'ESCAPE_ABORT_NO_PROGRESS'
            if abort_reason:
                self._abort_escape(now_sec, abort_reason)
                aborted_this_update = True

        longitudinal_limited = (
            escape_inputs_valid
            and stability.longitudinal_margin_m
            <= stability.lateral_margin_m + 1.0e-9
            and math.isclose(
                stability.minimum_margin_m,
                stability.longitudinal_margin_m,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        )
        escape_recoverable = (
            cfg.mixed_contact_escape_enabled
            and not self._escape_active
            and not aborted_this_update
            and not rearmed_now
            and self._escape_rearm_until_sec is None
            and terrain.contact_phase in self._MIXED_CONTACT_ESCAPE_PHASES
            and drive_enabled
            and escape_inputs_valid
            and stability.state in (SafetyState.STOP, SafetyState.CAUTION)
            and longitudinal_limited
            and stability.minimum_margin_m
            >= cfg.mixed_contact_escape_margin_min_m
            and stability.minimum_margin_m
            < cfg.mixed_contact_escape_margin_max_m
            and traction.state in (
                SafetyState.SAFE,
                SafetyState.CAUTION,
            )
            and traction.average_abs_slip < 0.15
            and traction.peak_abs_slip < cfg.slip_caution_threshold
            and feasibility.workspace_usage
            <= cfg.mixed_contact_escape_workspace_limit
            and abs(pitch_error_deg)
            <= cfg.mixed_contact_escape_pitch_error_limit_deg
        )
        if escape_recoverable:
            self._escape_active = True
            self._escape_start_sec = now_sec
            self._escape_start_progress_m = forward_progress_m
            self._escape_last_progress_sec = now_sec
            self._escape_reason = 'RECOVERABLE_MIXED_CONTACT_MARGIN'
            self._paused = False
            self._safe_since = None

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
        safe_now = (
            escape_inputs_valid
            and stability_scale > 0.0
            and traction_scale > 0.0
        )
        if self._escape_active:
            self._paused = False
            self._safe_since = None
            stop_reason = ''
        elif aborted_this_update or not safe_now:
            self._paused = True
            self._safe_since = None
        elif self._paused:
            if self._safe_since is None:
                self._safe_since = now_sec
            if now_sec - self._safe_since >= cfg.safe_pause_hold_sec:
                self._paused = False
                self._safe_since = None
        full_slope_healthy = (
            terrain.contact_phase == ContactPhase.FULL_SLOPE
            and drive_enabled
            and stability.state == SafetyState.SAFE
            and traction.state == SafetyState.SAFE
            and math.isfinite(pitch_error_deg)
            and abs(pitch_error_deg)
            <= cfg.full_slope_pitch_error_limit_deg
            and math.isfinite(feasibility.workspace_usage)
            and feasibility.workspace_usage
            <= cfg.full_slope_workspace_limit
            and not self._paused
            and not self._escape_active
        )
        if full_slope_healthy:
            if self._full_slope_safe_since is None:
                self._full_slope_safe_since = now_sec
            if (
                now_sec - self._full_slope_safe_since
                >= cfg.full_slope_recovery_hold_sec
            ):
                self._full_slope_recovered = True
        else:
            self._full_slope_safe_since = None
            self._full_slope_recovered = False
        product = (
            slope_scale * transition_scale * pitch_scale
            * stability_scale * traction_scale * workspace_scale
        )
        if product > 0.0:
            product = max(cfg.minimum_speed_scale, product)
        if (
            self._full_slope_recovered
            and full_slope_healthy
            and safe_now
            and not self._paused
        ):
            product = max(product, cfg.full_slope_speed_scale)
        product = clamp(product, 0.0, 1.0)
        normal_target = cfg.requested_speed_mps * product
        if not drive_enabled or not escape_inputs_valid:
            target = 0.0
        elif self._escape_active:
            target = min(
                cfg.mixed_contact_escape_speed_mps,
                cfg.requested_speed_mps,
            )
        elif self._paused:
            target = 0.0
        else:
            target = normal_target
        if aborted_this_update:
            stop_reason = self._escape_reason
        elif not escape_inputs_valid:
            stop_reason = 'INVALID_MOTION_SUPERVISOR_INPUT'
        if self._paused and not stop_reason:
            stop_reason = 'SAFE_PAUSE_WAITING_FOR_RECOVERY'
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
