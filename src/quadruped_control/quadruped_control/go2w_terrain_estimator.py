"""Deterministic terrain and contact-phase estimation for Go2-W."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from statistics import median

from quadruped_control.go2w_dynamic_models import ContactPhase, TerrainEstimate


@dataclass(frozen=True)
class TerrainEstimatorConfig:
    """Validated dual-window slope and phase-estimator settings."""

    fast_window_sec: float = 0.25
    stable_window_sec: float = 0.75
    minimum_progress_m: float = 0.018
    entry_slope_deg: float = 2.5
    flat_slope_deg: float = 1.8
    transition_confidence: float = 0.45
    entry_hold_sec: float = 0.12
    top_hold_sec: float = 0.80
    wheelbase_m: float = 0.3868
    minimum_crest_progress_m: float = 0.80
    minimum_crest_height_m: float = 0.15
    maximum_slope_deg: float = 55.0
    outlier_residual_m: float = 0.025

    def validate(self) -> None:
        """Reject inconsistent or non-physical estimator settings."""
        positive = (
            self.fast_window_sec, self.stable_window_sec,
            self.minimum_progress_m, self.entry_slope_deg,
            self.flat_slope_deg, self.entry_hold_sec, self.top_hold_sec,
            self.wheelbase_m, self.maximum_slope_deg,
            self.minimum_crest_progress_m, self.minimum_crest_height_m,
            self.outlier_residual_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError(
                'terrain estimator values must be finite and positive'
            )
        if self.fast_window_sec >= self.stable_window_sec:
            raise ValueError(
                'fast slope window must be shorter than stable window'
            )
        if not 0.0 <= self.transition_confidence <= 1.0:
            raise ValueError('transition confidence must be in [0, 1]')


class TerrainEstimator:
    """Estimate slope and contact phase from robust pose regression."""

    def __init__(self, config: TerrainEstimatorConfig) -> None:
        """Initialize empty estimator history."""
        config.validate()
        self.config = config
        self._history: deque[tuple[float, float, float]] = deque()
        self._initialized = False
        self._initial_x = 0.0
        self._initial_y = 0.0
        self._initial_z = 0.0
        self._heading_x = 1.0
        self._heading_y = 0.0
        self._last_stamp: float | None = None
        self._last_fast_slope = 0.0
        self._last_phase = ContactPhase.FLAT_APPROACH
        self._candidate_since: float | None = None
        self._entry_progress: float | None = None
        self._crest_progress: float | None = None
        self._maximum_stable_slope = 0.0

    @property
    def initialized(self) -> bool:
        """Report whether the current-position origin has been captured."""
        return self._initialized

    def reset(
        self, x_m: float, y_m: float, z_m: float, yaw_rad: float,
    ) -> None:
        """Capture the current pose as progress and height origin."""
        values = (x_m, y_m, z_m, yaw_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('terrain origin must be finite')
        self._history.clear()
        self._initial_x = x_m
        self._initial_y = y_m
        self._initial_z = z_m
        self._heading_x = math.cos(yaw_rad)
        self._heading_y = math.sin(yaw_rad)
        self._last_stamp = None
        self._last_fast_slope = 0.0
        self._last_phase = ContactPhase.FLAT_APPROACH
        self._candidate_since = None
        self._entry_progress = None
        self._crest_progress = None
        self._maximum_stable_slope = 0.0
        self._initialized = True

    @staticmethod
    def _linear_fit(
        samples: list[tuple[float, float, float]],
    ) -> tuple[float, float, float] | None:
        if len(samples) < 4:
            return None
        progress = [item[1] for item in samples]
        heights = [item[2] for item in samples]
        mean_s = sum(progress) / len(progress)
        mean_z = sum(heights) / len(heights)
        denominator = sum((value - mean_s) ** 2 for value in progress)
        if denominator <= 1.0e-10:
            return None
        gradient = sum(
            (s_value - mean_s) * (z_value - mean_z)
            for s_value, z_value in zip(progress, heights)
        ) / denominator
        intercept = mean_z - gradient * mean_s
        residuals = [
            z_value - (gradient * s_value + intercept)
            for s_value, z_value in zip(progress, heights)
        ]
        rms = math.sqrt(
            sum(value * value for value in residuals) / len(residuals)
        )
        return gradient, intercept, rms

    def _robust_slope(
        self, samples: list[tuple[float, float, float]], window_sec: float,
    ) -> tuple[float, float, bool]:
        if len(samples) < 4:
            return 0.0, 0.0, False
        progress = [item[1] for item in samples]
        span = max(progress) - min(progress)
        time_span = samples[-1][0] - samples[0][0]
        required_progress = self.config.minimum_progress_m * (
            0.70 if window_sec == self.config.fast_window_sec else 1.0
        )
        if span < required_progress or time_span < 0.45 * window_sec:
            return 0.0, 0.0, False
        fit = self._linear_fit(samples)
        if fit is None:
            return 0.0, 0.0, False
        gradient, intercept, _ = fit
        residuals = [
            item[2] - (gradient * item[1] + intercept) for item in samples
        ]
        residual_median = median(residuals)
        deviations = [abs(value - residual_median) for value in residuals]
        mad = median(deviations)
        cutoff = max(self.config.outlier_residual_m, 3.5 * 1.4826 * mad)
        filtered = [
            sample for sample, residual in zip(samples, residuals)
            if abs(residual - residual_median) <= cutoff
        ]
        fit = self._linear_fit(filtered)
        if fit is None:
            return 0.0, 0.0, False
        gradient, _, rms = fit
        angle = math.degrees(math.atan(gradient))
        if (
            not math.isfinite(angle)
            or abs(angle) > self.config.maximum_slope_deg
        ):
            return 0.0, 0.0, False
        progress_score = min(1.0, span / (2.0 * required_progress))
        time_score = min(1.0, time_span / window_sec)
        fit_score = max(0.0, 1.0 - rms / self.config.outlier_residual_m)
        retention_score = len(filtered) / len(samples)
        confidence = progress_score * time_score * fit_score * retention_score
        return angle, max(0.0, min(1.0, confidence)), True

    def _set_phase(self, phase: ContactPhase) -> None:
        if phase != self._last_phase:
            self._last_phase = phase
            self._candidate_since = None

    def _update_phase(
        self,
        stamp: float,
        progress: float,
        fast_slope: float,
        stable_slope: float,
        confidence: float,
        stable_confidence: float,
        slope_rate: float,
        vertical_speed: float,
        pitch_rate: float,
        height_gain: float,
    ) -> None:
        cfg = self.config
        credible = confidence >= cfg.transition_confidence
        stable_credible = stable_confidence >= cfg.transition_confidence
        phase = self._last_phase
        transition_signal = max(fast_slope, stable_slope)

        if phase == ContactPhase.FLAT_APPROACH:
            if credible and transition_signal >= cfg.entry_slope_deg:
                self._candidate_since = self._candidate_since or stamp
                if stamp - self._candidate_since >= cfg.entry_hold_sec:
                    self._entry_progress = progress
                    self._set_phase(ContactPhase.ENTRY_DETECTED)
            else:
                self._candidate_since = None
            return

        entry_distance = (
            0.0
            if self._entry_progress is None
            else progress - self._entry_progress
        )
        if phase == ContactPhase.ENTRY_DETECTED:
            if entry_distance >= 0.08 * cfg.wheelbase_m or (
                vertical_speed > 0.01 and abs(pitch_rate) > 0.02
            ):
                self._set_phase(ContactPhase.FRONT_AXLE_ON_RAMP)
            return
        if phase == ContactPhase.FRONT_AXLE_ON_RAMP:
            if entry_distance >= 0.25 * cfg.wheelbase_m:
                self._set_phase(ContactPhase.MIXED_CONTACT_ENTRY)
            return
        if phase == ContactPhase.MIXED_CONTACT_ENTRY:
            # Entry was already confirmed with a credible regression. Once
            # the robot has advanced most of a wheelbase, full contact is a
            # geometric fact even if deliberately slow motion no longer
            # spans the regression's minimum-progress window.
            full_slope = (
                max(fast_slope, stable_slope) >= cfg.entry_slope_deg
                and entry_distance >= 0.72 * cfg.wheelbase_m
            )
            if full_slope:
                self._set_phase(ContactPhase.FULL_SLOPE)
            return
        if phase == ContactPhase.FULL_SLOPE:
            self._maximum_stable_slope = max(
                self._maximum_stable_slope, stable_slope,
            )
            crest_level = (
                stable_credible
                and self._maximum_stable_slope >= 2.0 * cfg.entry_slope_deg
                and entry_distance >= cfg.minimum_crest_progress_m
                and height_gain >= cfg.minimum_crest_height_m
                and fast_slope <= max(
                    cfg.flat_slope_deg,
                    0.45 * self._maximum_stable_slope,
                )
                and stable_slope <= max(
                    cfg.flat_slope_deg,
                    0.35 * self._maximum_stable_slope,
                )
            )
            if crest_level:
                self._candidate_since = self._candidate_since or stamp
                if (
                    self._candidate_since is not None
                    and stamp - self._candidate_since >= cfg.top_hold_sec
                ):
                    self._crest_progress = progress
                    self._set_phase(ContactPhase.CREST_DETECTED)
            else:
                self._candidate_since = None
            return

        crest_distance = (
            0.0
            if self._crest_progress is None
            else progress - self._crest_progress
        )
        if phase == ContactPhase.CREST_DETECTED:
            if crest_distance >= 0.08 * cfg.wheelbase_m:
                self._set_phase(ContactPhase.FRONT_AXLE_ON_TOP)
            return
        if phase == ContactPhase.FRONT_AXLE_ON_TOP:
            if crest_distance >= 0.25 * cfg.wheelbase_m:
                self._set_phase(ContactPhase.MIXED_CONTACT_EXIT)
            return
        if phase == ContactPhase.MIXED_CONTACT_EXIT:
            regression_flat = (
                credible and abs(stable_slope) <= cfg.flat_slope_deg
            )
            quasi_stationary_top = (
                abs(vertical_speed) <= 0.02
                and abs(pitch_rate) <= 0.15
            )
            on_top = regression_flat or quasi_stationary_top
            if on_top and crest_distance >= 0.72 * cfg.wheelbase_m:
                self._candidate_since = self._candidate_since or stamp
                if stamp - self._candidate_since >= cfg.top_hold_sec:
                    self._set_phase(ContactPhase.TOP_PLATFORM)
            else:
                self._candidate_since = None

    def update(
        self,
        stamp_sec: float,
        x_m: float,
        y_m: float,
        z_m: float,
        vertical_speed_mps: float = 0.0,
        pitch_rate_radps: float = 0.0,
        vertical_accel_mps2: float = 0.0,
    ) -> TerrainEstimate:
        """Ingest one pose sample and return the current terrain estimate."""
        values = (
            stamp_sec, x_m, y_m, z_m, vertical_speed_mps,
            pitch_rate_radps, vertical_accel_mps2,
        )
        if not self._initialized:
            raise RuntimeError('terrain estimator must be reset before update')
        if not all(math.isfinite(value) for value in values):
            return TerrainEstimate(stale=True)
        if self._last_stamp is not None and stamp_sec <= self._last_stamp:
            raise ValueError('terrain timestamps must increase monotonically')
        dx = x_m - self._initial_x
        dy = y_m - self._initial_y
        progress = dx * self._heading_x + dy * self._heading_y
        height_gain = z_m - self._initial_z
        self._history.append((stamp_sec, progress, z_m))
        oldest = stamp_sec - 1.15 * self.config.stable_window_sec
        while self._history and self._history[0][0] < oldest:
            self._history.popleft()
        samples = list(self._history)
        fast_samples = [
            item for item in samples
            if item[0] >= stamp_sec - self.config.fast_window_sec
        ]
        stable_samples = [
            item for item in samples
            if item[0] >= stamp_sec - self.config.stable_window_sec
        ]
        raw_slope, raw_confidence, raw_valid = self._robust_slope(
            fast_samples, self.config.fast_window_sec,
        )
        stable_raw, stable_confidence, stable_valid = self._robust_slope(
            stable_samples, self.config.stable_window_sec,
        )
        fast_alpha = 0.55
        fast_slope = (
            fast_alpha * raw_slope + (1.0 - fast_alpha) * self._last_fast_slope
            if raw_valid else self._last_fast_slope
        )
        stable_slope = stable_raw if stable_valid else fast_slope
        dt = 0.0 if self._last_stamp is None else stamp_sec - self._last_stamp
        slope_rate = (
            (fast_slope - self._last_fast_slope) / dt if dt > 1.0e-6 else 0.0
        )
        confidence = max(raw_confidence, stable_confidence)
        dynamic_signal = min(
            1.0,
            abs(slope_rate) / 15.0
            + abs(vertical_speed_mps) / 0.20
            + abs(pitch_rate_radps) / 1.0
            + abs(vertical_accel_mps2) / 20.0,
        )
        self._update_phase(
            stamp_sec, progress, fast_slope, stable_slope, confidence,
            stable_confidence if stable_valid else 0.0, slope_rate,
            vertical_speed_mps, pitch_rate_radps, height_gain,
        )
        self._last_stamp = stamp_sec
        self._last_fast_slope = fast_slope
        valid = raw_valid or stable_valid
        return TerrainEstimate(
            forward_progress_m=progress,
            height_gain_m=height_gain,
            raw_slope_deg=raw_slope if raw_valid else fast_slope,
            fast_slope_deg=fast_slope,
            stable_slope_deg=stable_slope,
            slope_confidence=confidence,
            slope_rate_degps=slope_rate,
            transition_probability=max(dynamic_signal, confidence),
            contact_phase=self._last_phase,
            valid=valid,
            stale=False,
        )
