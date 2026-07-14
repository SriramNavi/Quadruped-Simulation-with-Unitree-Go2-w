"""Online analytical-IK feasibility envelope for adaptive Go2-W posture."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Mapping

from quadruped_control.go2w_analytical_ik import (
    AnalyticalLegFK,
    AnalyticalLegIK,
    IKError,
    JointSolution,
    LegGeometry,
)
from quadruped_control.go2w_dynamic_models import (
    ContactPhase,
    FeasibilityResult,
    JointCommandResult,
    LEGS,
)


@dataclass(frozen=True)
class FeasibilityConfig:
    """Validated online IK-envelope settings and absolute backstops."""

    update_hz: float = 15.0
    joint_limit_margin_rad: float = 0.035
    workspace_margin_m: float = 0.0005
    absolute_pitch_cap_deg: float = 25.0
    absolute_roll_cap_deg: float = 5.0
    absolute_leg_offset_cap_m: float = 0.060
    envelope_safety_deg: float = 0.30
    material_slope_change_deg: float = 2.0
    binary_search_iterations: int = 14

    def validate(self) -> None:
        """Reject invalid update, margin, cap, or search settings."""
        values = (
            self.update_hz, self.joint_limit_margin_rad,
            self.workspace_margin_m, self.absolute_pitch_cap_deg,
            self.absolute_roll_cap_deg, self.absolute_leg_offset_cap_m,
            self.envelope_safety_deg,
            self.material_slope_change_deg,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('feasibility configuration must be non-negative')
        if self.update_hz <= 0.0 or self.binary_search_iterations < 8:
            raise ValueError('invalid feasibility update rate or search depth')


class OnlineFeasibilitySolver:
    """Find an all-four-leg correction envelope without partial commands."""

    def __init__(self, config: FeasibilityConfig) -> None:
        """Create one analytical FK/IK adapter per leg."""
        config.validate()
        self.config = config
        self.geometries = {leg: LegGeometry.for_leg(leg) for leg in LEGS}
        self.fk = {
            leg: AnalyticalLegFK(self.geometries[leg]) for leg in LEGS
        }
        self.ik = {
            leg: AnalyticalLegIK(
                self.geometries[leg], config.workspace_margin_m,
            )
            for leg in LEGS
        }
        self.nominal_feet: dict[str, tuple[float, float, float]] = {}
        self.previous_solutions: dict[str, JointSolution] = {}
        self._last_envelope = FeasibilityResult()
        self._last_update_sec = -math.inf
        self._last_phase: ContactPhase | None = None
        self._last_slope_deg = math.nan

    def capture_stance(
        self, joint_positions: Mapping[str, float],
    ) -> tuple[
        dict[str, tuple[float, float, float]],
        dict[str, JointSolution],
    ]:
        """Capture the current FK stance with branch continuity."""
        feet: dict[str, tuple[float, float, float]] = {}
        solutions: dict[str, JointSolution] = {}
        for leg in LEGS:
            names = tuple(
                f'{leg}_{joint}_joint'
                for joint in ('hip', 'thigh', 'calf')
            )
            try:
                solution = JointSolution(*(
                    float(joint_positions[name]) for name in names
                ))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'missing or invalid {leg} joints') from error
            if not all(math.isfinite(value) for value in solution.as_tuple()):
                raise ValueError(f'non-finite {leg} joints')
            margin, joint = self._joint_margin(leg, solution)
            if margin < self.config.joint_limit_margin_rad:
                raise ValueError(
                    f'{leg} {joint} lacks configured joint margin'
                )
            foot = self.fk[leg].solve(solution)
            recovered = self.ik[leg].solve(foot, solution)
            if max(
                abs(a - b)
                for a, b in zip(recovered.as_tuple(), solution.as_tuple())
            ) > 1.0e-5:
                raise ValueError(f'{leg} FK/IK branch round trip failed')
            feet[leg] = foot
            solutions[leg] = solution
        self.set_stance(feet, solutions)
        return feet, solutions

    def observed_stance(
        self,
        joint_positions: Mapping[str, float],
        tolerance_rad: float = 0.0,
    ) -> tuple[dict[str, tuple[float, float, float]], float]:
        """Return current FK contacts and the smallest hard-limit margin."""
        feet: dict[str, tuple[float, float, float]] = {}
        minimum_margin = math.inf
        for leg in LEGS:
            names = tuple(
                f'{leg}_{joint}_joint'
                for joint in ('hip', 'thigh', 'calf')
            )
            try:
                solution = JointSolution(*(
                    float(joint_positions[name]) for name in names
                ))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'missing or invalid {leg} joints') from error
            margin, joint = self._joint_margin(leg, solution)
            if margin < -tolerance_rad:
                raise ValueError(f'{leg} {joint} exceeds hard joint limit')
            minimum_margin = min(minimum_margin, margin)
            feet[leg] = self.fk[leg].solve(solution)
        return feet, minimum_margin

    def set_stance(
        self,
        nominal_feet: Mapping[str, tuple[float, float, float]],
        previous_solutions: Mapping[str, JointSolution],
    ) -> None:
        """Install a complete validated nominal stance."""
        if (
            set(nominal_feet) != set(LEGS)
            or set(previous_solutions) != set(LEGS)
        ):
            raise ValueError('complete four-leg stance is required')
        values = [value for point in nominal_feet.values() for value in point]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('nominal foot positions must be finite')
        self.nominal_feet = dict(nominal_feet)
        self.previous_solutions = dict(previous_solutions)
        self._last_update_sec = -math.inf

    def _joint_margin(
        self, leg: str, solution: JointSolution,
    ) -> tuple[float, str]:
        geometry = self.geometries[leg]
        values = {
            'hip': (solution.hip, geometry.hip_limits),
            'thigh': (solution.thigh, geometry.thigh_limits),
            'calf': (solution.calf, geometry.calf_limits),
        }
        margins = {
            name: min(value - limits[0], limits[1] - value)
            for name, (value, limits) in values.items()
        }
        joint = min(margins, key=margins.get)
        return margins[joint], joint

    def _attempt(
        self,
        pitch_correction_deg: float,
        roll_correction_deg: float,
        previous: Mapping[str, JointSolution] | None = None,
    ) -> JointCommandResult:
        if not self.nominal_feet:
            return JointCommandResult(False, limiting_reason='stance_not_set')
        previous = previous or self.previous_solutions
        pitch_rad = math.radians(pitch_correction_deg)
        roll_rad = math.radians(roll_correction_deg)
        positions: list[float] = []
        solutions: dict[str, JointSolution] = {}
        minimum_margin = math.inf
        limiting_leg = ''
        limiting_joint = ''
        front_offsets: list[float] = []
        rear_offsets: list[float] = []
        left_offsets: list[float] = []
        right_offsets: list[float] = []
        for leg in LEGS:
            geometry = self.geometries[leg]
            pitch_offset = geometry.hip_x * math.tan(pitch_rad)
            hip_y = geometry.side_sign * (
                0.0465 + geometry.lateral_offset
            )
            roll_offset = hip_y * math.tan(roll_rad)
            target = list(self.nominal_feet[leg])
            total_offset = pitch_offset + roll_offset
            if abs(total_offset) > self.config.absolute_leg_offset_cap_m:
                return JointCommandResult(
                    False,
                    limiting_leg=leg,
                    limiting_reason='absolute_leg_offset_cap',
                )
            target[2] += total_offset
            try:
                solution = self.ik[leg].solve(target, previous.get(leg))
            except (IKError, ValueError) as error:
                return JointCommandResult(
                    False,
                    limiting_leg=leg,
                    limiting_reason=f'workspace_or_joint_limit:{error}',
                )
            margin, joint = self._joint_margin(leg, solution)
            if margin < self.config.joint_limit_margin_rad:
                return JointCommandResult(
                    False,
                    limiting_leg=leg,
                    limiting_joint=joint,
                    limiting_reason='joint_limit_margin',
                    joint_margin_rad=margin,
                )
            if margin < minimum_margin:
                minimum_margin = margin
                limiting_leg = leg
                limiting_joint = joint
            solutions[leg] = solution
            positions.extend(solution.as_tuple())
            (front_offsets if leg.startswith('F') else rear_offsets).append(
                pitch_offset
            )
            (left_offsets if leg.endswith('L') else right_offsets).append(
                roll_offset
            )
        if (
            len(positions) != 12
            or not all(math.isfinite(v) for v in positions)
        ):
            return JointCommandResult(
                False, limiting_reason='non_finite_solution',
            )
        return JointCommandResult(
            True,
            positions=tuple(positions),
            solutions=solutions,
            applied_pitch_correction_deg=pitch_correction_deg,
            applied_roll_correction_deg=roll_correction_deg,
            front_offset_m=sum(front_offsets) / len(front_offsets),
            rear_offset_m=sum(rear_offsets) / len(rear_offsets),
            left_offset_m=sum(left_offsets) / len(left_offsets),
            right_offset_m=sum(right_offsets) / len(right_offsets),
            scale=1.0,
            joint_margin_rad=minimum_margin,
            limiting_leg=limiting_leg,
            limiting_joint=limiting_joint,
        )

    def _search_limit(
        self, direction: float, roll: bool = False,
    ) -> tuple[float, JointCommandResult, JointCommandResult]:
        cap = (
            self.config.absolute_roll_cap_deg
            if roll else self.config.absolute_pitch_cap_deg
        )
        nominal = self._attempt(0.0, 0.0)
        if not nominal.valid:
            return 0.0, nominal, nominal
        low = 0.0
        high = cap
        best = nominal
        failure = nominal
        for _ in range(self.config.binary_search_iterations):
            midpoint = 0.5 * (low + high)
            result = self._attempt(
                0.0 if roll else direction * midpoint,
                direction * midpoint if roll else 0.0,
            )
            if result.valid:
                low = midpoint
                best = result
            else:
                high = midpoint
                failure = result
        safe_limit = max(0.0, low - self.config.envelope_safety_deg)
        return direction * safe_limit, best, failure

    def envelope(
        self,
        now_sec: float,
        contact_phase: ContactPhase,
        slope_deg: float,
        force: bool = False,
    ) -> FeasibilityResult:
        """Return a cached or freshly searched all-leg envelope."""
        if not all(math.isfinite(value) for value in (now_sec, slope_deg)):
            raise ValueError('feasibility update inputs must be finite')
        material_change = (
            self._last_phase != contact_phase
            or not math.isfinite(self._last_slope_deg)
            or abs(slope_deg - self._last_slope_deg)
            >= self.config.material_slope_change_deg
        )
        due = now_sec - self._last_update_sec >= 1.0 / self.config.update_hz
        if not (force or material_change or due):
            return replace(self._last_envelope, recomputed=False)
        negative, negative_best, negative_failure = self._search_limit(-1.0)
        positive, positive_best, positive_failure = self._search_limit(1.0)
        roll_negative, _, _ = self._search_limit(-1.0, roll=True)
        roll_positive, _, _ = self._search_limit(1.0, roll=True)
        nominal = self._attempt(0.0, 0.0)
        limiting_candidates = [
            item for item in (negative_failure, positive_failure)
            if not item.valid and item.limiting_reason
        ]
        limiter = limiting_candidates[0] if limiting_candidates else nominal
        max_front = max(
            abs(negative_best.front_offset_m),
            abs(positive_best.front_offset_m),
        )
        max_rear = max(
            abs(negative_best.rear_offset_m), abs(positive_best.rear_offset_m),
        )
        result = FeasibilityResult(
            valid=nominal.valid and positive >= 0.0 and negative <= 0.0,
            min_correction_deg=negative,
            max_correction_deg=positive,
            max_front_offset_m=max_front,
            max_rear_offset_m=max_rear,
            max_roll_correction_deg=min(
                abs(roll_negative), abs(roll_positive),
            ),
            joint_margin_rad=nominal.joint_margin_rad,
            workspace_margin_m=self.config.workspace_margin_m,
            workspace_usage=0.0,
            limiting_leg=limiter.limiting_leg,
            limiting_joint=limiter.limiting_joint,
            limiting_reason=limiter.limiting_reason or 'absolute_safety_cap',
            recomputed=True,
        )
        self._last_envelope = result
        self._last_update_sec = now_sec
        self._last_phase = contact_phase
        self._last_slope_deg = slope_deg
        return result

    def solve_command(
        self,
        pitch_correction_deg: float,
        roll_correction_deg: float,
    ) -> JointCommandResult:
        """Solve an atomic pitch/roll command, scaling only if required."""
        requested = self._attempt(pitch_correction_deg, roll_correction_deg)
        if requested.valid:
            self.previous_solutions = dict(requested.solutions)
            return requested
        nominal = self._attempt(0.0, 0.0)
        if not nominal.valid:
            return requested
        low = 0.0
        high = 1.0
        best = nominal
        for _ in range(self.config.binary_search_iterations):
            scale = 0.5 * (low + high)
            result = self._attempt(
                scale * pitch_correction_deg,
                scale * roll_correction_deg,
            )
            if result.valid:
                low = scale
                best = result
            else:
                high = scale
        if low <= 1.0e-4:
            return JointCommandResult(
                False,
                limiting_leg=requested.limiting_leg,
                limiting_joint=requested.limiting_joint,
                limiting_reason=requested.limiting_reason,
            )
        scaled = JointCommandResult(
            **{
                **best.__dict__,
                'scale': low,
                'limiting_leg': requested.limiting_leg,
                'limiting_joint': requested.limiting_joint,
                'limiting_reason': requested.limiting_reason,
            }
        )
        self.previous_solutions = dict(scaled.solutions)
        return scaled
