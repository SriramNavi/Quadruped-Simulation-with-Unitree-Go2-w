"""Analytical Go2-W leg kinematics derived from the robot URDF."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


class IKError(ValueError):
    """Raised when a Cartesian target has no safe analytical solution."""


@dataclass(frozen=True)
class LegGeometry:
    """Geometry and limits for one Go2-W leg, in the hip joint frame."""

    name: str
    side_sign: float
    hip_x: float
    lateral_offset: float = 0.0955
    thigh_length: float = 0.213
    calf_length: float = 0.2264
    hip_limits: tuple[float, float] = (-1.0472, 1.0472)
    thigh_limits: tuple[float, float] = (-1.5707963267948966, 3.4907)
    calf_limits: tuple[float, float] = (-2.7227, -0.83776)

    @classmethod
    def for_leg(cls, name: str) -> 'LegGeometry':
        if name not in ('FL', 'FR', 'RL', 'RR'):
            raise ValueError(f'unknown Go2-W leg: {name}')
        front = name.startswith('F')
        left = name.endswith('L')
        thigh_limits = (
            (-1.5707963267948966, 3.4907)
            if front else (-0.5236, 4.5379)
        )
        return cls(
            name=name,
            side_sign=1.0 if left else -1.0,
            hip_x=0.1934 if front else -0.1934,
            thigh_limits=thigh_limits,
        )


@dataclass(frozen=True)
class JointSolution:
    hip: float
    thigh: float
    calf: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.hip, self.thigh, self.calf)


class AnalyticalLegFK:
    def __init__(self, geometry: LegGeometry) -> None:
        self.geometry = geometry

    def solve(self, joints: JointSolution | Sequence[float]) -> tuple[float, float, float]:
        q0, q1, q2 = (
            joints.as_tuple() if isinstance(joints, JointSolution) else joints
        )
        if not all(math.isfinite(value) for value in (q0, q1, q2)):
            raise ValueError('joint positions must be finite')
        g = self.geometry
        x = -g.thigh_length * math.sin(q1) - g.calf_length * math.sin(q1 + q2)
        z_plane = -g.thigh_length * math.cos(q1) - g.calf_length * math.cos(q1 + q2)
        y_plane = g.side_sign * g.lateral_offset
        y = math.cos(q0) * y_plane - math.sin(q0) * z_plane
        z = math.sin(q0) * y_plane + math.cos(q0) * z_plane
        result = (x, y, z)
        if not all(math.isfinite(value) for value in result):
            raise ValueError('forward kinematics produced a non-finite position')
        return result


class AnalyticalLegIK:
    _TRIG_TOLERANCE = 1.0e-10

    def __init__(self, geometry: LegGeometry, workspace_margin: float = 1.0e-5) -> None:
        if not math.isfinite(workspace_margin) or workspace_margin < 0.0:
            raise ValueError('workspace_margin must be finite and non-negative')
        self.geometry = geometry
        self.workspace_margin = workspace_margin

    @staticmethod
    def _within(value: float, limits: tuple[float, float], tolerance: float = 1.0e-9) -> bool:
        return limits[0] - tolerance <= value <= limits[1] + tolerance

    @staticmethod
    def _distance(candidate: JointSolution, previous: JointSolution | None) -> float:
        if previous is None:
            return abs(candidate.hip) + abs(candidate.thigh - 0.5) + abs(candidate.calf + 1.4)
        return sum(
            (a - b) ** 2
            for a, b in zip(candidate.as_tuple(), previous.as_tuple())
        )

    def solve(
        self,
        target: Sequence[float],
        previous: JointSolution | None = None,
    ) -> JointSolution:
        if len(target) != 3:
            raise IKError('target must contain x, y, and z')
        x, y, z = (float(value) for value in target)
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise IKError('target must be finite')

        g = self.geometry
        yz_plane_sq = y * y + z * z - g.lateral_offset**2
        if yz_plane_sq < -self._TRIG_TOLERANCE:
            raise IKError('target lies inside the hip-offset singular cylinder')
        yz_plane_sq = max(0.0, yz_plane_sq)
        candidates: list[JointSolution] = []

        for z_plane in (-math.sqrt(yz_plane_sq), math.sqrt(yz_plane_sq)):
            reach_sq = x * x + z_plane * z_plane
            reach = math.sqrt(reach_sq)
            lower = abs(g.thigh_length - g.calf_length) + self.workspace_margin
            upper = g.thigh_length + g.calf_length - self.workspace_margin
            if reach < lower or reach > upper:
                continue
            cosine = (
                reach_sq - g.thigh_length**2 - g.calf_length**2
            ) / (2.0 * g.thigh_length * g.calf_length)
            if cosine < -1.0 - self._TRIG_TOLERANCE or cosine > 1.0 + self._TRIG_TOLERANCE:
                continue
            cosine = min(1.0, max(-1.0, cosine))
            for q2 in (-math.acos(cosine), math.acos(cosine)):
                q0 = math.atan2(
                    g.side_sign * g.lateral_offset * z - z_plane * y,
                    g.side_sign * g.lateral_offset * y + z_plane * z,
                )
                target_angle = math.atan2(-x, -z_plane)
                correction = math.atan2(
                    g.calf_length * math.sin(q2),
                    g.thigh_length + g.calf_length * math.cos(q2),
                )
                solution = JointSolution(q0, target_angle - correction, q2)
                finite = all(
                    math.isfinite(value) for value in solution.as_tuple()
                )
                within_limits = all((
                    self._within(solution.hip, g.hip_limits),
                    self._within(solution.thigh, g.thigh_limits),
                    self._within(solution.calf, g.calf_limits),
                ))
                if finite and within_limits:
                    candidates.append(solution)

        if not candidates:
            raise IKError('target is unreachable within Go2-W joint limits')
        return min(candidates, key=lambda item: self._distance(item, previous))
