"""Generic 3-DOF leg kinematics.

The model assumes an abduction/adduction joint about the hip x-axis and two
pitch joints about the hip/knee y-axis. It is intentionally small and
robot-agnostic so robot-specific dimensions can come from YAML later.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin, sqrt


@dataclass(frozen=True)
class LegGeometry:
    abduction_offset: float = 0.05
    thigh_length: float = 0.20
    calf_length: float = 0.20
    side_sign: float = 1.0


class ThreeDofLeg:
    """Forward/inverse kinematics and numerical Jacobian for one leg."""

    def __init__(self, geometry: LegGeometry | None = None) -> None:
        self.geometry = geometry or LegGeometry()

    def forward_kinematics(self, joint_positions: list[float] | tuple[float, float, float]) -> tuple[float, float, float]:
        q_abd, q_hip, q_knee = joint_positions
        geom = self.geometry

        x_plane = (
            -geom.thigh_length * sin(q_hip)
            - geom.calf_length * sin(q_hip + q_knee)
        )
        z_plane = (
            -geom.thigh_length * cos(q_hip)
            - geom.calf_length * cos(q_hip + q_knee)
        )
        y_plane = geom.side_sign * geom.abduction_offset

        y = cos(q_abd) * y_plane - sin(q_abd) * z_plane
        z = sin(q_abd) * y_plane + cos(q_abd) * z_plane
        return (x_plane, y, z)

    def inverse_kinematics(self, foot_position: list[float] | tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = foot_position
        geom = self.geometry
        lateral_offset = geom.side_sign * geom.abduction_offset

        yz_radius_sq = y * y + z * z
        z_plane_sq = max(yz_radius_sq - lateral_offset * lateral_offset, 1e-9)
        z_plane = -sqrt(z_plane_sq)

        q_abd = atan2(
            lateral_offset * z - z_plane * y,
            lateral_offset * y + z_plane * z,
        )

        reach = sqrt(x * x + z_plane * z_plane)
        max_reach = geom.thigh_length + geom.calf_length - 1e-6
        min_reach = abs(geom.thigh_length - geom.calf_length) + 1e-6
        reach = min(max(reach, min_reach), max_reach)

        cos_knee = (
            reach * reach - geom.thigh_length**2 - geom.calf_length**2
        ) / (2.0 * geom.thigh_length * geom.calf_length)
        cos_knee = min(1.0, max(-1.0, cos_knee))
        q_knee = -acos(cos_knee)

        target_angle = atan2(-x, -z_plane)
        correction = atan2(
            geom.calf_length * sin(q_knee),
            geom.thigh_length + geom.calf_length * cos(q_knee),
        )
        q_hip = target_angle - correction

        return (self._wrap_to_pi(q_abd), self._wrap_to_pi(q_hip), self._wrap_to_pi(q_knee))

    def jacobian(self, joint_positions: list[float] | tuple[float, float, float]) -> list[list[float]]:
        """Return a finite-difference 3x3 foot-position Jacobian."""
        epsilon = 1e-6
        base = list(joint_positions)
        jacobian_columns: list[tuple[float, float, float]] = []

        for joint_index in range(3):
            plus = base.copy()
            minus = base.copy()
            plus[joint_index] += epsilon
            minus[joint_index] -= epsilon
            p_plus = self.forward_kinematics(plus)
            p_minus = self.forward_kinematics(minus)
            jacobian_columns.append(tuple(
                (p_plus[axis] - p_minus[axis]) / (2.0 * epsilon)
                for axis in range(3)
            ))

        return [
            [jacobian_columns[col][row] for col in range(3)]
            for row in range(3)
        ]

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle
