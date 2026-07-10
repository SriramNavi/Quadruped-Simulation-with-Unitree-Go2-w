"""Simple swing foot trajectory generator."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SwingTrajectory:
    step_length: float = 0.08
    step_height: float = 0.04
    nominal_z: float = -0.30

    def sample(self, phase: float, nominal_x: float, nominal_y: float) -> tuple[float, float, float]:
        """Sample a parabolic swing arc for phase in [0, 1]."""
        clamped_phase = min(1.0, max(0.0, phase))
        x = nominal_x + self.step_length * (clamped_phase - 0.5)
        z_lift = 4.0 * self.step_height * clamped_phase * (1.0 - clamped_phase)
        return (x, nominal_y, self.nominal_z + z_lift)
