"""Small math helpers used across controller prototypes."""

from __future__ import annotations

from collections.abc import Iterable


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value into the inclusive range [lower, upper]."""
    if lower > upper:
        raise ValueError('lower bound must not be greater than upper bound')
    return max(lower, min(upper, value))


def saturate(value: float, limit: float) -> float:
    """Clamp a scalar to +/- limit."""
    abs_limit = abs(limit)
    return clamp(value, -abs_limit, abs_limit)


def saturate_sequence(values: Iterable[float], limit: float) -> list[float]:
    """Clamp a sequence of scalars to +/- limit."""
    return [saturate(float(value), limit) for value in values]


def vector_norm(values: Iterable[float]) -> float:
    """Return the Euclidean norm of a short numeric sequence."""
    return sum(float(value) ** 2 for value in values) ** 0.5
