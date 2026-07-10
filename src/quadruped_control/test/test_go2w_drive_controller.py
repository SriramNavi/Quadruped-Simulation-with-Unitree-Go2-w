"""Unit tests for Go2-W drive command shaping helpers."""

from dataclasses import replace

import pytest

from quadruped_control.go2w_cmd_vel_to_wheels import (
    curvature_preserving_saturation,
    differential_drive_wheel_speeds,
    DriveCommandState,
    DriveLimits,
    limit_angular_velocity,
    rate_limit,
)


LIMITS = DriveLimits()


def wheel_speeds(linear: float, angular: float) -> tuple[float, float]:
    """Use the default Go2-W geometry for a test body command."""
    return differential_drive_wheel_speeds(
        linear,
        angular,
        LIMITS.wheel_radius_m,
        LIMITS.track_width_m,
    )


def test_straight_forward_has_equal_positive_wheel_speeds() -> None:
    left, right = wheel_speeds(0.5, 0.0)
    assert left == pytest.approx(right)
    assert left > 0.0


def test_straight_reverse_has_equal_negative_wheel_speeds() -> None:
    left, right = wheel_speeds(-0.5, 0.0)
    assert left == pytest.approx(right)
    assert left < 0.0


def test_left_arc_drives_right_wheel_faster() -> None:
    left, right = wheel_speeds(0.5, 0.2)
    assert right > left


def test_right_arc_drives_left_wheel_faster() -> None:
    left, right = wheel_speeds(0.5, -0.2)
    assert left > right


def test_wheel_saturation_scales_both_sides_by_same_factor() -> None:
    left, right = curvature_preserving_saturation(20.0, 5.0, 10.0)
    assert left == pytest.approx(10.0)
    assert right == pytest.approx(2.5)
    assert left / 20.0 == pytest.approx(right / 5.0)


def test_minimum_turning_radius_limits_high_speed_yaw() -> None:
    radius_limited = replace(
        LIMITS,
        max_angular_velocity_at_zero_speed_radps=10.0,
        max_angular_velocity_at_max_speed_radps=10.0,
        max_angular_velocity_radps=10.0,
    )
    linear = 1.0
    angular = limit_angular_velocity(linear, 10.0, radius_limited)
    assert angular == pytest.approx(linear / LIMITS.minimum_turning_radius_m)


def test_emergency_stop_makes_wheel_output_exactly_zero() -> None:
    state = DriveCommandState(LIMITS)
    state.set_target(1.0, 0.2)
    assert state.update(0.1) != (0.0, 0.0)

    state.set_emergency_stop(True)

    assert state.update(0.1) == (0.0, 0.0)
    assert state.current_left_wheel_command == 0.0
    assert state.current_right_wheel_command == 0.0


def test_stale_command_decelerates_toward_zero() -> None:
    state = DriveCommandState(LIMITS)
    state.set_target(1.0, 0.0)
    initial_left, initial_right = state.update(0.1)

    stale_left, stale_right = state.update(0.01, command_stale=True)

    assert 0.0 <= stale_left < initial_left
    assert 0.0 <= stale_right < initial_right
    assert state.target_linear_velocity == 0.0
    assert state.target_angular_velocity == 0.0


def test_rate_limiter_respects_rate_over_elapsed_time() -> None:
    assert rate_limit(0.0, 10.0, maximum_rate=2.0, elapsed_sec=0.25) == 0.5
    assert rate_limit(1.0, -10.0, maximum_rate=2.0, elapsed_sec=0.25) == 0.5
