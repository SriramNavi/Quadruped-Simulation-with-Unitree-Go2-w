"""Focused deterministic tests for Go2-W slope monitor helper logic."""

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from quadruped_control.go2w_slope_test_monitor import (
    JOINT_NAMES,
    Go2WSlopeTestMonitor,
    PrecheckConfig,
    PrecheckSample,
    RAMP_GEOMETRY,
    TrialStateMachine,
    automation_safety_abort_reason,
    body_up_vector_vertical_component,
    calculate_deceleration_start_x,
    command_speed_for_state,
    evaluate_precheck,
    extract_joint_values,
    motion_command_publication_enabled,
    pose_matches_expected_spawn,
    precheck_blockers,
    quaternion_to_rpy,
    rate_limited_speed,
    ros_timeout_elapsed,
    warning_threshold_active,
)


def quaternion_for_roll(angle: float) -> tuple[float, float, float, float]:
    return math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)


def quaternion_for_pitch(angle: float) -> tuple[float, float, float, float]:
    return 0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0)


def started_machine(**overrides) -> TrialStateMachine:
    values = {'geometry': RAMP_GEOMETRY[15], 'lane_y': -6.0}
    values.update(overrides)
    machine = TrialStateMachine(**values)
    machine.step(0.0, data_ready=True, moving_toward_ramp=True)
    return machine


def upright_sample(machine: TrialStateMachine, time_sec: float, **overrides) -> str:
    values = {
        'now_sec': time_sec,
        'data_ready': True,
        'moving_toward_ramp': True,
        'roll_deg': 0.0,
        'pitch_deg': 0.0,
        'base_z': 0.4,
        'body_up_alignment': 1.0,
        'base_x': 0.0,
        'base_y': -6.0,
    }
    values.update(overrides)
    return machine.step(**values)


def valid_precheck_sample(**overrides) -> PrecheckSample:
    values = {
        'odometry_valid': True,
        'joint_states_valid': True,
        'odometry_fresh': True,
        'joint_states_fresh': True,
        'cmd_vel_subscriber_count': 1,
        'emergency_stop_active': False,
        'base_x_m': -3.0,
        'base_y_m': -6.0,
        'yaw_deg': 0.0,
        'roll_deg': 0.0,
        'pitch_deg': 0.0,
        'planar_speed_mps': 0.0,
    }
    values.update(overrides)
    return PrecheckSample(**values)


def precheck_config() -> PrecheckConfig:
    return PrecheckConfig(
        expected_x_m=-3.0,
        expected_y_m=-6.0,
        expected_yaw_deg=0.0,
        position_tolerance_m=0.35,
        yaw_tolerance_deg=5.0,
        stationary_speed_threshold_mps=0.03,
        upright_roll_tolerance_deg=5.0,
        upright_pitch_tolerance_deg=5.0,
    )


def safety_reason(**overrides) -> str:
    values = {
        'odometry_fresh': True,
        'joint_states_fresh': True,
        'cmd_vel_subscriber_available': True,
        'require_cmd_vel_subscriber': True,
        'lane_error_m': 0.0,
        'heading_error_deg': 0.0,
        'max_lane_error_m': 0.75,
        'max_heading_error_deg': 15.0,
    }
    values.update(overrides)
    return automation_safety_abort_reason(**values)


def test_identity_quaternion_has_zero_rpy() -> None:
    assert quaternion_to_rpy(0.0, 0.0, 0.0, 1.0) == pytest.approx((0.0, 0.0, 0.0))


def test_known_roll_quaternion_converts_correctly() -> None:
    roll = math.radians(35.0)
    assert quaternion_to_rpy(*quaternion_for_roll(roll))[0] == pytest.approx(roll)


def test_known_pitch_quaternion_converts_correctly() -> None:
    pitch = math.radians(30.0)
    assert quaternion_to_rpy(*quaternion_for_pitch(pitch))[1] == pytest.approx(pitch)


def test_upright_body_up_alignment_is_one() -> None:
    assert body_up_vector_vertical_component(0.0, 0.0, 0.0, 1.0) == pytest.approx(1.0)


def test_body_up_alignment_decreases_when_tipped() -> None:
    tipped = body_up_vector_vertical_component(*quaternion_for_roll(math.radians(60.0)))
    assert tipped == pytest.approx(0.5)
    assert tipped < 1.0


def test_joint_extraction_uses_names_when_reordered() -> None:
    names = list(reversed(JOINT_NAMES))
    positions = [float(index) for index in range(len(names))]
    values = extract_joint_values(names, positions, positions, positions)
    assert values[JOINT_NAMES[0]][0] == pytest.approx(15.0)
    assert values[JOINT_NAMES[-1]][0] == pytest.approx(0.0)


def test_missing_effort_is_nan() -> None:
    values = extract_joint_values(
        [JOINT_NAMES[0]], [1.0], [2.0], [], [JOINT_NAMES[0]],
    )
    assert math.isnan(values[JOINT_NAMES[0]][2])


def test_warning_thresholds_activate() -> None:
    assert warning_threshold_active(30.0, 0.0, 30.0, 45.0)
    assert warning_threshold_active(0.0, -45.0, 30.0, 45.0)
    assert not warning_threshold_active(29.9, 44.9, 30.0, 45.0)


def test_transient_topple_sample_does_not_declare_toppled() -> None:
    machine = started_machine(topple_hold_sec=0.25)
    state = upright_sample(machine, 0.1, roll_deg=60.0)
    assert state == 'TOPPLING'
    assert upright_sample(machine, 0.2) == 'RUNNING'


def test_sustained_topple_condition_declares_toppled() -> None:
    machine = started_machine(topple_hold_sec=0.25)
    assert upright_sample(machine, 0.1, roll_deg=60.0) == 'TOPPLING'
    assert upright_sample(machine, 0.36, roll_deg=60.0) == 'TOPPLED'
    assert machine.topple_trigger == 'ROLL_LIMIT'


def test_success_requires_lane_region_upright_and_dwell() -> None:
    machine = started_machine(success_hold_sec=1.0, lane_tolerance_m=0.5)
    x = (RAMP_GEOMETRY[15].success_min_x + RAMP_GEOMETRY[15].success_max_x) / 2.0
    platform_z = RAMP_GEOMETRY[15].platform_top_height + 0.3
    assert upright_sample(
        machine, 1.0, base_x=x, base_y=-5.0, base_z=platform_z,
    ) == 'RUNNING'
    assert upright_sample(
        machine, 2.0, base_x=x, base_y=-6.0, base_z=platform_z,
    ) == 'RUNNING'
    assert upright_sample(
        machine, 2.5, base_x=x, base_y=-6.0, base_z=platform_z,
    ) == 'RUNNING'
    assert upright_sample(
        machine, 3.0, base_x=x, base_y=-6.0, base_z=platform_z,
    ) == 'SUCCESS'


def test_timeout_produces_timeout() -> None:
    machine = started_machine(trial_timeout_sec=2.0)
    assert upright_sample(machine, 2.0) == 'TIMEOUT'


def test_slide_back_after_forward_progress_is_recorded() -> None:
    machine = started_machine(slide_back_distance_m=0.75)
    upright_sample(machine, 1.0, base_x=2.0)
    upright_sample(machine, 2.0, base_x=4.0)
    upright_sample(machine, 3.0, base_x=3.0)
    assert machine.slide_back_detected


def test_automated_precheck_passes_for_valid_stationary_start() -> None:
    assert evaluate_precheck(valid_precheck_sample(), precheck_config()) == (True, '')


def test_multiple_precheck_blockers_are_reported_independently() -> None:
    sample = valid_precheck_sample(
        odometry_valid=False,
        joint_states_valid=False,
        cmd_vel_subscriber_count=0,
        emergency_stop_active=True,
    )
    assert precheck_blockers(sample, precheck_config()) == (
        'WAITING_FOR_ODOMETRY',
        'WAITING_FOR_JOINT_STATES',
        'EMERGENCY_STOP_ACTIVE',
        'CMD_VEL_NO_SUBSCRIBER',
    )


def test_no_odometry_at_startup_does_not_timeout_early() -> None:
    start, elapsed = ros_timeout_elapsed(None, 10.0)
    assert elapsed == 0.0
    start, elapsed = ros_timeout_elapsed(start, 39.999)
    assert elapsed < 30.0
    assert precheck_blockers(
        valid_precheck_sample(odometry_valid=False), precheck_config(),
    )[0] == 'WAITING_FOR_ODOMETRY'


def test_simulation_time_zero_does_not_start_or_expire_precheck_timeout() -> None:
    start, elapsed = ros_timeout_elapsed(None, 0.0)
    assert start is None
    assert elapsed == 0.0
    start, elapsed = ros_timeout_elapsed(start, 100.0)
    assert start == 100.0
    assert elapsed == 0.0


def test_first_odometry_before_timeout_permits_precheck_progression() -> None:
    waiting = valid_precheck_sample(odometry_valid=False)
    assert 'WAITING_FOR_ODOMETRY' in precheck_blockers(waiting, precheck_config())
    assert precheck_blockers(valid_precheck_sample(), precheck_config()) == ()


def test_missing_joint_states_at_startup_is_recoverable() -> None:
    blockers = precheck_blockers(
        valid_precheck_sample(joint_states_valid=False), precheck_config(),
    )
    assert blockers == ('WAITING_FOR_JOINT_STATES',)


def test_precheck_timeout_default_is_loaded_from_yaml() -> None:
    config_path = Path(__file__).parents[1] / 'config' / 'go2w_slope_test_monitor.yaml'
    parameters = yaml.safe_load(config_path.read_text())[
        'go2w_slope_test_monitor'
    ]['ros__parameters']
    assert parameters['precheck_timeout_sec'] == pytest.approx(30.0)


def test_ground_truth_index_must_match_configured_spawn() -> None:
    assert pose_matches_expected_spawn((-3.0, -6.0, 0.4), -3.0, -6.0, 0.75)
    assert not pose_matches_expected_spawn((0.0, 0.0, 0.0), -3.0, -6.0, 0.75)


@pytest.mark.parametrize(
    ('sample', 'reason'),
    [
        (valid_precheck_sample(base_y_m=-5.0), 'START_POSITION_INVALID'),
        (valid_precheck_sample(yaw_deg=6.0), 'START_HEADING_INVALID'),
        (valid_precheck_sample(roll_deg=6.0), 'ROBOT_NOT_UPRIGHT'),
        (valid_precheck_sample(planar_speed_mps=0.04), 'ROBOT_NOT_STATIONARY'),
    ],
)
def test_automated_precheck_rejects_invalid_start(sample, reason) -> None:
    assert evaluate_precheck(sample, precheck_config()) == (False, reason)


def test_acceleration_is_rate_limited_and_clamped_to_target() -> None:
    speed = rate_limited_speed(0.0, 0.25, 0.20, 0.5)
    assert speed == pytest.approx(0.10)
    assert rate_limited_speed(speed, 0.25, 0.20, 2.0) == pytest.approx(0.25)


def test_deceleration_is_rate_limited_and_clamped_to_zero() -> None:
    speed = rate_limited_speed(0.25, 0.0, 0.35, 0.5)
    assert speed == pytest.approx(0.075)
    assert rate_limited_speed(speed, 0.0, 0.35, 1.0) == pytest.approx(0.0)


def test_topple_timeout_and_abort_states_force_zero_command() -> None:
    assert command_speed_for_state(0.25, 'RUNNING', True) == 0.0
    assert command_speed_for_state(0.25, 'TIMEOUT') == 0.0
    assert command_speed_for_state(0.25, 'ABORTED') == 0.0


def test_lane_deviation_requests_abort() -> None:
    assert safety_reason(lane_error_m=0.76) == 'LANE_DEVIATION'


def test_stale_odometry_after_trial_start_requests_abort() -> None:
    assert safety_reason(odometry_fresh=False) == 'ODOMETRY_TIMEOUT'


def test_stale_joint_states_request_abort() -> None:
    assert safety_reason(joint_states_fresh=False) == 'JOINT_STATES_TIMEOUT'


def test_success_deceleration_starts_before_safe_platform_end_limit() -> None:
    geometry = RAMP_GEOMETRY[15]
    start_x = calculate_deceleration_start_x(geometry, 0.25, 0.35, 0.50)
    stopping_distance = 0.25 ** 2 / (2.0 * 0.35)
    assert start_x <= geometry.success_min_x
    assert start_x + stopping_distance <= geometry.platform_end_x - 0.50


def test_shutdown_state_forces_zero_command() -> None:
    assert command_speed_for_state(0.25, 'ABORTED') == 0.0
    fake_monitor = SimpleNamespace(
        auto_drive_enabled=True,
        current_commanded_speed_mps=0.25,
        _publish_drive_command=Mock(),
    )
    Go2WSlopeTestMonitor._publish_zero(fake_monitor)
    fake_monitor._publish_drive_command.assert_called_once_with(0.0)


def test_safety_abort_path_publishes_zero() -> None:
    machine = SimpleNamespace(state='RUNNING', abort=Mock())
    fake_monitor = SimpleNamespace(
        machine=machine,
        abort_reason='',
        state_data_timeouts=[],
        automation_phase='CRUISING',
        automation_phase_at_result='',
        _publish_zero=Mock(),
        _publish_emergency_stop=Mock(),
        _begin_stopping=Mock(),
    )
    Go2WSlopeTestMonitor._set_abort(
        fake_monitor, 'ODOMETRY_TIMEOUT', 12.0, state_data_timeout=True,
    )
    fake_monitor._publish_zero.assert_called_once_with()
    fake_monitor._publish_emergency_stop.assert_called_once_with(True)
    assert fake_monitor.state_data_timeouts == ['ODOMETRY_TIMEOUT']


def test_passive_mode_command_path_is_explicitly_disabled() -> None:
    assert not motion_command_publication_enabled(False)
    fake_monitor = SimpleNamespace(
        auto_drive_enabled=False,
        command_publisher=Mock(),
    )
    Go2WSlopeTestMonitor._publish_drive_command(fake_monitor, 0.25)
    fake_monitor.command_publisher.publish.assert_not_called()
