from quadruped_gait.gait_patterns import duty_factor_for_gait, phase_offset_for_gait
from quadruped_gait.swing_trajectory import SwingTrajectory


def test_trot_diagonal_pairs_share_phase() -> None:
    assert phase_offset_for_gait('trot', 'front_left') == phase_offset_for_gait('trot', 'rear_right')
    assert phase_offset_for_gait('trot', 'front_right') == phase_offset_for_gait('trot', 'rear_left')


def test_crawl_has_longer_duty_factor_than_trot() -> None:
    assert duty_factor_for_gait('crawl') > duty_factor_for_gait('trot')


def test_swing_trajectory_lifts_mid_phase() -> None:
    trajectory = SwingTrajectory(step_height=0.05, nominal_z=-0.30)
    start = trajectory.sample(0.0, 0.0, 0.0)
    middle = trajectory.sample(0.5, 0.0, 0.0)
    assert middle[2] > start[2]
