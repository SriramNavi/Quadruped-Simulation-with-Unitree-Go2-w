"""Static contract tests for the two-terminal IMU/IK launch."""

from pathlib import Path


LAUNCH = Path(__file__).parents[1] / 'launch' / 'go2w_imu_ik_world.launch.py'


def test_world_launch_contains_visualizers_but_not_controller():
    text = LAUNCH.read_text(encoding='utf-8')
    assert "package='rqt_graph'" in text
    assert "package='rqt_plot'" in text
    assert 'go2w_imu_ik_controller' not in text


def test_world_launch_exposes_required_arguments_and_plot_topics():
    text = LAUNCH.read_text(encoding='utf-8')
    for argument in (
        'gui', 'rviz', 'friction_preset', 'spawn_on_ramp',
        'launch_rqt_graph', 'launch_rqt_plot',
    ):
        assert f"DeclareLaunchArgument('{argument}'" in text
    assert '/go2w/imu_ik/pitch/filtered_deg/data' in text
    assert '/go2w/imu_ik/pid/output_deg/data' in text
    assert '/go2w/imu_ik/offset/front_m/data' in text
