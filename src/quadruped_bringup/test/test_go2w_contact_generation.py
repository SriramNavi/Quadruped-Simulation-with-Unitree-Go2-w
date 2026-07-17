"""Regression tests for isolated Go2-W friction description generation."""

import hashlib
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import pytest

from quadruped_control.go2w_friction_presets import (
    generate_runtime_descriptions,
    load_configuration,
    resolve_friction,
)

import yaml


SRC = Path(__file__).resolve().parents[2]
BRINGUP = SRC / 'quadruped_bringup'
DESCRIPTION = SRC / 'unitree_go2w_description'
PRESETS = DESCRIPTION / 'config' / 'go2w_friction_presets.yaml'
XACRO = DESCRIPTION / 'urdf' / 'go2w_gazebo_control.urdf.xacro'
URDF = DESCRIPTION / 'urdf' / 'go2w_description.urdf'
CONTROLLERS = DESCRIPTION / 'config' / 'go2w_ros2_control.yaml'
WORLD = BRINGUP / 'worlds' / 'go2w_slope_test.sdf'
LAUNCH = BRINGUP / 'launch' / 'go2w_reference.launch.py'
AUTHORITATIVE_FILES = (PRESETS, XACRO, URDF, WORLD, LAUNCH)
WHEELS = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')
FRICTION_TAGS = {'mu', 'mu2', 'slip1', 'slip2'}


def _hashes() -> dict[Path, str]:
    return {
        path: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in AUTHORITATIVE_FILES
    }


def _generate(configuration, preset, overrides=None):
    return generate_runtime_descriptions(
        friction=resolve_friction(configuration, preset, overrides),
        source_world=WORLD,
        control_xacro=XACRO,
        source_urdf=URDF,
        controllers_file=CONTROLLERS,
    )


def _without_friction(path: Path) -> bytes:
    root = deepcopy(ET.parse(path).getroot())
    for element in root.iter():
        if element.tag in FRICTION_TAGS:
            element.text = 'FRICTION'
    return ET.tostring(root)


def _assert_robot(runtime, expected):
    root = ET.parse(runtime.robot_sdf).getroot()
    for wheel in WHEELS:
        link = root.find(f".//link[@name='{wheel}']")
        assert link is not None
        collision = link.find('collision')
        cylinder = collision.find('./geometry/cylinder')
        assert float(cylinder.findtext('radius')) == pytest.approx(0.086)
        assert float(cylinder.findtext('length')) == pytest.approx(0.052)
        assert collision.find('./geometry/mesh') is None
        assert link.find('./visual/geometry/mesh') is not None
        ode = collision.find('./surface/friction/ode')
        assert float(ode.findtext('mu')) == expected['wheel_mu_longitudinal']
        assert float(ode.findtext('mu2')) == expected['wheel_mu_lateral']
        assert float(ode.findtext('slip1')) == expected[
            'wheel_slip_longitudinal'
        ]
        assert float(ode.findtext('slip2')) == expected['wheel_slip_lateral']
    assert root.find(".//sensor[@name='imu_sensor']") is not None
    control_plugin = root.find(
        ".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
    )
    assert control_plugin is not None
    assert control_plugin.findtext('controller_manager_name') == (
        'go2w_controller_manager'
    )
    assert control_plugin.findtext('ros/remapping') == (
        '/robot_description:=/go2w/robot_description'
    )


def _assert_world(runtime, expected):
    root = ET.parse(runtime.world_sdf).getroot()
    surfaces = root.findall('.//collision/surface/friction/ode')
    assert surfaces
    for ode in surfaces:
        assert float(ode.findtext('mu')) == expected['terrain_mu_longitudinal']
        assert float(ode.findtext('mu2')) == expected['terrain_mu_lateral']


def test_generation_is_unique_correct_and_source_immutable():
    """Generate four variants and prove geometry and sources are unchanged."""
    before = _hashes()
    configuration = load_configuration(PRESETS)
    runtimes = [
        _generate(configuration, 'dry_concrete'),
        _generate(configuration, 'zero_friction'),
        _generate(configuration, 'dry_concrete', {
            'wheel_mu_longitudinal': '0.35',
            'wheel_mu_lateral': '0.30',
            'terrain_mu_longitudinal': '0.35',
            'terrain_mu_lateral': '0.30',
        }),
        _generate(configuration, 'dry_concrete', {
            name: '0.0' for name in (
                'wheel_mu_longitudinal', 'wheel_mu_lateral',
                'terrain_mu_longitudinal', 'terrain_mu_lateral',
                'wheel_slip_longitudinal', 'wheel_slip_lateral',
            )
        }),
    ]
    try:
        assert len({runtime.directory for runtime in runtimes}) == len(
            runtimes
        )
        assert all(str(runtime.directory).startswith('/tmp/go2w_friction_')
                   for runtime in runtimes)
        for runtime in runtimes:
            assert runtime.robot_sdf.is_file()
            assert runtime.world_sdf.is_file()
            assert runtime.effective_parameters.is_file()
            _assert_robot(runtime, runtime.friction.values)
            _assert_world(runtime, runtime.friction.values)
            snapshot = yaml.safe_load(runtime.effective_parameters.read_text())
            assert snapshot['requested_preset'] == (
                runtime.friction.requested_preset
            )
            assert snapshot['effective_parameters'] == runtime.friction.values
            assert snapshot['generated_robot_sdf'] == str(runtime.robot_sdf)
            assert snapshot['generated_world_sdf'] == str(runtime.world_sdf)

        dry, zero, custom, explicit_zero = runtimes
        assert dry.robot_sdf.read_bytes() != zero.robot_sdf.read_bytes()
        assert dry.world_sdf.read_bytes() != zero.world_sdf.read_bytes()
        assert _without_friction(dry.robot_sdf) == _without_friction(
            zero.robot_sdf
        )
        assert _without_friction(dry.world_sdf) == _without_friction(
            zero.world_sdf
        )
        assert custom.friction.values['wheel_mu_longitudinal'] == 0.35
        assert custom.friction.values['wheel_slip_longitudinal'] == 0.0001
        for name in (
            'wheel_mu_longitudinal', 'wheel_mu_lateral',
            'terrain_mu_longitudinal', 'terrain_mu_lateral',
            'wheel_slip_longitudinal', 'wheel_slip_lateral',
        ):
            assert explicit_zero.friction.values[name] == 0.0
        launch_text = LAUNCH.read_text(encoding='utf-8')
        assert '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU' in launch_text
        assert 'GO2W_CONTROLLER_MANAGER = "/go2w_controller_manager"' in (
            launch_text
        )
        assert 'GO2W_ROBOT_DESCRIPTION_TOPIC = ' in launch_text
        assert _hashes() == before
    finally:
        directories = [runtime.directory for runtime in runtimes]
        for runtime in runtimes:
            runtime.cleanup()
        assert all(not directory.exists() for directory in directories)


@pytest.mark.parametrize('value', ['nan', 'inf', '-0.1', 'not-a-number'])
def test_invalid_override_is_rejected(value):
    """Reject malformed, non-finite, and negative override values."""
    configuration = load_configuration(PRESETS)
    with pytest.raises(ValueError):
        resolve_friction(
            configuration, 'dry_concrete', {'wheel_mu_longitudinal': value},
        )


def test_unknown_preset_lists_valid_names():
    """Unknown preset errors enumerate valid terminal choices."""
    configuration = load_configuration(PRESETS)
    with pytest.raises(
        ValueError, match='available:.*dry_concrete.*zero_friction',
    ):
        resolve_friction(configuration, 'missing')
