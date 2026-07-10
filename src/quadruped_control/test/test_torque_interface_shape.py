from quadruped_control.defaults import DEFAULT_JOINT_NAMES
from quadruped_utils.math_utils import saturate_sequence


def test_default_joint_list_has_twelve_joints() -> None:
    assert len(DEFAULT_JOINT_NAMES) == 12


def test_torque_saturation_preserves_order() -> None:
    efforts = saturate_sequence([-30.0, 0.0, 40.0], 20.0)
    assert efforts == [-20.0, 0.0, 20.0]
