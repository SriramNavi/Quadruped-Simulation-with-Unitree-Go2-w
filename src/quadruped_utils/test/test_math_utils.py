from quadruped_utils.math_utils import clamp, saturate_sequence


def test_clamp() -> None:
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def test_saturate_sequence() -> None:
    assert saturate_sequence([-2.0, 0.5, 3.0], 1.0) == [-1.0, 0.5, 1.0]
