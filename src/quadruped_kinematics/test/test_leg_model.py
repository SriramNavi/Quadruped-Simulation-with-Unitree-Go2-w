from math import isclose

from quadruped_kinematics.leg_model import LegGeometry, ThreeDofLeg


def test_fk_ik_round_trip_near_nominal_pose() -> None:
    leg = ThreeDofLeg(LegGeometry(side_sign=1.0))
    target = (0.04, 0.05, -0.30)
    joint_positions = leg.inverse_kinematics(target)
    foot = leg.forward_kinematics(joint_positions)

    assert isclose(foot[0], target[0], abs_tol=1e-4)
    assert isclose(foot[1], target[1], abs_tol=1e-4)
    assert isclose(foot[2], target[2], abs_tol=1e-4)


def test_jacobian_shape() -> None:
    leg = ThreeDofLeg()
    jacobian = leg.jacobian((0.0, 0.2, -0.5))
    assert len(jacobian) == 3
    assert all(len(row) == 3 for row in jacobian)
