import math

import pytest

from quadruped_control.go2w_analytical_ik import (
    AnalyticalLegFK,
    AnalyticalLegIK,
    IKError,
    JointSolution,
    LegGeometry,
)


LEGS = ('FL', 'FR', 'RL', 'RR')


@pytest.mark.parametrize('leg', LEGS)
@pytest.mark.parametrize('joints', [
    JointSolution(0.0, 0.5, -1.4),
    JointSolution(0.08, 0.62, -1.55),
    JointSolution(-0.08, 0.42, -1.25),
])
def test_fk_ik_round_trip(leg, joints):
    geometry = LegGeometry.for_leg(leg)
    target = AnalyticalLegFK(geometry).solve(joints)
    solved = AnalyticalLegIK(geometry).solve(target, joints)
    rebuilt = AnalyticalLegFK(geometry).solve(solved)
    assert math.dist(target, rebuilt) <= 1.0e-5
    assert all(math.isfinite(value) for value in solved.as_tuple())


@pytest.mark.parametrize('leg', LEGS)
@pytest.mark.parametrize('delta_z', [-0.02, 0.0, 0.02])
def test_height_corrected_stance_is_reachable(leg, delta_z):
    geometry = LegGeometry.for_leg(leg)
    fk = AnalyticalLegFK(geometry)
    target = list(fk.solve(JointSolution(0.0, 0.5, -1.4)))
    target[2] += delta_z
    solution = AnalyticalLegIK(geometry).solve(target)
    assert math.dist(target, fk.solve(solution)) <= 1.0e-5


def test_left_right_symmetry():
    left = LegGeometry.for_leg('FL')
    right = LegGeometry.for_leg('FR')
    q = JointSolution(0.1, 0.55, -1.45)
    lp = AnalyticalLegFK(left).solve(q)
    rp = AnalyticalLegFK(right).solve(JointSolution(-q.hip, q.thigh, q.calf))
    assert rp == pytest.approx((lp[0], -lp[1], lp[2]), abs=1.0e-12)


def test_previous_solution_preserves_branch():
    geometry = LegGeometry.for_leg('FL')
    previous = JointSolution(0.05, 0.6, -1.5)
    target = AnalyticalLegFK(geometry).solve(previous)
    solution = AnalyticalLegIK(geometry).solve(target, previous)
    assert solution.as_tuple() == pytest.approx(previous.as_tuple(), abs=1.0e-9)


@pytest.mark.parametrize('target', [
    (0.0, 0.0, 0.0),
    (1.0, 0.0955, -0.1),
    (math.nan, 0.1, -0.3),
    (math.inf, 0.1, -0.3),
])
def test_invalid_or_unreachable_targets_are_rejected(target):
    with pytest.raises(IKError):
        AnalyticalLegIK(LegGeometry.for_leg('FL')).solve(target)


def test_joint_limits_reject_wrong_knee_branch():
    geometry = LegGeometry.for_leg('FL')
    target = AnalyticalLegFK(geometry).solve(JointSolution(0.0, 0.5, -1.4))
    solution = AnalyticalLegIK(geometry).solve(target)
    assert geometry.calf_limits[0] <= solution.calf <= geometry.calf_limits[1]
    assert solution.calf < 0.0
