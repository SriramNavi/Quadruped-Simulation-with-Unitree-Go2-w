"""Focused tests for world-frame RPY stabilization in the Go2-W WBC."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest

from quadruped_control.go2w_dynamic_models import TerrainEstimate
from quadruped_control.go2w_inverse_dynamics_wbc_climb import (
    ACTUATED_JOINT_NAMES,
    ContactEstimator,
    EstimatedState,
    HybridIkTarget,
    HybridPositionVelocityAdapter,
    IK_RESIDUAL_ACCEPTED,
    IK_RESIDUAL_DEGRADED,
    IK_RESIDUAL_HARD_REJECTED,
    IK_RESIDUAL_NO_CORRECTION_NEEDED,
    IK_RESIDUAL_REJECTED,
    LEG_JOINT_NAMES,
    LEGS,
    MAX_BOUND,
    maximum_qp_constraint_violation,
    qp_constraint_validation_tolerance,
    orientation_error_body,
    PostureTarget,
    QpConstraintViolation,
    QpResult,
    RobotModel,
    SlopePosturePlanner,
    TerrainSlopeDiagnostics,
    WbcConfig,
    WholeBodyQp,
)


URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / 'unitree_go2w_description/urdf/go2w_description.urdf'
)
BASE_DAE_PATH = URDF_PATH.parents[1] / 'dae/base.dae'


def posture_target() -> PostureTarget:
    """Build a fixed identity-orientation target for SO(3) tests."""
    return PostureTarget(
        desired_roll_rad=0.0,
        desired_pitch_rad=0.0,
        desired_yaw_rad=0.0,
        desired_com_position=np.zeros(3),
        desired_com_acceleration=np.zeros(3),
        longitudinal_margin_m=0.1,
        lateral_margin_m=0.1,
    )


def orientation_controller(config: WbcConfig | None = None) -> WholeBodyQp:
    """Construct only the stateless orientation-feedback portion of the QP."""
    controller = object.__new__(WholeBodyQp)
    controller.config = config or WbcConfig()
    return controller


@pytest.mark.parametrize('slope_deg', [0.0, 5.0, 15.0, 30.0])
def test_world_rpy_target_is_independent_of_terrain_slope(slope_deg):
    """Keep all configured torso angles fixed while terrain slope changes."""
    config = replace(
        WbcConfig(),
        desired_level_roll_deg=2.0,
        desired_level_pitch_deg=-3.0,
        desired_level_yaw_deg=7.0,
    )
    planner = SlopePosturePlanner(config)
    state = SimpleNamespace(
        base_position=np.zeros(3),
        base_quaternion=np.asarray((0.0, 0.0, 0.0, 1.0)),
    )
    dynamics = SimpleNamespace(
        com_position=np.asarray((0.0, 0.0, 0.3)),
        com_velocity=np.zeros(3),
    )
    contacts = {
        leg: SimpleNamespace(point=np.asarray(point, dtype=float))
        for leg, point in zip(
            LEGS,
            (
                (0.30, 0.20, 0.0),
                (0.30, -0.20, 0.0),
                (-0.30, 0.20, 0.0),
                (-0.30, -0.20, 0.0),
            ),
        )
    }
    target = planner.plan(
        state,
        dynamics,
        contacts,
        TerrainEstimate(
            fast_slope_deg=slope_deg,
            stable_slope_deg=slope_deg,
        ),
        desired_speed=0.05,
    )

    assert target.desired_roll_rad == pytest.approx(math.radians(2.0))
    assert target.desired_pitch_rad == pytest.approx(math.radians(-3.0))
    assert target.desired_yaw_rad == pytest.approx(math.radians(7.0))


@pytest.mark.parametrize(
    ('axis', 'measured_rpy'),
    (
        (0, (1.0, 0.0, 0.0)),
        (1, (0.0, 1.0, 0.0)),
        (2, (0.0, 0.0, 1.0)),
    ),
)
def test_so3_orientation_feedback_has_correct_correction_sign(
    axis,
    measured_rpy,
):
    """Correct positive measured RPY errors with negative angular targets."""
    rotation = pin.rpy.rpyToMatrix(
        *(math.radians(value) for value in measured_rpy)
    )
    angular_target = orientation_controller()._orientation_angular_target(
        rotation,
        np.zeros(3),
        posture_target(),
    )

    assert angular_target[axis] < 0.0
    assert np.delete(angular_target, axis) == pytest.approx(
        np.zeros(2), abs=1e-12,
    )


def test_so3_orientation_feedback_deadband_removes_proportional_correction():
    """Suppress proportional correction for sub-deadband orientation noise."""
    rotation = pin.rpy.rpyToMatrix(math.radians(0.1), 0.0, 0.0)
    angular_target = orientation_controller()._orientation_angular_target(
        rotation,
        np.zeros(3),
        posture_target(),
    )

    assert angular_target == pytest.approx(np.zeros(3), abs=1e-12)


def test_so3_orientation_feedback_limits_angular_acceleration_norm():
    """Bound the angular target norm under a large attitude error."""
    config = replace(WbcConfig(), maximum_base_angular_accel_radps2=2.0)
    rotation = pin.rpy.rpyToMatrix(math.radians(45.0), 0.0, 0.0)
    controller = orientation_controller(config)
    angular_target = controller._orientation_angular_target(
        rotation,
        np.zeros(3),
        posture_target(),
    )

    assert np.linalg.norm(angular_target) == pytest.approx(2.0)


def hybrid_adapter(
    config: WbcConfig | None = None,
) -> tuple[HybridPositionVelocityAdapter, SimpleNamespace]:
    """Build a publisher-free hybrid adapter with deterministic index maps."""
    robot = SimpleNamespace(
        q_indices={name: index for index, name in enumerate(LEG_JOINT_NAMES)},
        v_indices={
            name: 6 + index for index, name in enumerate(ACTUATED_JOINT_NAMES)
        },
        position_lower=np.full(len(LEG_JOINT_NAMES), -2.0),
        position_upper=np.full(len(LEG_JOINT_NAMES), 2.0),
        wheel_radius=0.1,
    )
    adapter = HybridPositionVelocityAdapter(
        node=None,
        robot=robot,
        config=config or WbcConfig(),
        publishers_enabled=False,
    )
    return adapter, robot


def hybrid_state(robot: SimpleNamespace) -> SimpleNamespace:
    """Build the state fields consumed by the hybrid adapter."""
    return SimpleNamespace(
        q=np.zeros(len(LEG_JOINT_NAMES)),
        v=np.zeros(6 + len(ACTUATED_JOINT_NAMES)),
        joint_efforts=np.zeros(len(ACTUATED_JOINT_NAMES)),
    )


@pytest.fixture(scope='module')
def robot_model() -> RobotModel:
    """Load the authoritative Go2-W model for Jacobian sign tests."""
    return RobotModel(str(URDF_PATH))


def model_state(
    robot: RobotModel,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> EstimatedState:
    """Build one finite nominal state with a controlled world attitude."""
    q = robot.nominal_q.copy()
    q[:3] = (0.0, 0.0, 0.45)
    rotation = pin.rpy.rpyToMatrix(*(
        math.radians(value) for value in (roll_deg, pitch_deg, yaw_deg)
    ))
    q[3:7] = pin.Quaternion(rotation).coeffs()
    return EstimatedState(
        stamp_sec=1.0,
        q=q,
        v=np.zeros(robot.nv),
        joint_efforts=np.zeros(len(ACTUATED_JOINT_NAMES)),
        base_position=q[:3].copy(),
        base_quaternion=q[3:7].copy(),
        base_linear_velocity_body=np.zeros(3),
        base_angular_velocity_body=np.zeros(3),
        pose_source='test',
    )


@pytest.mark.parametrize(
    ('joint_type', 'expected_axis'),
    (
        ('hip', (1.0, 0.0, 0.0)),
        ('thigh', (0.0, 1.0, 0.0)),
        ('calf', (0.0, 1.0, 0.0)),
        ('foot', (0.0, 1.0, 0.0)),
    ),
)
def test_authoritative_urdf_joint_axes(joint_type, expected_axis):
    """Pin axis allocation to the authoritative URDF parent convention."""
    root = ET.parse(URDF_PATH).getroot()
    for leg in LEGS:
        joint = root.find(f"./joint[@name='{leg}_{joint_type}_joint']")
        assert joint is not None
        axis = joint.find('axis')
        origin = joint.find('origin')
        assert axis is not None and origin is not None
        assert np.fromstring(axis.get('xyz', ''), sep=' ') == pytest.approx(
            expected_axis, abs=1.0e-12,
        )
        assert np.fromstring(origin.get('rpy', ''), sep=' ') == pytest.approx(
            np.zeros(3), abs=1.0e-12,
        )


def test_nominal_real_jacobians_separate_pitch_and_roll_joint_authority(
    robot_model,
):
    """Verify numerical wheel-center and virtual-base axis contributions."""
    q = robot_model.nominal_q.copy()
    q[:3] = (0.0, 0.0, 0.45)
    data = robot_model.model.createData()
    pin.forwardKinematics(robot_model.model, data, q)
    pin.updateFramePlacements(robot_model.model, data)
    pin.computeJointJacobians(robot_model.model, data, q)
    for leg in LEGS:
        jacobian = np.asarray(pin.getFrameJacobian(
            robot_model.model,
            data,
            robot_model.frame_ids[leg],
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        ))[:3]
        leg_indices = [
            robot_model.v_indices[f'{leg}_{joint}_joint']
            for joint in ('hip', 'thigh', 'calf')
        ]
        leg_jacobian = jacobian[:, leg_indices]
        assert np.abs(leg_jacobian) == pytest.approx(
            np.asarray((
                (0.0, 0.327658, 0.140732),
                (0.327658, 0.0, 0.0),
                (0.0955, 0.075228, 0.177345),
            )),
            abs=1.0e-6,
        )
        compensation = np.linalg.solve(
            leg_jacobian, -jacobian[:, 3:6],
        )
        assert compensation[0, 0] == pytest.approx(-1.0, abs=1.0e-12)
        assert compensation[0, 1] == pytest.approx(0.0, abs=1.0e-12)
        assert abs(compensation[0, 2]) > 0.35
        assert np.linalg.norm(compensation[1:, 1]) > 1.0


def model_contacts(robot, config, state, dynamics, slope_deg):
    """Build real contact-point Jacobians on a selected terrain plane."""
    estimator = ContactEstimator(config, robot.wheel_radius)
    estimator.slope_diagnostics = TerrainSlopeDiagnostics(
        fused_slope_deg=slope_deg,
        front_local_slope_deg=slope_deg,
        rear_local_slope_deg=slope_deg,
        front_local_slope_valid=True,
        rear_local_slope_valid=True,
    )
    return estimator.frames(
        state,
        dynamics,
        TerrainEstimate(
            fast_slope_deg=slope_deg,
            stable_slope_deg=slope_deg,
            valid=True,
        ),
    )


def measured_leg_positions(robot, state):
    """Return the controller's fixed 12-joint leg ordering."""
    return np.asarray([
        state.q[robot.q_indices[name]] for name in LEG_JOINT_NAMES
    ])


def zero_qp_result(robot):
    """Build a finite no-motion WBC result for adapter tests."""
    return QpResult(
        success=True,
        status='test',
        qdd=np.zeros(robot.nv),
        force_components=np.zeros(12),
        world_forces=np.zeros((4, 3)),
        tau=np.zeros(16),
    )


def full_authority_ik_config(**changes):
    """Use a finite correction large enough to exercise nonlinear IK."""
    defaults = {
        'hybrid_leveling_gain': 1.0,
        'hybrid_leveling_damping': 0.01,
        'maximum_leveling_orientation_step_deg': 10.0,
        'maximum_leveling_joint_correction_rad': 0.50,
        'maximum_leveling_joint_step_rad': 0.50,
        'hybrid_leveling_max_iterations': 5,
    }
    defaults.update(changes)
    return replace(WbcConfig(), **defaults)


def test_base_imu_and_torso_visual_have_no_fixed_pitch_offset(robot_model):
    """Verify controlled, IMU, collision, and visual frames are aligned."""
    root = ET.parse(URDF_PATH).getroot()
    base = root.find("./link[@name='base']")
    imu_joint = root.find("./joint[@name='imu_joint']")
    assert base is not None and imu_joint is not None
    for origin in (
        base.find('./visual/origin'),
        base.find('./collision/origin'),
        imu_joint.find('./origin'),
    ):
        assert origin is not None
        assert np.fromstring(origin.get('rpy', ''), sep=' ') == pytest.approx(
            np.zeros(3), abs=1.0e-12,
        )

    state = model_state(robot_model, roll_deg=4.0, pitch_deg=-7.0, yaw_deg=9.0)
    data = robot_model.model.createData()
    pin.framesForwardKinematics(robot_model.model, data, state.q)
    controlled = pin.Quaternion(
        state.q[6], state.q[3], state.q[4], state.q[5],
    ).matrix()
    for frame_name in ('base', 'imu'):
        frame_id = robot_model.model.getFrameId(frame_name)
        assert data.oMf[frame_id].rotation == pytest.approx(
            controlled, abs=1.0e-12,
        )

    dae_root = ET.parse(BASE_DAE_PATH).getroot()
    namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    matrix_node = dae_root.find(
        './/c:library_visual_scenes/c:visual_scene/c:node/c:matrix',
        namespace,
    )
    assert matrix_node is not None and matrix_node.text is not None
    mesh_rotation = np.fromstring(matrix_node.text, sep=' ').reshape(4, 4)[:3, :3]
    # The DAE exporter transform preserves the longitudinal x axis; it is not
    # a fixed pitch rotation that could explain the observed torso pitch.
    assert mesh_rotation @ np.asarray((1.0, 0.0, 0.0)) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1.0e-6,
    )


def test_local_world_aligned_point_jacobian_uses_body_tangent_rotation(
    robot_model,
):
    """Verify free-flyer rotational columns accept body-frame displacement."""
    config = WbcConfig()
    state = model_state(robot_model)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    frame = contacts['FL']
    frame_id = robot_model.frame_ids['FL']
    initial_data = robot_model.model.createData()
    pin.forwardKinematics(robot_model.model, initial_data, state.q)
    pin.updateFramePlacements(robot_model.model, initial_data)
    initial_placement = initial_data.oMf[frame_id]
    local_offset = initial_placement.rotation.T @ (
        frame.point - initial_placement.translation
    )
    tangent = np.zeros(robot_model.nv)
    tangent[3:6] = (0.02, -0.03, 0.01)
    epsilon = 1.0e-6
    displaced_q = pin.integrate(
        robot_model.model, state.q, epsilon * tangent,
    )
    displaced_data = robot_model.model.createData()
    pin.forwardKinematics(robot_model.model, displaced_data, displaced_q)
    pin.updateFramePlacements(robot_model.model, displaced_data)
    displaced_placement = displaced_data.oMf[frame_id]
    initial_point = (
        initial_placement.translation
        + initial_placement.rotation @ local_offset
    )
    displaced_point = (
        displaced_placement.translation
        + displaced_placement.rotation @ local_offset
    )

    finite_difference = (displaced_point - initial_point) / epsilon
    assert finite_difference == pytest.approx(
        frame.point_jacobian @ tangent, abs=1.0e-8,
    )


def test_hybrid_leveling_is_zero_at_target_orientation(robot_model):
    """Do not perturb the legs when measured and desired attitudes agree."""
    config = WbcConfig()
    state = model_state(robot_model)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 0.0)
    adapter = HybridPositionVelocityAdapter(
        None, robot_model, config, publishers_enabled=False,
    )
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )

    assert result.positions == pytest.approx(measured, abs=1.0e-12)
    assert adapter.last_leveling_orientation_body == pytest.approx(
        np.zeros(3), abs=1.0e-12,
    )


def test_independent_and_stacked_linear_leveling_solves_are_equivalent(
    robot_model,
):
    """Prove block-diagonal stacking alone cannot change the old solution."""
    config = WbcConfig()
    state = model_state(robot_model, pitch_deg=-3.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    delta_theta = orientation_error_body(
        dynamics.base_rotation, posture_target(),
    )
    damping_squared = config.hybrid_leveling_damping ** 2
    stacked_matrix = np.zeros((12, 12))
    stacked_rhs = np.zeros(12)
    independent = np.zeros(12)
    for leg_index, leg in enumerate(LEGS):
        indices = [
            robot_model.v_indices[f'{leg}_{joint}_joint']
            for joint in ('hip', 'thigh', 'calf')
        ]
        jacobian = contacts[leg].point_jacobian
        leg_jacobian = jacobian[:, indices]
        rhs = -(jacobian[:, 3:6] @ delta_theta)
        rows = slice(3 * leg_index, 3 * leg_index + 3)
        stacked_matrix[rows, rows] = leg_jacobian
        stacked_rhs[rows] = rhs
        independent[rows] = leg_jacobian.T @ np.linalg.solve(
            leg_jacobian @ leg_jacobian.T
            + damping_squared * np.eye(3),
            rhs,
        )
    stacked = stacked_matrix.T @ np.linalg.solve(
        stacked_matrix @ stacked_matrix.T
        + damping_squared * np.eye(12),
        stacked_rhs,
    )

    assert independent == pytest.approx(stacked, abs=1.0e-12)


@pytest.mark.parametrize(
    ('roll_deg', 'pitch_deg', 'axis'),
    ((-2.0, 0.0, 0), (0.0, -2.0, 1)),
)
def test_hybrid_leveling_has_restoring_roll_pitch_sign_and_fixed_contacts(
    robot_model,
    roll_deg,
    pitch_deg,
    axis,
):
    """Use real Jacobians to compensate a positive restoring body rotation."""
    config = full_authority_ik_config()
    state = model_state(robot_model, roll_deg=roll_deg, pitch_deg=pitch_deg)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(
        None, robot_model, config, publishers_enabled=False,
    )
    measured = measured_leg_positions(robot_model, state)
    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )
    adapter._record_ik_diagnostics(
        state, contacts, result, result.base_rotation, result.positions,
    )
    delta_theta = adapter.last_leveling_orientation_body

    assert orientation_error_body(dynamics.base_rotation, posture_target())[
        axis
    ] > 0.0
    assert delta_theta[axis] > 0.0
    assert np.max(adapter.last_ik_contact_residuals) <= (
        config.hybrid_leveling_contact_tolerance_m
    )
    assert np.max(np.abs(
        adapter.last_ik_wheelbase_after
        - adapter.last_ik_wheelbase_before
    )) < 0.0015
    assert np.max(np.abs(
        adapter.last_ik_axle_width_after
        - adapter.last_ik_axle_width_before
    )) < 0.0015
    assert np.all(np.isfinite(result.positions))


def test_pitch_leveling_uses_sagittal_joints_without_hip_excursion(
    robot_model,
):
    """Favor thigh/calf articulation for a pure restoring pitch motion."""
    config = full_authority_ik_config()
    state = model_state(robot_model, pitch_deg=-5.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )
    adapter._record_ik_diagnostics(
        state, contacts, result, result.base_rotation, result.positions,
    )
    correction = result.positions - measured
    hip = correction[0::3]
    sagittal = np.delete(correction.reshape(4, 3), 0, axis=1)

    assert result.pitch_requested_rad > 0.0
    assert result.pitch_accepted_fraction > 0.0
    assert adapter.last_leveling_orientation_body[1] > 0.0
    assert result.pitch_max_hip_delta_rad == pytest.approx(0.0, abs=1e-12)
    assert adapter.last_ik_pitch_hip_to_sagittal_ratio <= (
        config.hybrid_leveling_pitch_max_hip_fraction
    )
    assert np.max(np.abs(hip)) <= (
        config.hybrid_leveling_pitch_max_hip_fraction
        * np.max(np.abs(sagittal))
    )
    assert np.any(np.abs(correction[1::3]) > 0.01)
    assert np.any(np.abs(correction[2::3]) > 0.01)
    assert adapter.last_ik_final_command_residual_m <= (
        config.hybrid_leveling_contact_tolerance_m
    )
    assert np.max(np.abs(
        adapter.last_ik_wheelbase_after - adapter.last_ik_wheelbase_before
    )) < 0.0015
    assert np.max(np.abs(
        adapter.last_ik_axle_width_after - adapter.last_ik_axle_width_before
    )) < 0.0015


def test_pitch_allocation_failure_cannot_erase_the_original_demand(
    robot_model,
    monkeypatch,
):
    """Regress the runtime zero-fraction/zero-residual false no-op."""
    config = full_authority_ik_config()
    state = model_state(robot_model, pitch_deg=-2.0, yaw_deg=-1.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)
    wbc_target = measured.copy()
    # This WBC hip offset reproduces the contaminated 0.379 runtime ratio.
    wbc_target[0::3] -= 0.020
    ik_target = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), wbc_target,
    )
    lower, upper = adapter._safe_joint_bounds()
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual', lambda *_args: 0.0,
    )

    adapter._residual_safe_leg_target(
        state, dynamics, contacts, ik_target, measured,
        wbc_target, lower, upper,
    )

    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_residual_state != IK_RESIDUAL_NO_CORRECTION_NEEDED
    assert adapter.last_ik_pitch_hip_to_sagittal_ratio == pytest.approx(
        0.0, abs=1.0e-12,
    )
    assert adapter.last_ik_selected_correction_fraction > 0.0
    assert not np.allclose(ik_target.base_rotation, dynamics.base_rotation)


def test_pitch_plus_yaw_keeps_pitch_sagittal_and_leg_yaw_disabled(
    robot_model,
):
    """Prevent a small yaw error from contaminating pitch allocation."""
    config = full_authority_ik_config()
    state = model_state(robot_model, pitch_deg=-5.0, yaw_deg=-1.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )

    assert result.pitch_accepted_fraction > 0.0
    assert result.pitch_max_hip_delta_rad == pytest.approx(0.0, abs=1e-12)
    assert adapter.last_ik_pitch_hip_to_sagittal_ratio == pytest.approx(
        0.0, abs=1e-12,
    )
    assert result.leg_yaw_requested_rad == pytest.approx(0.0, abs=1e-12)
    assert result.leg_yaw_accepted_fraction == 0.0
    assert result.positions[0::3] == pytest.approx(
        measured[0::3], abs=1e-12,
    )


def test_pitch_plus_roll_sequentially_preserves_pitch_hip_exclusion(
    robot_model,
):
    """Allow roll hips only after the sagittal-only pitch subsolve."""
    config = full_authority_ik_config()
    state = model_state(robot_model, roll_deg=-2.0, pitch_deg=-5.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )
    final_residual = adapter._candidate_contact_residual(
        state, contacts, result.base_rotation, result.positions,
    )

    assert result.pitch_accepted_fraction > 0.0
    assert result.pitch_max_hip_delta_rad == pytest.approx(0.0, abs=1e-12)
    assert result.roll_accepted_fraction > 0.0
    assert result.roll_max_hip_delta_rad > 0.0
    assert adapter.last_ik_pitch_hip_to_sagittal_ratio == pytest.approx(
        0.0, abs=1e-12,
    )
    assert final_residual <= config.hybrid_leveling_contact_tolerance_m


def test_pitch_sagittal_ik_selects_largest_feasible_backtrack(robot_model):
    """Choose 0.25 when joint-correction bounds reject 1.0 and 0.5."""
    config = full_authority_ik_config(
        maximum_leveling_joint_correction_rad=0.05,
    )
    state = model_state(robot_model, pitch_deg=-5.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )

    assert result.pitch_accepted_fraction == pytest.approx(0.25)
    assert result.pitch_max_hip_delta_rad == pytest.approx(0.0, abs=1e-12)
    assert result.pitch_final_residual_m <= (
        config.hybrid_leveling_contact_tolerance_m
    )
    assert result.allocation_degraded


def test_roll_leveling_uses_bounded_left_right_articulation(robot_model):
    """Correct roll with mirrored sagittal response and bounded hip motion."""
    config = full_authority_ik_config()
    state = model_state(robot_model, roll_deg=-2.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 0.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )
    adapter._record_ik_diagnostics(
        state, contacts, result, result.base_rotation, result.positions,
    )
    correction = (result.positions - measured).reshape(4, 3)

    assert adapter.last_leveling_orientation_body[0] > 0.0
    assert np.max(np.abs(correction[:, 0])) > 0.01
    assert np.max(np.abs(correction[:, 0])) <= (
        config.maximum_roll_hip_correction_rad
    )
    assert np.sign(correction[0, 1]) == -np.sign(correction[1, 1])
    assert np.sign(correction[2, 2]) == -np.sign(correction[3, 2])
    assert adapter.last_ik_final_command_residual_m <= (
        config.hybrid_leveling_contact_tolerance_m
    )


def test_yaw_leveling_keeps_leg_ik_small_and_wheel_differential_available(
    robot_model,
):
    """Retain bounded leg yaw help while wheel velocity carries yaw authority."""
    config = WbcConfig()
    state = model_state(robot_model, yaw_deg=-3.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 0.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)
    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )

    assert adapter.last_ik_axis_mode == 'yaw'
    assert np.max(np.abs(result.positions - measured)) < 0.003
    qdd = np.zeros(robot_model.nv)
    for leg, acceleration in zip(LEGS, (-2.0, 2.0, -2.0, 2.0)):
        qdd[robot_model.v_indices[f'{leg}_foot_joint']] = acceleration
    wheel_result = replace(zero_qp_result(robot_model), qdd=qdd)
    command = adapter.propose(state, wheel_result, 0.0, 0.01)
    assert command.wheel_velocities[0] < 0.0
    assert command.wheel_velocities[1] > 0.0
    assert command.wheel_velocities[2] < 0.0
    assert command.wheel_velocities[3] > 0.0


def test_axis_aware_regularization_blends_continuously():
    """Avoid joint-target jumps while an error moves from pitch to roll."""
    adapter, _robot = hybrid_adapter()
    weights = []
    for roll_fraction in np.linspace(0.0, 1.0, 21):
        weights.append(adapter._axis_aware_joint_regularization(np.asarray((
            roll_fraction, 1.0 - roll_fraction, 0.0,
        )))[:3])
        assert np.sum(adapter.last_ik_axis_fractions) == pytest.approx(1.0)
    weights = np.asarray(weights)

    assert weights[0] == pytest.approx((50.0, 1.0, 1.0))
    assert weights[-1] == pytest.approx((2.0, 1.0, 1.0))
    assert np.max(np.abs(np.diff(weights, axis=0))) < 6.0
    assert np.all(np.isfinite(weights))
    assert np.all(weights > 0.0)
    deadband = math.radians(adapter.config.hybrid_leveling_axis_deadband_deg)
    before = adapter._axis_aware_joint_regularization(np.asarray((
        0.0, 0.1, deadband - 1.0e-8,
    )))[:3]
    after = adapter._axis_aware_joint_regularization(np.asarray((
        0.0, 0.1, deadband + 1.0e-8,
    )))[:3]
    assert after == pytest.approx(before, abs=1.0e-5)


def test_leveling_target_is_absolute_and_unwinds_without_windup(robot_model):
    """Keep a bounded target correction and unwind it when error vanishes."""
    config = full_authority_ik_config(
        maximum_leveling_joint_step_rad=0.01,
    )
    state = model_state(robot_model, pitch_deg=-3.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    result = zero_qp_result(robot_model)
    measured = measured_leg_positions(robot_model, state)
    for _ in range(20):
        adapter.propose(
            state, result, 0.0, 0.01, dynamics, contacts, posture_target(),
        )
    saturated_correction = adapter.last_leveling_correction.copy()
    assert np.max(np.abs(saturated_correction)) < (
        config.maximum_leveling_joint_correction_rad
    )

    measured_target = PostureTarget(
        desired_roll_rad=0.0,
        desired_pitch_rad=math.radians(-3.0),
        desired_yaw_rad=0.0,
        desired_com_position=np.zeros(3),
        desired_com_acceleration=np.zeros(3),
        longitudinal_margin_m=0.1,
        lateral_margin_m=0.1,
    )
    for _ in range(60):
        adapter.propose(
            state, result, 0.0, 0.01,
            dynamics, contacts, measured_target,
        )

    assert adapter.last_leveling_correction == pytest.approx(
        np.zeros(12), abs=1.0e-12,
    )
    assert adapter.last_leveling_target == pytest.approx(
        measured, abs=1.0e-12,
    )


def test_leveling_recomputes_short_lived_contacts_while_driving(robot_model):
    """Do not retain world contact targets that would freeze forward rolling."""
    config = full_authority_ik_config()
    state_a = model_state(robot_model, pitch_deg=-3.0)
    state_b_q = state_a.q.copy()
    state_b_q[0] += 0.10
    state_b = replace(
        state_a,
        stamp_sec=2.0,
        q=state_b_q,
        base_position=state_b_q[:3].copy(),
    )
    corrections = []
    target_points = []
    for state in (state_a, state_b):
        dynamics = robot_model.compute(state)
        contacts = model_contacts(
            robot_model, config, state, dynamics, 15.0,
        )
        adapter = HybridPositionVelocityAdapter(
            None, robot_model, config, False,
        )
        measured = measured_leg_positions(robot_model, state)
        result = adapter._leveling_joint_target(
            state, dynamics, contacts, posture_target(), measured,
        )
        corrections.append(result.positions - measured)
        target_points.append(np.vstack([
            contacts[leg].point for leg in LEGS
        ]))

    assert corrections[1] == pytest.approx(corrections[0], abs=1.0e-9)
    assert target_points[1][:, 0] - target_points[0][:, 0] == pytest.approx(
        np.full(4, 0.10), abs=1.0e-9,
    )


def test_joint_limit_proximity_smoothly_removes_leveling_authority(robot_model):
    """Avoid driving an already margin-limited leg farther into saturation."""
    config = full_authority_ik_config()
    state = model_state(robot_model, pitch_deg=-5.0)
    q = state.q.copy()
    q[robot_model.q_indices['FL_calf_joint']] = (
        robot_model.position_upper[2] - config.joint_limit_margin_rad
    )
    state = replace(state, q=q)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    measured = measured_leg_positions(robot_model, state)

    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )
    lower, upper = adapter._safe_joint_bounds()
    adapter._residual_safe_leg_target(
        state, dynamics, contacts, result, measured,
        measured, lower, upper,
    )

    assert result.authority_scale == pytest.approx(0.0, abs=1.0e-12)
    assert result.positions == pytest.approx(measured, abs=1.0e-12)
    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_REJECTED
    assert adapter.last_ik_residual_state != IK_RESIDUAL_NO_CORRECTION_NEEDED
    assert adapter.last_ik_pitch_requested_rad > 0.0
    assert adapter.last_ik_pitch_accepted_fraction == 0.0
    assert adapter.last_ik_pitch_max_hip_delta_rad == 0.0
    assert adapter.last_ik_pitch_final_residual_m <= (
        config.hybrid_leveling_contact_tolerance_m
    )
    assert np.all(result.positions >= lower)
    assert np.all(result.positions <= upper)


@pytest.mark.parametrize('slope_deg', [0.0, 5.0, 15.0])
def test_hybrid_leveling_is_finite_and_limited_across_terrain_planes(
    robot_model,
    slope_deg,
):
    """Keep the world target fixed while terrain-aligned contacts change."""
    config = WbcConfig()
    state = model_state(robot_model, pitch_deg=-3.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(
        robot_model, config, state, dynamics, slope_deg,
    )
    adapter = HybridPositionVelocityAdapter(
        None, robot_model, config, publishers_enabled=False,
    )

    result = SimpleNamespace(
        qdd=np.zeros(robot_model.nv),
        tau=np.zeros(len(ACTUATED_JOINT_NAMES)),
    )
    command = adapter.propose(
        state=state,
        result=result,
        effective_speed=0.0,
        dt=0.01,
        dynamics=dynamics,
        contacts=contacts,
        target=posture_target(),
    )
    correction = adapter.last_leveling_correction

    assert np.all(np.isfinite(correction))
    assert np.max(np.abs(correction)) <= (
        config.maximum_leveling_joint_step_rad + 1.0e-12
    )
    measured = np.asarray([
        state.q[robot_model.q_indices[name]] for name in LEG_JOINT_NAMES
    ])
    assert np.max(np.abs(command.leg_positions - measured)) <= (
        config.max_leg_position_step_rad + 1.0e-12
    )
    assert np.all(command.leg_positions >= (
        robot_model.position_lower + config.joint_limit_margin_rad
    ))
    assert np.all(command.leg_positions <= (
        robot_model.position_upper - config.joint_limit_margin_rad
    ))
    assert posture_target().desired_pitch_rad == 0.0


def leveling_contacts(robot, near_singular=False):
    """Build deterministic point Jacobians for adapter limit tests."""
    contacts = {}
    for leg in LEGS:
        jacobian = np.zeros((3, 6 + len(ACTUATED_JOINT_NAMES)))
        jacobian[:, 3:6] = np.eye(3)
        indices = [
            robot.v_indices[f'{leg}_{joint}_joint']
            for joint in ('hip', 'thigh', 'calf')
        ]
        jacobian[:, indices] = (
            1.0e-12 * np.eye(3) if near_singular else np.eye(3)
        )
        contacts[leg] = SimpleNamespace(point_jacobian=jacobian)
    return contacts


def test_hybrid_leveling_reduces_authority_for_singular_jacobian(
    robot_model,
    monkeypatch,
):
    """Return a finite safe target when the stacked geometry is ill-conditioned."""
    config = full_authority_ik_config()
    state = model_state(robot_model, pitch_deg=-5.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 15.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    original = adapter._ik_contact_system

    def poorly_conditioned(q, contact_frames):
        matrix, residual, points, _singular, _condition = original(
            q, contact_frames,
        )
        return 1.0e-12 * matrix, residual, points, 1.0e-12, MAX_BOUND

    monkeypatch.setattr(adapter, '_ik_contact_system', poorly_conditioned)
    measured = measured_leg_positions(robot_model, state)
    result = adapter._leveling_joint_target(
        state, dynamics, contacts, posture_target(), measured,
    )

    assert np.all(np.isfinite(result.positions))
    assert result.authority_scale < 1.0e-6
    assert result.positions == pytest.approx(measured, abs=1.0e-9)


def residual_gate_fixture(robot_model, **config_changes):
    """Build a real-model residual-gate input with a synthetic IK target."""
    config = full_authority_ik_config(
        max_leg_position_step_rad=0.004,
        maximum_leveling_joint_step_rad=0.006,
        **config_changes,
    )
    state = model_state(robot_model, pitch_deg=-2.0)
    dynamics = robot_model.compute(state)
    contacts = model_contacts(robot_model, config, state, dynamics, 0.0)
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    wbc_target = measured_leg_positions(robot_model, state)
    ik_target = HybridIkTarget(
        positions=wbc_target + 0.05,
        base_rotation=dynamics.base_rotation @ pin.exp3(
            np.asarray((0.0, 0.02, 0.0))
        ),
        iterations=1,
        condition_number=1.0,
        minimum_singular_value=0.1,
        authority_scale=1.0,
        requested_rotation_body=(0.0, 0.02, 0.0),
    )
    lower, upper = adapter._safe_joint_bounds()
    return (
        adapter, state, dynamics, contacts, ik_target,
        wbc_target, lower, upper,
    )


def run_residual_gate(
    adapter,
    state,
    dynamics,
    contacts,
    ik_target,
    wbc_target,
    lower,
    upper,
):
    """Run the exact residual gate with measured positions as its reference."""
    return adapter._residual_safe_leg_target(
        state, dynamics, contacts, ik_target, wbc_target,
        wbc_target, lower, upper,
    )


def test_final_rate_limited_command_is_backtracked_when_ik_target_passes(
    robot_model,
    monkeypatch,
):
    """Gate the exact limited command, not only the unconstrained IK result."""
    (
        adapter, state, dynamics, contacts, ik_target,
        wbc_target, lower, upper,
    ) = residual_gate_fixture(robot_model)

    def residual(_state, _contacts, _rotation, positions):
        if np.allclose(positions, ik_target.positions):
            return 0.0010
        if np.allclose(positions, wbc_target):
            return 0.0005
        return 0.0020

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    assert residual(state, contacts, ik_target.base_rotation, ik_target.positions) <= (
        adapter.config.hybrid_leveling_contact_tolerance_m
    )
    leg, _rotation = adapter._residual_safe_leg_target(
        state, dynamics, contacts, ik_target, wbc_target,
        wbc_target, lower, upper,
    )

    assert 0.0 < adapter.last_ik_selected_correction_fraction < 1.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_DEGRADED
    assert adapter.last_ik_final_command_residual_m == pytest.approx(0.0020)
    assert adapter.last_ik_final_command_residual_m <= (
        adapter.config.hybrid_leveling_contact_soft_limit_m
    )
    assert np.max(np.abs(leg - wbc_target)) <= 0.004 + 1.0e-12
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)


def test_hard_final_residual_rejects_leveling_and_uses_wbc_only_target(
    robot_model,
    monkeypatch,
):
    """Never return the leveling component when its exact residual is hard."""
    (
        adapter, state, dynamics, contacts, ik_target,
        wbc_target, lower, upper,
    ) = residual_gate_fixture(robot_model)
    calls = []

    def residual(_state, _contacts, _rotation, positions):
        calls.append(positions.copy())
        return 0.005 if np.linalg.norm(positions - wbc_target) > 1.0e-12 else 0.0005

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    leg, rotation = adapter._residual_safe_leg_target(
        state, dynamics, contacts, ik_target, wbc_target,
        wbc_target, lower, upper,
    )

    assert len(calls) == (
        adapter.config.hybrid_leveling_max_backtracking_steps + 1
    )
    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_HARD_REJECTED
    assert adapter.last_ik_candidate_residual_m > (
        adapter.config.hybrid_leveling_contact_hard_limit_m
    )
    assert adapter.last_ik_final_command_residual_m < (
        adapter.config.hybrid_leveling_contact_tolerance_m
    )
    assert leg == pytest.approx(wbc_target)
    assert rotation == pytest.approx(dynamics.base_rotation)
    assert adapter.last_leveling_correction == pytest.approx(np.zeros(12))
    assert adapter.ik_residual_rejection_count == 1


def test_residual_supervision_counts_persistence_and_scales_speed():
    """Reduce speed smoothly without turning 0.35 m/s into a fixed crawl."""
    config = replace(
        WbcConfig(), hybrid_leveling_residual_failure_cycles=3,
    )
    adapter, _robot = hybrid_adapter(config)
    adapter.last_ik_correction_requested = True
    adapter.last_ik_residual_state = IK_RESIDUAL_HARD_REJECTED
    for _ in range(3):
        adapter._update_ik_residual_supervision(
            0.0005, IK_RESIDUAL_HARD_REJECTED,
            correction_requested=True,
            nonzero_candidate_attempted=True,
            selected_fraction=0.0,
        )

    assert adapter.consecutive_ik_residual_failures == 3
    assert adapter.ik_residual_rejection_count == 3
    assert adapter.last_ik_speed_scale == pytest.approx(
        config.hybrid_ik_minimum_speed_scale
    )
    assert adapter.persistent_ik_residual_failure()
    adapter._update_ik_residual_supervision(
        0.001, IK_RESIDUAL_ACCEPTED,
        correction_requested=True,
        nonzero_candidate_attempted=True,
        selected_fraction=1.0,
    )
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)


def test_safe_degraded_correction_never_accumulates_fatal_persistence(
    robot_model,
    monkeypatch,
):
    """Keep a valid reduced-authority command healthy past the threshold."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model, hybrid_leveling_residual_failure_cycles=cycles,
    )
    adapter = fixture[0]
    ik_target = replace(
        fixture[4], authority_scale=0.5, allocation_degraded=True,
    )
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual', lambda *_args: 0.001,
    )

    for _ in range(cycles + 2):
        run_residual_gate(*fixture[:4], ik_target, *fixture[5:])

    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction == pytest.approx(0.5)
    assert adapter.last_ik_residual_state == IK_RESIDUAL_DEGRADED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.ik_degraded_count == cycles + 2
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)
    assert not adapter.persistent_ik_residual_failure()


def test_soft_degraded_correction_can_reduce_speed_without_faulting(
    robot_model,
    monkeypatch,
):
    """Allow a nonzero soft-policy command without rejection persistence."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model,
        hybrid_leveling_residual_failure_cycles=cycles,
        hybrid_ik_speed_soft_residual_m=0.0015,
    )
    adapter, _state, _dynamics, _contacts, _target, wbc_target, _lo, _hi = fixture

    def residual(_state, _contacts, _rotation, positions):
        return (
            0.002
            if np.linalg.norm(positions - wbc_target) > 1.0e-12
            else 0.0
        )

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    for _ in range(cycles + 2):
        run_residual_gate(*fixture)

    assert adapter.last_ik_selected_correction_fraction > 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_DEGRADED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.ik_degraded_count == cycles + 2
    assert (
        adapter.config.hybrid_ik_minimum_speed_scale
        <= adapter.last_ik_speed_scale <= 1.0
    )
    assert not adapter.persistent_ik_residual_failure()


def test_hard_rejection_still_reaches_fatal_persistence(
    robot_model,
    monkeypatch,
):
    """Keep hard-policy fallback rejection fatal after the threshold."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model, hybrid_leveling_residual_failure_cycles=cycles,
    )
    adapter, _state, _dynamics, _contacts, _target, wbc_target, _lo, _hi = fixture

    def residual(_state, _contacts, _rotation, positions):
        return (
            0.005
            if np.linalg.norm(positions - wbc_target) > 1.0e-12
            else 0.0005
        )

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    for _ in range(cycles):
        run_residual_gate(*fixture)

    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_HARD_REJECTED
    assert adapter.consecutive_ik_residual_failures == cycles
    assert adapter.ik_residual_rejection_count == cycles
    assert adapter.persistent_ik_residual_failure()


def test_unsafe_wbc_baseline_does_not_poison_valid_orientation_correction(
    robot_model,
    monkeypatch,
):
    """Allocate from the safe hold when the WBC-only leg target is invalid."""
    fixture = residual_gate_fixture(robot_model)
    (
        adapter, _state, dynamics, _contacts, ik_target,
        positions, lower, upper,
    ) = fixture
    wbc_target = np.clip(positions + 0.10, lower, upper)
    correction = np.zeros(12)
    correction[1::3] = 0.001
    requested = 1.05 * math.radians(max(
        adapter.config.orientation_deadband_deg,
        adapter.config.hybrid_leveling_axis_deadband_deg,
    ))
    ik_target = replace(
        ik_target,
        positions=positions + correction,
        base_rotation=dynamics.base_rotation @ pin.exp3(
            np.asarray((0.0, requested, 0.0))
        ),
        requested_rotation_body=(0.0, requested, 0.0),
        pitch_requested_rad=requested,
        pitch_accepted_fraction=1.0,
        allocation_attempted=True,
    )

    def residual(_state, _contacts, rotation, leg_positions):
        rotation_delta = float(np.linalg.norm(pin.log3(
            dynamics.base_rotation.T @ rotation
        )))
        position_delta = float(np.max(np.abs(
            leg_positions - positions
        )))
        if rotation_delta <= 1.0e-12:
            return 0.014 if position_delta > 0.003 else 0.0
        return 0.001 if position_delta <= 0.003 else 0.014

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    cycles = adapter.config.hybrid_leveling_residual_failure_cycles + 5
    for _ in range(cycles):
        leg, _rotation = adapter._residual_safe_leg_target(
            fixture[1], dynamics, fixture[3], ik_target,
            positions, wbc_target, lower, upper,
        )
        assert np.max(np.abs(leg - positions)) <= 0.001 + 1.0e-12
        assert adapter.last_ik_residual_state == IK_RESIDUAL_ACCEPTED
        assert not adapter.persistent_ik_residual_failure()

    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction == pytest.approx(1.0)
    assert adapter.last_ik_final_command_residual_m == pytest.approx(0.001)
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0


def test_valid_degraded_correction_cannot_fault_between_status_updates(
    robot_model,
    monkeypatch,
):
    """Cover one 100 Hz interval between one-second status messages."""
    fixture = residual_gate_fixture(
        robot_model, control_rate_hz=100.0, status_period_sec=1.0,
    )
    adapter = fixture[0]
    ik_target = replace(
        fixture[4], authority_scale=0.5, allocation_degraded=True,
    )
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual', lambda *_args: 0.001,
    )
    cycles_between_status = int(
        adapter.config.control_rate_hz * adapter.config.status_period_sec
    )

    for _ in range(cycles_between_status):
        run_residual_gate(*fixture[:4], ik_target, *fixture[5:])
        assert not adapter.persistent_ik_residual_failure()

    assert cycles_between_status > (
        adapter.config.hybrid_leveling_residual_failure_cycles
    )
    assert adapter.last_ik_residual_state == IK_RESIDUAL_DEGRADED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0


def test_a_startup_no_correction_remains_nonfaulting(robot_model, monkeypatch):
    """Keep a zero-demand startup healthy beyond the persistence threshold."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model, hybrid_leveling_residual_failure_cycles=cycles,
    )
    (
        adapter, _state, dynamics, _contacts, ik_target,
        wbc_target, _lower, _upper,
    ) = fixture
    ik_target = replace(
        ik_target,
        positions=wbc_target.copy(),
        base_rotation=dynamics.base_rotation.copy(),
        requested_rotation_body=(0.0, 0.0, 0.0),
    )
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual',
        lambda *_args: 0.0,
    )

    assert adapter.last_ik_residual_state == IK_RESIDUAL_NO_CORRECTION_NEEDED
    for _ in range(cycles + 2):
        run_residual_gate(*fixture[:4], ik_target, *fixture[5:])
        assert adapter.last_ik_residual_state == (
            IK_RESIDUAL_NO_CORRECTION_NEEDED
        )
        assert adapter.last_ik_selected_correction_fraction == 0.0
        assert adapter.consecutive_ik_residual_failures == 0
        assert adapter.ik_residual_rejection_count == 0
        assert adapter.last_ik_speed_scale == pytest.approx(1.0)
        assert not adapter.persistent_ik_residual_failure()


def test_b_below_deadband_correction_is_not_rejected(robot_model, monkeypatch):
    """Treat a bounded sub-deadband SO(3) delta as no correction demand."""
    fixture = residual_gate_fixture(robot_model)
    (
        adapter, _state, dynamics, _contacts, ik_target,
        wbc_target, _lower, _upper,
    ) = fixture
    tiny_delta = 1.0e-10
    ik_target = replace(
        ik_target,
        positions=wbc_target.copy(),
        base_rotation=dynamics.base_rotation @ pin.exp3(
            np.asarray((0.0, tiny_delta, 0.0))
        ),
        requested_rotation_body=(0.0, tiny_delta, 0.0),
    )
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual',
        lambda *_args: 0.0,
    )

    run_residual_gate(*fixture[:4], ik_target, *fixture[5:])

    assert not adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_NO_CORRECTION_NEEDED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)


def test_c_accepted_correction_resets_residual_persistence(
    robot_model,
    monkeypatch,
):
    """Accept a requested nonzero correction with a valid final residual."""
    fixture = residual_gate_fixture(robot_model)
    adapter = fixture[0]
    adapter.consecutive_ik_residual_failures = 2
    adapter.ik_residual_rejection_count = 2
    adapter.last_ik_speed_scale = 0.4
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual',
        lambda *_args: 0.001,
    )

    run_residual_gate(*fixture)

    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction > 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_ACCEPTED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)


def test_d_real_rejection_counts_only_requested_correction(
    robot_model,
    monkeypatch,
):
    """Count fallback only after every nonzero correction candidate fails."""
    fixture = residual_gate_fixture(robot_model)
    adapter, _state, _dynamics, _contacts, _target, wbc_target, _lo, _hi = fixture

    def residual(_state, _contacts, _rotation, positions):
        return (
            0.003
            if np.linalg.norm(positions - wbc_target) > 1.0e-12
            else 0.0
        )

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    run_residual_gate(*fixture)

    assert adapter.last_ik_correction_requested
    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_REJECTED
    assert adapter.consecutive_ik_residual_failures == 1
    assert adapter.ik_residual_rejection_count == 1


def test_e_persistent_genuine_rejection_reduces_speed_and_faults(
    robot_model,
    monkeypatch,
):
    """Preserve the speed reduction and fault gate for real rejections."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model, hybrid_leveling_residual_failure_cycles=cycles,
    )
    adapter, _state, _dynamics, _contacts, _target, wbc_target, _lo, _hi = fixture

    def residual(_state, _contacts, _rotation, positions):
        return (
            0.003
            if np.linalg.norm(positions - wbc_target) > 1.0e-12
            else 0.0
        )

    monkeypatch.setattr(adapter, '_candidate_contact_residual', residual)
    for _ in range(cycles):
        run_residual_gate(*fixture)

    assert adapter.last_ik_residual_state == IK_RESIDUAL_REJECTED
    assert adapter.consecutive_ik_residual_failures == cycles
    assert adapter.ik_residual_rejection_count == cycles
    assert adapter.last_ik_speed_scale == pytest.approx(
        adapter.config.hybrid_ik_minimum_speed_scale
    )
    assert adapter.persistent_ik_residual_failure()


def test_f_no_correction_recovers_from_genuine_rejections(
    robot_model,
    monkeypatch,
):
    """Clear stale rejection persistence on a valid no-correction cycle."""
    fixture = residual_gate_fixture(robot_model)
    (
        adapter, _state, dynamics, _contacts, ik_target,
        wbc_target, _lower, _upper,
    ) = fixture

    def rejected_residual(_state, _contacts, _rotation, positions):
        return (
            0.003
            if np.linalg.norm(positions - wbc_target) > 1.0e-12
            else 0.0
        )

    monkeypatch.setattr(
        adapter, '_candidate_contact_residual', rejected_residual,
    )
    run_residual_gate(*fixture)
    run_residual_gate(*fixture)
    no_correction_target = replace(
        ik_target,
        positions=wbc_target.copy(),
        base_rotation=dynamics.base_rotation.copy(),
        requested_rotation_body=(0.0, 0.0, 0.0),
    )
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual',
        lambda *_args: 0.0,
    )

    run_residual_gate(*fixture[:4], no_correction_target, *fixture[5:])

    assert adapter.last_ik_residual_state == IK_RESIDUAL_NO_CORRECTION_NEEDED
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)
    assert not adapter.persistent_ik_residual_failure()


def test_g_exact_zero_residual_never_reaches_fault_threshold(
    robot_model,
    monkeypatch,
):
    """Enforce that exact-zero accepted commands cannot accumulate a fault."""
    cycles = 3
    fixture = residual_gate_fixture(
        robot_model, hybrid_leveling_residual_failure_cycles=cycles,
    )
    adapter = fixture[0]
    monkeypatch.setattr(
        adapter, '_candidate_contact_residual',
        lambda *_args: 0.0,
    )

    for _ in range(cycles + 5):
        run_residual_gate(*fixture)

    assert adapter.last_ik_residual_state == IK_RESIDUAL_ACCEPTED
    assert adapter.last_ik_final_command_residual_m == 0.0
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.last_ik_speed_scale == pytest.approx(1.0)
    assert not adapter.persistent_ik_residual_failure()


def test_hybrid_leveling_and_final_commands_respect_all_position_limits():
    """Rate-limit correction and final atomic commands before joint clipping."""
    config = replace(
        WbcConfig(),
        max_leg_position_step_rad=0.004,
        maximum_leveling_joint_correction_rad=0.020,
        maximum_leveling_joint_step_rad=0.003,
        maximum_leveling_orientation_step_deg=5.0,
        hybrid_leveling_gain=1.0,
    )
    adapter, robot = hybrid_adapter(config)
    state = hybrid_state(robot)
    result = SimpleNamespace(
        qdd=np.zeros(6 + len(ACTUATED_JOINT_NAMES)),
        tau=np.zeros(len(ACTUATED_JOINT_NAMES)),
    )
    dynamics = SimpleNamespace(
        base_rotation=pin.rpy.rpyToMatrix(0.0, math.radians(-10.0), 0.0),
    )
    contacts = leveling_contacts(robot)
    previous_leveling = np.zeros(12)
    previous_command = np.zeros(12)
    for _ in range(20):
        command = adapter.propose(
            state=state,
            result=result,
            effective_speed=0.0,
            dt=0.01,
            dynamics=dynamics,
            contacts=contacts,
            target=posture_target(),
        )
        assert np.max(np.abs(
            adapter.last_leveling_correction - previous_leveling
        )) <= config.maximum_leveling_joint_step_rad + 1.0e-12
        assert np.max(np.abs(
            command.leg_positions - previous_command
        )) <= config.max_leg_position_step_rad + 1.0e-12
        assert np.all(command.leg_positions >= (
            robot.position_lower + config.joint_limit_margin_rad
        ))
        assert np.all(command.leg_positions <= (
            robot.position_upper - config.joint_limit_margin_rad
        ))
        previous_leveling = adapter.last_leveling_correction.copy()
        previous_command = command.leg_positions.copy()
    assert np.max(np.abs(adapter.last_leveling_correction)) <= (
        config.maximum_leveling_joint_correction_rad + 1.0e-12
    )


def test_high_speed_moving_reference_keeps_final_geometry_bounded(robot_model):
    """Exercise command updates at the operational 0.35 m/s target."""
    config = WbcConfig()
    adapter = HybridPositionVelocityAdapter(None, robot_model, config, False)
    previous_leg = None
    maximum_hip_correction = 0.0
    for cycle in range(30):
        pitch_deg = -2.0 * (1.0 - cycle / 29.0)
        state = model_state(robot_model, pitch_deg=pitch_deg)
        q = state.q.copy()
        q[0] = cycle * 0.35 / config.control_rate_hz
        if previous_leg is not None:
            for name, position in zip(LEG_JOINT_NAMES, previous_leg):
                q[robot_model.q_indices[name]] = position
        state = replace(
            state,
            stamp_sec=1.0 + cycle / config.control_rate_hz,
            q=q,
            base_position=q[:3].copy(),
        )
        dynamics = robot_model.compute(state)
        contacts = model_contacts(robot_model, config, state, dynamics, 0.0)
        command = adapter.propose(
            state, zero_qp_result(robot_model), 0.35,
            1.0 / config.control_rate_hz,
            dynamics, contacts, posture_target(),
        )
        previous_leg = command.leg_positions
        maximum_hip_correction = max(
            maximum_hip_correction,
            adapter.last_ik_pitch_max_hip_delta_rad,
        )
        classification_deadband = math.radians(max(
            config.orientation_deadband_deg,
            config.hybrid_leveling_axis_deadband_deg,
        ))
        if np.linalg.norm(
            adapter.last_ik_bounded_requested_rotation_body
        ) > classification_deadband:
            assert adapter.last_ik_correction_requested
            assert adapter.last_ik_pitch_accepted_fraction > 0.0
            assert adapter.last_ik_residual_state in {
                IK_RESIDUAL_ACCEPTED, IK_RESIDUAL_DEGRADED,
            }
        assert adapter.last_ik_pitch_max_hip_delta_rad == pytest.approx(
            0.0, abs=1e-12,
        )
        assert adapter.last_ik_final_command_residual_m <= (
            config.hybrid_leveling_contact_soft_limit_m
        )
        assert np.all(command.leg_positions >= (
            robot_model.position_lower + config.joint_limit_margin_rad
        ))
        assert np.all(command.leg_positions <= (
            robot_model.position_upper - config.joint_limit_margin_rad
        ))
        assert np.max(np.abs(
            adapter.last_ik_axle_width_after
            - adapter.last_ik_axle_width_before
        )) < config.hybrid_leveling_contact_hard_limit_m
    assert maximum_hip_correction == pytest.approx(0.0, abs=1e-12)
    assert np.max(np.abs(adapter.last_wheel_command)) > 0.0


def test_hybrid_zero_speed_preserves_bounded_signed_wheel_stabilization():
    """Keep signed QP wheel intent at rest while enforcing all limits."""
    adapter, robot = hybrid_adapter()
    adapter.last_wheel_command[:] = (0.0, 0.0, -0.70, 9.80)
    qdd = np.zeros(6 + len(ACTUATED_JOINT_NAMES))
    for leg, acceleration in zip(LEGS, (-10.0, 10.0, -100.0, 100.0)):
        qdd[robot.v_indices[f'{leg}_foot_joint']] = acceleration
    result = SimpleNamespace(
        qdd=qdd,
        tau=np.zeros(len(ACTUATED_JOINT_NAMES)),
    )
    previous = adapter.last_wheel_command.copy()
    command = adapter.propose(
        hybrid_state(robot), result, effective_speed=0.0, dt=0.02,
    )

    assert command.wheel_velocities[0] < 0.0
    assert command.wheel_velocities[1] > 0.0
    assert np.min(command.wheel_velocities) >= -0.75 - 1e-12
    assert np.max(command.wheel_velocities) <= 10.0 + 1e-12
    assert np.max(np.abs(command.wheel_velocities - previous)) <= 0.5 + 1e-12


def test_hybrid_stop_clears_all_wheel_commands():
    """Keep the stop path authoritative over signed stabilization motion."""
    adapter, _robot = hybrid_adapter()
    adapter.last_wheel_command[:] = (-0.4, 0.3, -0.2, 0.1)
    adapter.last_leveling_orientation_body[:] = 0.1
    adapter.last_leveling_correction[:] = 0.1
    adapter.last_leveling_target[:] = 0.1
    adapter.last_ik_contact_residuals[:] = 0.01
    adapter.last_ik_selected_correction_fraction = 0.5
    adapter.last_ik_residual_state = IK_RESIDUAL_DEGRADED
    adapter.ik_residual_rejection_count = 3
    adapter.consecutive_ik_residual_failures = 2
    adapter.last_ik_speed_scale = 0.4
    hold = np.linspace(-0.2, 0.2, 12)

    adapter.stop(hold)

    assert adapter.last_wheel_command == pytest.approx(np.zeros(4))
    assert adapter.last_leveling_orientation_body == pytest.approx(np.zeros(3))
    assert adapter.last_leveling_correction == pytest.approx(np.zeros(12))
    assert adapter.last_leveling_target == pytest.approx(np.zeros(12))
    assert adapter.last_ik_contact_residuals == pytest.approx(np.zeros(4))
    assert adapter.last_ik_selected_correction_fraction == 0.0
    assert adapter.last_ik_residual_state == IK_RESIDUAL_NO_CORRECTION_NEEDED
    assert adapter.ik_residual_rejection_count == 0
    assert adapter.consecutive_ik_residual_failures == 0
    assert adapter.last_ik_speed_scale == 1.0
    assert adapter.last_leg_command == pytest.approx(hold)


@pytest.mark.parametrize(
    ('mutate', 'category', 'name'),
    (
        (lambda force, tau: force.__setitem__(2, 0.5),
         'minimum_normal_force', 'FL'),
        (lambda force, tau: force.__setitem__(5, 181.0),
         'maximum_normal_force', 'FR'),
        (lambda force, tau: force.__setitem__(6, 13.0),
         'longitudinal_friction', 'RL'),
        (lambda force, tau: force.__setitem__(10, 11.0),
         'lateral_friction', 'RR'),
        (lambda force, tau: tau.__setitem__(0, 100.0),
         'actuator_torque', 'FL_hip_joint'),
    ),
)
def test_qp_violation_reporting_identifies_exact_bound(
    robot_model,
    mutate,
    category,
    name,
):
    """Report the specific wheel/actuator and bound behind rejection."""
    force = np.tile(np.asarray((0.0, 0.0, 10.0)), 4)
    tau = np.zeros(16)
    mutate(force, tau)

    violation = maximum_qp_constraint_violation(
        force, tau, robot_model, WbcConfig(),
    )

    assert isinstance(violation, QpConstraintViolation)
    assert violation.category == category
    assert violation.name == name
    assert violation.amount > 0.0
    if category == 'minimum_normal_force':
        assert violation.value < violation.bound
    else:
        assert violation.value > violation.bound


def test_qp_minimum_normal_postcheck_uses_newton_tolerance(robot_model):
    """Accept the observed 0.01719 N residue but reject a real force loss."""
    config = WbcConfig()
    force = np.tile(np.asarray((0.0, 0.0, 10.0)), 4)
    tau = np.zeros(16)
    force[5] = 0.98281
    near = maximum_qp_constraint_violation(
        force, tau, robot_model, config,
    )
    near_tolerance = qp_constraint_validation_tolerance(
        near, config, primal_residual=0.0019332269918752685,
    )

    assert near.category == 'minimum_normal_force'
    assert near.amount == pytest.approx(0.01719)
    assert near_tolerance == pytest.approx(0.025)
    assert near.amount <= near_tolerance

    force[5] = 0.95
    large = maximum_qp_constraint_violation(
        force, tau, robot_model, config,
    )
    assert large.amount > qp_constraint_validation_tolerance(
        large, config, primal_residual=0.0019332269918752685,
    )


@pytest.mark.parametrize(
    ('category', 'amount', 'accepted'),
    (
        ('longitudinal_friction', 0.014, True),
        ('longitudinal_friction', 0.016, False),
        ('actuator_torque', 0.014, True),
        ('actuator_torque', 0.016, False),
    ),
)
def test_qp_friction_and_torque_postchecks_keep_unit_specific_limits(
    robot_model,
    category,
    amount,
    accepted,
):
    """Do not transfer the larger normal-force allowance to other units."""
    config = WbcConfig()
    force = np.tile(np.asarray((0.0, 0.0, 10.0)), 4)
    tau = np.zeros(16)
    if category == 'longitudinal_friction':
        force[0] = config.friction_mu_longitudinal * force[2] + amount
    else:
        torque_bound = robot_model.effort_limits[0] * config.leg_torque_scale
        tau[0] = torque_bound + amount
    violation = maximum_qp_constraint_violation(
        force, tau, robot_model, config,
    )
    tolerance = qp_constraint_validation_tolerance(
        violation, config, primal_residual=2.0e-4,
    )

    assert violation.category == category
    assert (violation.amount <= tolerance) is accepted


@pytest.mark.parametrize(
    'changes',
    (
        {'orientation_deadband_deg': -0.01},
        {'maximum_base_angular_accel_radps2': 0.0},
        {
            'orientation_deadband_deg': 20.0,
            'maximum_orientation_error_deg': 20.0,
        },
        {'orientation_roll_weight_scale': 0.0},
        {'orientation_pitch_weight_scale': -1.0},
        {'orientation_yaw_weight_scale': 0.0},
        {'maximum_reverse_stabilization_wheel_speed_radps': 0.0},
        {'maximum_orientation_error_deg': 181.0},
        {'hybrid_leveling_enabled': 1},
        {'hybrid_leveling_gain': 0.0},
        {'hybrid_leveling_damping': 0.0},
        {'maximum_leveling_orientation_step_deg': 0.0},
        {'maximum_leveling_joint_correction_rad': 0.0},
        {'maximum_leveling_joint_step_rad': 0.0},
        {'hybrid_leveling_roll_scale': 0.0},
        {'hybrid_leveling_pitch_scale': 0.0},
        {'hybrid_leveling_yaw_scale': 0.0},
        {'hybrid_leveling_leg_yaw_scale': 1.1},
        {'hybrid_leveling_contact_normal_weight': 0.0},
        {'hybrid_leveling_contact_lateral_weight': -1.0},
        {'hybrid_leveling_contact_tangent_weight': 0.0},
        {'hybrid_leveling_pitch_hip_weight': 0.0},
        {'hybrid_leveling_roll_thigh_weight': 0.0},
        {'hybrid_leveling_yaw_calf_weight': 0.0},
        {'hybrid_leveling_axis_dominance_ratio': 1.0},
        {'hybrid_leveling_axis_deadband_deg': -0.1},
        {'hybrid_leveling_pitch_max_hip_fraction': 1.1},
        {'hybrid_leveling_pitch_emergency_hip_rad': -0.01},
        {'maximum_roll_hip_correction_rad': 0.0},
        {'hybrid_leveling_wbc_target_weight': 0.0},
        {'hybrid_leveling_joint_limit_avoidance_weight': 0.0},
        {'hybrid_leveling_contact_tolerance_m': 0.0},
        {'hybrid_leveling_contact_soft_limit_m': 0.0010},
        {'hybrid_leveling_contact_hard_limit_m': 0.0020},
        {'hybrid_leveling_backtracking_factor': 1.0},
        {'hybrid_leveling_max_backtracking_steps': 0},
        {'hybrid_leveling_residual_failure_cycles': 0},
        {'hybrid_ik_speed_reduction_enabled': 1},
        {'hybrid_ik_speed_soft_residual_m': 0.005},
        {'hybrid_ik_minimum_speed_scale': 0.0},
        {'normal_force_validation_tolerance_n': 0.0},
        {'friction_validation_tolerance_n': 0.0},
        {'torque_validation_tolerance_nm': 0.0},
        {'hybrid_leveling_max_iterations': 0},
        {'hybrid_leveling_max_iterations': 21},
    ),
)
def test_orientation_parameter_validation_rejects_invalid_values(changes):
    """Reject unsafe deadbands, limits, axis weights, and reverse bounds."""
    with pytest.raises(ValueError):
        replace(WbcConfig(), **changes).validate()
