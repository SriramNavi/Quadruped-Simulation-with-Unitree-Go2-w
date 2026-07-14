"""Deterministic tests for the adaptive Go2-W ramp-climb pure modules."""

from __future__ import annotations

import math

import pytest

from quadruped_control.go2w_adaptive_attitude_controller import (
    AdaptiveAttitudeController,
    AttitudeControllerConfig,
)
from quadruped_control.go2w_analytical_ik import JointSolution
from quadruped_control.go2w_dynamic_models import (
    ClimbState,
    ContactPhase,
    FeasibilityResult,
    SafetyState,
    StabilityResult,
    TerrainEstimate,
    TractionResult,
)
from quadruped_control.go2w_feasibility_solver import (
    FeasibilityConfig,
    OnlineFeasibilitySolver,
)
from quadruped_control.go2w_motion_supervisor import (
    AdaptiveClimbStateMachine,
    ClimbStateMachineConfig,
    MotionSupervisor,
    MotionSupervisorConfig,
)
from quadruped_control.go2w_stability_supervisor import (
    StabilityConfig,
    StabilitySupervisor,
    TractionConfig,
    TractionSupervisor,
)
from quadruped_control.go2w_terrain_estimator import (
    TerrainEstimator,
    TerrainEstimatorConfig,
)


LEGS = ('FL', 'FR', 'RL', 'RR')


def nominal_joints(solution: JointSolution | None = None) -> dict[str, float]:
    """Build a complete symmetric nominal joint map."""
    solution = solution or JointSolution(0.0, 0.5, -1.4)
    result = {}
    for leg in LEGS:
        for joint, value in zip(
            ('hip', 'thigh', 'calf'), solution.as_tuple(),
        ):
            result[f'{leg}_{joint}_joint'] = value
    return result


def configured_solver(**changes) -> OnlineFeasibilitySolver:
    """Create a stance-captured solver with selected config overrides."""
    values = FeasibilityConfig().__dict__ | changes
    solver = OnlineFeasibilitySolver(FeasibilityConfig(**values))
    solver.capture_stance(nominal_joints())
    return solver


def slope_trajectory(
    slope_deg: float,
    duration_sec: float = 3.0,
    noise_m: float = 0.0,
) -> TerrainEstimate:
    """Run a deterministic constant-slope pose trajectory."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    result = TerrainEstimate()
    steps = int(duration_sec / 0.02)
    for index in range(steps + 1):
        stamp = 0.001 + 0.02 * index
        progress = 0.003 * index
        noise = noise_m * math.sin(index * 1.73)
        height = progress * math.tan(math.radians(slope_deg)) + noise
        result = estimator.update(stamp, progress, 0.0, height)
    return result


@pytest.mark.parametrize('slope_deg', [0.0, 15.0, 30.0, 35.0, 45.0])
def test_terrain_estimator_known_slopes(slope_deg):
    """Estimate flat and all requested ramp angles without a lookup table."""
    result = slope_trajectory(slope_deg)
    assert result.valid
    assert result.stable_slope_deg == pytest.approx(slope_deg, abs=0.7)
    assert result.slope_confidence > 0.7


def test_terrain_estimator_rejects_insufficient_progress():
    """Reject a window that cannot geometrically constrain slope."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    result = TerrainEstimate()
    for index in range(50):
        progress = index * 0.00005
        result = estimator.update(
            0.001 + index * 0.02,
            progress,
            0.0,
            progress * math.tan(math.radians(30.0)),
        )
    assert not result.valid
    assert result.slope_confidence == 0.0


def test_terrain_estimator_is_robust_to_noise_and_outlier():
    """Retain slope accuracy with noise and a gross height outlier."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    result = TerrainEstimate()
    for index in range(180):
        progress = index * 0.003
        noise = 0.002 * math.sin(index * 1.3)
        if index == 150:
            noise += 0.20
        result = estimator.update(
            0.001 + index * 0.02,
            progress,
            0.0,
            progress * math.tan(math.radians(30.0)) + noise,
        )
    assert result.valid
    assert result.stable_slope_deg == pytest.approx(30.0, abs=1.5)


def test_terrain_estimator_rejects_nonmonotonic_time():
    """Reject time reversal deterministically."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    estimator.update(1.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match='monotonically'):
        estimator.update(0.9, 0.1, 0.0, 0.0)


def test_mixed_contact_reaches_full_slope_at_low_safe_speed():
    """Avoid a confidence/speed deadlock after credible ramp entry."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    tangent = math.tan(math.radians(15.0))
    stamp = 0.001
    progress = 0.0
    result = TerrainEstimate()
    for _ in range(500):
        stamp += 0.02
        progress += 0.002
        height = max(0.0, progress - 0.10) * tangent
        result = estimator.update(stamp, progress, 0.0, height)
        if result.contact_phase == ContactPhase.MIXED_CONTACT_ENTRY:
            break
    assert result.contact_phase == ContactPhase.MIXED_CONTACT_ENTRY
    for _ in range(2000):
        stamp += 0.02
        progress += 0.0002
        height = (progress - 0.10) * tangent
        result = estimator.update(stamp, progress, 0.0, height)
        if result.contact_phase == ContactPhase.FULL_SLOPE:
            break
    assert result.contact_phase == ContactPhase.FULL_SLOPE


def test_terrain_entry_full_slope_and_crest_phases():
    """Detect the complete entry, climb, crest, and top phase sequence."""
    estimator = TerrainEstimator(TerrainEstimatorConfig())
    estimator.reset(0.0, 0.0, 0.0, 0.0)
    phases = []
    tangent = math.tan(math.radians(30.0))
    for index in range(801):
        stamp = 0.001 + index * 0.02
        progress = index * 0.0025
        if progress < 0.20:
            height = 0.0
        elif progress < 1.20:
            height = (progress - 0.20) * tangent
        else:
            height = 1.00 * tangent
        result = estimator.update(stamp, progress, 0.0, height)
        phases.append(result.contact_phase)
    assert ContactPhase.ENTRY_DETECTED in phases
    assert ContactPhase.FRONT_AXLE_ON_RAMP in phases
    assert ContactPhase.MIXED_CONTACT_ENTRY in phases
    assert ContactPhase.FULL_SLOPE in phases
    assert ContactPhase.CREST_DETECTED in phases
    assert ContactPhase.FRONT_AXLE_ON_TOP in phases
    assert ContactPhase.MIXED_CONTACT_EXIT in phases
    assert result.contact_phase == ContactPhase.TOP_PLATFORM


def attitude_for(
    slope_deg: float,
    capability_deg: float,
    measured_pitch_deg: float = 0.0,
    pitch_rate_radps: float = 0.0,
    state: ClimbState = ClimbState.CLIMB,
    config: AttitudeControllerConfig | None = None,
):
    """Calculate one attitude command for a synthetic steady slope."""
    controller = AdaptiveAttitudeController(
        config or AttitudeControllerConfig(),
    )
    controller.update_imu(
        1.0,
        math.radians(measured_pitch_deg),
        0.0,
        pitch_rate_radps,
        0.0,
    )
    terrain = TerrainEstimate(
        fast_slope_deg=slope_deg,
        stable_slope_deg=slope_deg,
        slope_confidence=1.0,
        contact_phase=ContactPhase.FULL_SLOPE,
        valid=slope_deg >= 0.0,
        stale=False,
    )
    feasibility = FeasibilityResult(
        valid=True,
        min_correction_deg=-capability_deg,
        max_correction_deg=capability_deg,
        max_roll_correction_deg=4.0,
    )
    stability = StabilityResult(
        minimum_margin_m=0.1,
        state=SafetyState.SAFE,
    )
    return controller.compute(terrain, feasibility, stability, state, 0.01)


def test_adaptive_target_flat_and_sufficient_15_degree_workspace():
    """Keep the feasible torso target level when correction is sufficient."""
    flat = attitude_for(0.0, 15.0)
    slope = attitude_for(15.0, 15.0)
    assert flat.feasible_target_pitch_deg == pytest.approx(0.0)
    assert slope.feasible_target_pitch_deg == pytest.approx(0.0)
    assert slope.feedforward_deg == pytest.approx(15.0)


@pytest.mark.parametrize(
    ('slope', 'capability', 'target'),
    [(30.0, 15.0, -15.0), (45.0, 15.0, -30.0)],
)
def test_adaptive_target_uses_online_capability(slope, capability, target):
    """Permit residual body slope when online correction is limited."""
    command = attitude_for(slope, capability, measured_pitch_deg=target)
    assert command.feasible_target_pitch_deg == pytest.approx(target)
    assert command.final_correction_deg == pytest.approx(capability)


def test_adaptive_target_rejects_negative_or_invalid_slope_feedforward():
    """Do not generate uphill feedforward from a negative estimate."""
    command = attitude_for(-10.0, 15.0)
    assert command.feedforward_deg == 0.0
    assert command.feasible_target_pitch_deg == 0.0


def test_adaptive_target_changes_continuously_with_feasibility():
    """Vary target continuously with online capability."""
    targets = [
        attitude_for(30.0, capability).feasible_target_pitch_deg
        for capability in (10.0, 12.0, 14.0, 16.0)
    ]
    assert targets == pytest.approx([-20.0, -18.0, -16.0, -14.0])


def test_feedforward_feedback_residual_and_same_cycle_response():
    """Apply residual feedback in the feedforward calculation cycle."""
    command = attitude_for(15.0, 20.0, measured_pitch_deg=-2.0)
    assert command.feedforward_deg == pytest.approx(15.0)
    assert command.feedback_deg > 0.0
    assert command.final_correction_deg > command.feedforward_deg


def test_derivative_damping_and_output_feasibility_clamp():
    """Damp measured pitch rate and clamp output to online limits."""
    damped = attitude_for(
        15.0, 20.0, measured_pitch_deg=0.0, pitch_rate_radps=0.5,
    )
    clamped = attitude_for(30.0, 10.0, measured_pitch_deg=-30.0)
    assert damped.d_term_deg < 0.0
    assert clamped.final_correction_deg <= 10.0


def test_tiny_pitch_error_does_not_bang_bang_saturate():
    """Keep a tiny residual error away from the hard output bound."""
    command = attitude_for(0.0, 20.0, measured_pitch_deg=-0.1)
    assert 0.0 < command.final_correction_deg < 0.2
    assert not command.saturated


def test_feasibility_nominal_positive_negative_and_branch_continuity():
    """Search both directions and preserve the captured IK branch."""
    solver = configured_solver()
    envelope = solver.envelope(
        0.0, ContactPhase.FLAT_APPROACH, 0.0, force=True,
    )
    assert envelope.valid
    assert envelope.min_correction_deg < -10.0
    assert envelope.max_correction_deg > 10.0
    positive = solver.solve_command(10.0, 0.0)
    negative = solver.solve_command(-10.0, 0.0)
    assert positive.valid and negative.valid
    assert len(positive.positions) == len(negative.positions) == 12
    zero = solver.solve_command(0.0, 0.0)
    assert zero.valid
    assert zero.positions == pytest.approx(tuple(nominal_joints().values()))


def test_feasibility_reports_limit_and_preserves_all_four_leg_atomicity():
    """Scale atomically and reject a stance with one unreachable leg."""
    solver = configured_solver(absolute_pitch_cap_deg=30.0)
    result = solver.solve_command(60.0, 0.0)
    assert result.valid
    assert 0.0 < result.scale < 1.0
    assert len(result.positions) == 12
    assert result.limiting_leg in LEGS
    solver.nominal_feet['FL'] = (2.0, 0.0955, -0.1)
    failed = solver.solve_command(5.0, 0.0)
    assert not failed.valid
    assert failed.positions == ()
    assert failed.limiting_leg == 'FL'


def test_feasibility_enforces_joint_margin_and_front_rear_offsets():
    """Enforce joint margin and opposite front/rear pitch offsets."""
    with pytest.raises(ValueError, match='lacks configured joint margin'):
        solver = OnlineFeasibilitySolver(
            FeasibilityConfig(joint_limit_margin_rad=0.10),
        )
        solver.capture_stance(nominal_joints(JointSolution(0.0, 0.5, -2.70)))
    solver = configured_solver()
    result = solver.solve_command(10.0, 0.0)
    assert result.front_offset_m > 0.0
    assert result.rear_offset_m < 0.0
    assert result.joint_margin_rad >= solver.config.joint_limit_margin_rad


def nominal_feet() -> dict[str, tuple[float, float, float]]:
    """Return nominal FK wheel-center positions for stability tests."""
    solver = configured_solver()
    return dict(solver.nominal_feet)


def test_stability_safe_caution_stop_and_infeasible():
    """Classify each stability severity with finite margins."""
    feet = nominal_feet()
    safe = StabilitySupervisor(StabilityConfig()).evaluate(
        feet, 0.0, 0.0, 0.0, 0.0, ContactPhase.FULL_SLOPE,
    )
    caution = StabilitySupervisor(StabilityConfig(
        caution_margin_m=0.15,
        stop_margin_m=0.10,
    )).evaluate(feet, 0.0, 0.0, 0.0, 0.0, ContactPhase.FULL_SLOPE)
    stop = StabilitySupervisor(StabilityConfig(
        caution_margin_m=0.16,
        stop_margin_m=0.14,
    )).evaluate(feet, 0.0, 0.0, 0.0, 0.0, ContactPhase.FULL_SLOPE)
    infeasible = StabilitySupervisor(StabilityConfig(
        base_com_x_m=0.50,
    )).evaluate(feet, 0.0, 0.0, 0.0, 0.0, ContactPhase.FULL_SLOPE)
    assert safe.state == SafetyState.SAFE
    assert caution.state == SafetyState.CAUTION
    assert stop.state == SafetyState.STOP
    assert infeasible.state == SafetyState.INFEASIBLE
    assert all(math.isfinite(value) for value in (
        safe.longitudinal_margin_m,
        safe.lateral_margin_m,
        safe.minimum_margin_m,
    ))


def test_mixed_contact_uses_more_conservative_support_margin():
    """Inset longitudinal support during mixed contact."""
    supervisor = StabilitySupervisor(StabilityConfig())
    full = supervisor.evaluate(
        nominal_feet(), 0.0, 0.0, 0.0, 0.0, ContactPhase.FULL_SLOPE,
    )
    mixed = supervisor.evaluate(
        nominal_feet(), 0.0, 0.0, 0.0, 0.0,
        ContactPhase.MIXED_CONTACT_ENTRY,
    )
    assert mixed.longitudinal_margin_m < full.longitudinal_margin_m


def test_traction_pure_rolling_wheel_spin_braking_and_zero_exclusion():
    """Measure rolling, spin, braking slip, and exclude standstill."""
    supervisor = TractionSupervisor(TractionConfig())
    omega = 0.20 / 0.086
    rolling = supervisor.evaluate(0.0, [omega] * 4, 0.20, 0.20)
    spin = supervisor.evaluate(0.1, [2.0 * omega] * 4, 0.20, 0.20)
    braking = supervisor.evaluate(0.2, [0.5 * omega] * 4, 0.20, 0.20)
    stopped = supervisor.evaluate(0.3, [0.0] * 4, 0.0, 0.0)
    residual = supervisor.evaluate(0.4, [1.0] * 4, 0.0, 0.0)
    passive_recoil = supervisor.evaluate(0.5, [0.0] * 4, -0.20, 0.0)
    assert rolling.slips == pytest.approx([0.0] * 4)
    assert spin.slips == pytest.approx([0.5] * 4)
    assert braking.slips == pytest.approx([-0.5] * 4)
    assert stopped.slips == pytest.approx([0.0] * 4)
    assert stopped.confidence == 0.0
    assert residual.slips == pytest.approx([0.0] * 4)
    assert residual.confidence == 0.0
    assert passive_recoil.slips == pytest.approx([0.0] * 4)
    assert passive_recoil.confidence == 0.0


def test_sustained_slip_stops_then_reports_infeasible():
    """Escalate held wheel spin from stop to infeasible."""
    supervisor = TractionSupervisor(TractionConfig(
        slip_hold_sec=0.20,
        infeasible_hold_sec=0.50,
    ))
    result = TractionResult()
    for index in range(8):
        result = supervisor.evaluate(index * 0.10, [8.0] * 4, 0.05, 0.10)
    assert result.state == SafetyState.INFEASIBLE
    assert result.stop_reason == 'sustained_inadequate_traction'


def test_traction_pause_does_not_false_escalate_and_retry_can_recover():
    """Do not turn a commanded recovery pause into false slip evidence."""
    supervisor = TractionSupervisor(TractionConfig(
        slip_hold_sec=0.20,
        infeasible_hold_sec=0.50,
    ))
    supervisor.evaluate(0.0, [8.0] * 4, 0.05, 0.10)
    supervisor.evaluate(0.1, [8.0] * 4, 0.05, 0.10)
    stopped = supervisor.evaluate(10.0, [0.0] * 4, 0.0, 0.0)
    omega = 0.10 / 0.086
    retry = supervisor.evaluate(10.1, [8.0] * 4, 0.05, 0.10)
    recovered = supervisor.evaluate(10.2, [omega] * 4, 0.10, 0.10)
    assert stopped.state == SafetyState.CAUTION
    assert stopped.stop_reason == 'traction_recovery_pause'
    assert retry.state != SafetyState.INFEASIBLE
    assert recovered.state == SafetyState.SAFE


def motion_inputs(
    slope=0.0,
    phase=ContactPhase.FLAT_APPROACH,
    pitch_error=0.0,
    margin=0.10,
    stability_state=SafetyState.SAFE,
    slip=0.0,
    traction_state=SafetyState.SAFE,
    workspace=0.0,
):
    """Build pure module inputs for motion-scheduler tests."""
    return (
        TerrainEstimate(
            fast_slope_deg=slope,
            stable_slope_deg=slope,
            slope_confidence=1.0,
            contact_phase=phase,
            valid=True,
            stale=False,
        ),
        pitch_error,
        StabilityResult(
            minimum_margin_m=margin,
            state=stability_state,
        ),
        TractionResult(
            average_abs_slip=slip,
            state=traction_state,
        ),
        FeasibilityResult(valid=True, workspace_usage=workspace),
    )


def update_motion(supervisor, now, inputs, drive=True, dt=1.0):
    """Invoke the motion scheduler with fixed heading inputs."""
    terrain, pitch, stability, traction, feasibility = inputs
    return supervisor.update(
        now, dt, terrain, pitch, stability, traction, feasibility,
        0.0, 0.0, drive,
    )


def test_motion_flat_steep_mixed_pitch_and_workspace_scaling():
    """Reduce flat speed continuously for each requested risk factor."""
    config = MotionSupervisorConfig(
        requested_speed_mps=0.20,
        wheel_accel_limit_mps2=10.0,
        wheel_decel_limit_mps2=10.0,
    )
    flat = update_motion(MotionSupervisor(config), 0.0, motion_inputs())
    steep = update_motion(
        MotionSupervisor(config), 0.0, motion_inputs(slope=40.0),
    )
    mixed = update_motion(
        MotionSupervisor(config), 0.0,
        motion_inputs(phase=ContactPhase.MIXED_CONTACT_ENTRY),
    )
    pitch = update_motion(
        MotionSupervisor(config), 0.0, motion_inputs(pitch_error=10.0),
    )
    workspace = update_motion(
        MotionSupervisor(config), 0.0, motion_inputs(workspace=0.75),
    )
    assert flat.effective_speed_mps == pytest.approx(0.20)
    assert steep.effective_speed_mps < flat.effective_speed_mps
    assert mixed.effective_speed_mps < flat.effective_speed_mps
    assert pitch.effective_speed_mps < flat.effective_speed_mps
    assert workspace.effective_speed_mps < flat.effective_speed_mps
    assert all(value >= 0.0 for value in mixed.wheel_linear_mps)


def test_motion_stability_traction_stop_and_controlled_resume():
    """Pause on safety stop and resume only after a safe hold."""
    config = MotionSupervisorConfig(
        requested_speed_mps=0.20,
        wheel_accel_limit_mps2=10.0,
        wheel_decel_limit_mps2=10.0,
        safe_pause_hold_sec=0.20,
    )
    supervisor = MotionSupervisor(config)
    stopped = update_motion(
        supervisor,
        0.0,
        motion_inputs(margin=0.0, stability_state=SafetyState.STOP),
    )
    still_paused = update_motion(supervisor, 0.10, motion_inputs())
    update_motion(supervisor, 0.25, motion_inputs())
    resumed = update_motion(supervisor, 0.35, motion_inputs())
    assert stopped.effective_speed_mps == 0.0
    assert still_paused.paused
    assert resumed.effective_speed_mps > 0.0
    traction_stop = update_motion(
        MotionSupervisor(config),
        0.0,
        motion_inputs(traction_state=SafetyState.STOP),
    )
    assert traction_stop.effective_speed_mps == 0.0
    caution_crawl = update_motion(
        MotionSupervisor(config),
        0.0,
        motion_inputs(slip=0.8, traction_state=SafetyState.CAUTION),
    )
    assert 0.0 < caution_crawl.effective_speed_mps < 0.20


def run_success_sequence(machine: AdaptiveClimbStateMachine, slope_label: int):
    """Drive the pure state machine through a successful top sequence."""
    del slope_label
    machine.update(0.0, ContactPhase.FLAT_APPROACH, data_ready=True)
    machine.update(0.1, ContactPhase.FLAT_APPROACH, start_requested=True)
    machine.update(0.3, ContactPhase.FLAT_APPROACH, level_start=True)
    machine.update(0.4, ContactPhase.ENTRY_DETECTED)
    machine.update(0.5, ContactPhase.FRONT_AXLE_ON_RAMP)
    machine.update(0.6, ContactPhase.FULL_SLOPE)
    machine.update(0.7, ContactPhase.CREST_DETECTED)
    machine.update(0.8, ContactPhase.TOP_PLATFORM)
    machine.update(0.9, ContactPhase.TOP_PLATFORM, wheels_stopped=True)
    return machine.update(
        1.2,
        ContactPhase.TOP_PLATFORM,
        wheels_stopped=True,
        posture_neutral=True,
    )


@pytest.mark.parametrize('slope_label', [15, 30])
def test_state_machine_successful_adaptive_sequences(slope_label):
    """Use one state algorithm for representative 15 and 30 degree runs."""
    machine = AdaptiveClimbStateMachine(ClimbStateMachineConfig(
        startup_settle_sec=0.1,
        top_hold_sec=0.1,
        maximum_run_time_sec=5.0,
    ))
    assert run_success_sequence(machine, slope_label) == ClimbState.COMPLETE


def test_state_machine_safe_infeasible_fault_and_timeout_sequences():
    """Keep physical infeasibility distinct from faults and timeout."""
    infeasible = AdaptiveClimbStateMachine(ClimbStateMachineConfig())
    infeasible.update(0.0, ContactPhase.FLAT_APPROACH, data_ready=True)
    assert infeasible.update(
        0.1, ContactPhase.FLAT_APPROACH, infeasible=True,
    ) == ClimbState.SAFE_INFEASIBLE
    fault = AdaptiveClimbStateMachine(ClimbStateMachineConfig())
    assert fault.update(
        0.0, ContactPhase.FLAT_APPROACH, fault=True,
    ) == ClimbState.FAULT
    timeout = AdaptiveClimbStateMachine(ClimbStateMachineConfig(
        startup_settle_sec=0.0,
        maximum_run_time_sec=0.5,
    ))
    timeout.update(0.0, ContactPhase.FLAT_APPROACH, data_ready=True)
    timeout.update(0.1, ContactPhase.FLAT_APPROACH, start_requested=True)
    assert timeout.update(
        0.7, ContactPhase.FLAT_APPROACH,
    ) == ClimbState.TIMEOUT


def test_restricted_workspace_45_degree_sequence_stops_infeasible():
    """Stop a 45 degree run when deliberate workspace is insufficient."""
    solver = configured_solver(absolute_pitch_cap_deg=5.0)
    envelope = solver.envelope(
        0.0, ContactPhase.FLAT_APPROACH, 0.0, force=True,
    )
    command = attitude_for(
        45.0,
        envelope.max_correction_deg,
        measured_pitch_deg=-40.0,
    )
    assert envelope.max_correction_deg <= 5.0
    assert command.feasible_target_pitch_deg <= -40.0
    machine = AdaptiveClimbStateMachine(ClimbStateMachineConfig(
        startup_settle_sec=0.0,
    ))
    machine.update(0.0, ContactPhase.FLAT_APPROACH, data_ready=True)
    machine.update(0.1, ContactPhase.FLAT_APPROACH, start_requested=True)
    machine.update(0.2, ContactPhase.FLAT_APPROACH)
    machine.update(0.3, ContactPhase.ENTRY_DETECTED)
    machine.update(0.4, ContactPhase.FRONT_AXLE_ON_RAMP)
    machine.update(0.5, ContactPhase.FULL_SLOPE)
    assert machine.update(
        0.6, ContactPhase.FULL_SLOPE, infeasible=True,
    ) == ClimbState.SAFE_INFEASIBLE


def test_synthetic_slope_sweep_remains_finite_feasible_and_nonnegative():
    """Sweep 0--50 degrees without NaN, invalid IK, or wheel reversal."""
    solver = configured_solver()
    stability_supervisor = StabilitySupervisor(StabilityConfig())
    motion_config = MotionSupervisorConfig(
        requested_speed_mps=0.10,
        wheel_accel_limit_mps2=10.0,
        wheel_decel_limit_mps2=10.0,
    )
    for slope in [index * 0.5 for index in range(101)]:
        envelope = solver.envelope(
            slope + 1.0,
            ContactPhase.FULL_SLOPE,
            slope,
            force=True,
        )
        target_pitch = -max(0.0, slope - envelope.max_correction_deg)
        command = attitude_for(
            slope,
            envelope.max_correction_deg,
            measured_pitch_deg=target_pitch,
        )
        joint_command = solver.solve_command(
            command.final_correction_deg,
            command.roll_correction_deg,
        )
        assert joint_command.valid
        assert all(math.isfinite(value) for value in joint_command.positions)
        assert (
            envelope.min_correction_deg
            <= command.final_correction_deg
            <= envelope.max_correction_deg
        )
        stability = stability_supervisor.evaluate(
            solver.nominal_feet,
            0.0,
            math.radians(target_pitch),
            0.0,
            0.0,
            ContactPhase.FULL_SLOPE,
        )
        motion = update_motion(
            MotionSupervisor(motion_config),
            slope + 1.0,
            motion_inputs(
                slope=slope,
                margin=stability.minimum_margin_m,
                stability_state=stability.state,
            ),
        )
        assert math.isfinite(command.feasible_target_pitch_deg)
        assert math.isfinite(motion.effective_speed_mps)
        assert motion.effective_speed_mps >= 0.0
        if stability.state in (SafetyState.STOP, SafetyState.INFEASIBLE):
            assert motion.effective_speed_mps == 0.0
