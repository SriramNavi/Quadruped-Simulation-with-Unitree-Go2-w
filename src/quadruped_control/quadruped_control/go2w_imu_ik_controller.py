"""IMU pitch controller with exclusive direct analytical IK output for Go2-W."""

from __future__ import annotations

import math
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from quadruped_control.go2w_analytical_ik import (
    AnalyticalLegFK,
    AnalyticalLegIK,
    IKError,
    JointSolution,
    LegGeometry,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

LEGS = ('FL', 'FR', 'RL', 'RR')
JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in LEGS for joint in ('hip', 'thigh', 'calf')
)
TUNABLE_PARAMETERS = {
    'target_pitch_deg', 'kp', 'ki', 'kd', 'pitch_filter_cutoff_hz',
    'max_body_correction_deg', 'max_leg_offset_m', 'max_offset_rate_mps',
}


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


class Go2WImuIkController(Node):
    def __init__(self) -> None:
        super().__init__(
            'go2w_imu_ik_controller',
            automatically_declare_parameters_from_overrides=False,
        )
        defaults: dict[str, Any] = {
            'mode': 'shadow',
            'enabled_on_start': False,
            'imu_topic': '/imu/data',
            'joint_state_topic': '/joint_states',
            'command_backend': 'direct_leg_position_controller',
            'command_topic': '/go2w_leg_position_controller/commands',
            'expected_imu_frame': 'imu',
            'target_pitch_deg': 0.0,
            'control_rate_hz': 100.0,
            'pitch_filter_cutoff_hz': 5.0,
            'kp': 0.25,
            'ki': 0.0,
            'kd': 0.04,
            'integral_limit': 0.20,
            'max_body_correction_deg': 5.0,
            'max_leg_offset_m': 0.020,
            'max_offset_rate_mps': 0.020,
            'imu_timeout_sec': 0.20,
            'joint_state_timeout_sec': 0.20,
            'max_message_dt_sec': 0.10,
            'max_roll_deg': 35.0,
            'max_pitch_deg': 40.0,
            'max_saturation_sec': 3.0,
            'max_consecutive_ik_failures': 3,
            'workspace_margin_m': 1.0e-5,
            'diagnostics_rate_hz': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._load_parameters()
        if self.mode not in ('monitor', 'shadow', 'active'):
            raise ValueError('mode must be monitor, shadow, or active')
        if self.command_backend != 'direct_leg_position_controller':
            raise ValueError('only the proven direct_leg_position_controller backend is supported')

        self.geometries = {leg: LegGeometry.for_leg(leg) for leg in LEGS}
        self.fk = {leg: AnalyticalLegFK(self.geometries[leg]) for leg in LEGS}
        self.ik = {
            leg: AnalyticalLegIK(self.geometries[leg], self.workspace_margin_m)
            for leg in LEGS
        }
        self.raw_pitch: float | None = None
        self.filtered_pitch: float | None = None
        self.pitch_rate: float | None = None
        self.roll: float | None = None
        self.last_imu_stamp: float | None = None
        self.last_imu_receive: float | None = None
        self.last_joint_receive: float | None = None
        self.imu_valid = False
        self.imu_fault = 'waiting for IMU'
        self.latest_joints: dict[str, float] = {}
        self.nominal_feet: dict[str, tuple[float, float, float]] = {}
        self.previous_solutions: dict[str, JointSolution] = {}
        self.integral = 0.0
        self.correction_angle = 0.0
        self.last_control_time: float | None = None
        self.enabled = False
        self.enable_pending = bool(self.get_parameter('enabled_on_start').value)
        self.disabled_ramp_pending = False
        self.fault_latched = ''
        self.ik_failures = 0
        self.saturation_start: float | None = None
        self.last_state = 'waiting for IMU'
        self.last_terms = (0.0, 0.0, 0.0, 0.0)
        self.last_offsets = (0.0, 0.0)
        self.ik_valid = False
        self.saturated = False

        self.command_publisher = self.create_publisher(
            Float64MultiArray, self.command_topic, 10,
        )
        self.target_publisher = self.create_publisher(
            JointState, '/go2w/imu_ik/joint_targets', 10,
        )
        self.scalar_publishers = {
            key: self.create_publisher(Float64, topic, 10)
            for key, topic in {
                'raw': '/go2w/imu_ik/pitch/raw_deg',
                'filtered': '/go2w/imu_ik/pitch/filtered_deg',
                'target': '/go2w/imu_ik/pitch/target_deg',
                'error': '/go2w/imu_ik/pitch/error_deg',
                'rate': '/go2w/imu_ik/pitch/rate_radps',
                'p': '/go2w/imu_ik/pid/p_term',
                'i': '/go2w/imu_ik/pid/i_term',
                'd': '/go2w/imu_ik/pid/d_term',
                'output': '/go2w/imu_ik/pid/output_deg',
                'front': '/go2w/imu_ik/offset/front_m',
                'rear': '/go2w/imu_ik/offset/rear_m',
            }.items()
        }
        self.enabled_publisher = self.create_publisher(Bool, '/go2w/imu_ik/enabled', 10)
        self.saturation_publisher = self.create_publisher(Bool, '/go2w/imu_ik/saturated', 10)
        self.ik_valid_publisher = self.create_publisher(Bool, '/go2w/imu_ik/ik_valid', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/go2w/imu_ik/diagnostics', 10,
        )
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 20)
        self.create_subscription(JointState, self.joint_state_topic, self._on_joints, 20)
        self.create_service(SetBool, '/go2w/imu_ik/enable', self._on_enable)
        self.create_service(Trigger, '/go2w/imu_ik/reset', self._on_reset)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(1.0 / self.control_rate_hz, self._control)
        self.create_timer(1.0 / self.diagnostics_rate_hz, self._publish_diagnostics)
        self.get_logger().info(
            f'mode={self.mode} backend={self.command_backend} target='
            f'{self.target_pitch_deg:.3f} deg kp={self.kp:.3f} ki={self.ki:.3f} kd={self.kd:.3f}'
        )

    def _load_parameters(self) -> None:
        for name in (
            'mode', 'imu_topic', 'joint_state_topic', 'command_backend',
            'command_topic', 'expected_imu_frame',
        ):
            setattr(self, name, str(self.get_parameter(name).value))
        for name in (
            'target_pitch_deg', 'control_rate_hz', 'pitch_filter_cutoff_hz',
            'kp', 'ki', 'kd', 'integral_limit', 'max_body_correction_deg',
            'max_leg_offset_m', 'max_offset_rate_mps', 'imu_timeout_sec',
            'joint_state_timeout_sec', 'max_message_dt_sec', 'max_roll_deg',
            'max_pitch_deg', 'max_saturation_sec', 'workspace_margin_m',
            'diagnostics_rate_hz',
        ):
            setattr(self, name, float(self.get_parameter(name).value))
        self.max_consecutive_ik_failures = int(
            self.get_parameter('max_consecutive_ik_failures').value
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_imu(self, message: Imu) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        q = message.orientation
        values = (
            q.x, q.y, q.z, q.w,
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z, message.linear_acceleration.x,
            message.linear_acceleration.y, message.linear_acceleration.z,
        )
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        frame_ok = (
            not self.expected_imu_frame
            or message.header.frame_id == self.expected_imu_frame
        )
        monotonic = self.last_imu_stamp is None or stamp >= self.last_imu_stamp
        valid = all((
            stamp > 0.0,
            all(math.isfinite(value) for value in values),
            abs(norm - 1.0) <= 0.02,
            frame_ok,
            monotonic,
        ))
        if not valid:
            self.imu_valid = False
            if not frame_ok:
                self.imu_fault = f'unexpected IMU frame {message.header.frame_id!r}'
            elif not monotonic:
                self.imu_fault = 'simulation time moved backward'
                self._fault(self.imu_fault)
            else:
                self.imu_fault = 'invalid IMU message'
            return
        roll, pitch, _ = quaternion_to_rpy(q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        dt = None if self.last_imu_stamp is None else stamp - self.last_imu_stamp
        if self.filtered_pitch is None or dt is None or dt > self.max_message_dt_sec:
            self.filtered_pitch = pitch
        elif dt > 0.0:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.pitch_filter_cutoff_hz * dt)
            self.filtered_pitch += alpha * (pitch - self.filtered_pitch)
        self.raw_pitch = pitch
        self.roll = roll
        self.pitch_rate = float(message.angular_velocity.y)
        self.last_imu_stamp = stamp
        self.last_imu_receive = self._now()
        self.imu_valid = True
        self.imu_fault = ''

    def _on_joints(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions and math.isfinite(positions[name]) for name in JOINT_NAMES):
            self.latest_joints = {name: float(positions[name]) for name in JOINT_NAMES}
            self.last_joint_receive = self._now()

    def _fresh(self, received: float | None, timeout: float, now: float) -> bool:
        return received is not None and 0.0 <= now - received <= timeout

    def _backend_ready(self) -> tuple[bool, str]:
        if self.command_publisher.get_subscription_count() < 1:
            return False, 'leg position controller is not subscribed'
        publishers = self.get_publishers_info_by_topic(self.command_topic)
        if len(publishers) > 1:
            return False, 'another leg-command publisher is present'
        return True, ''

    def _capture_nominal(self) -> bool:
        if not self.latest_joints:
            return False
        feet: dict[str, tuple[float, float, float]] = {}
        solutions: dict[str, JointSolution] = {}
        try:
            for leg in LEGS:
                solution = JointSolution(*(
                    self.latest_joints[f'{leg}_{joint}_joint']
                    for joint in ('hip', 'thigh', 'calf')
                ))
                geometry = self.geometries[leg]
                within_limits = all((
                    geometry.hip_limits[0] <= solution.hip <= geometry.hip_limits[1],
                    geometry.thigh_limits[0] <= solution.thigh <= geometry.thigh_limits[1],
                    geometry.calf_limits[0] <= solution.calf <= geometry.calf_limits[1],
                ))
                if not within_limits:
                    return False
                feet[leg] = self.fk[leg].solve(solution)
                solutions[leg] = solution
        except (KeyError, ValueError):
            return False
        self.nominal_feet = feet
        self.previous_solutions = solutions
        return True

    def _prerequisites(self, now: float, require_backend: bool) -> tuple[bool, str]:
        if not self.imu_valid:
            return False, self.imu_fault or 'waiting for IMU'
        if not self._fresh(self.last_imu_receive, self.imu_timeout_sec, now):
            return False, 'stale IMU'
        if not self._fresh(self.last_joint_receive, self.joint_state_timeout_sec, now):
            return False, 'stale joint states'
        if self.roll is None or abs(math.degrees(self.roll)) > self.max_roll_deg:
            return False, 'roll safety limit exceeded'
        pitch_outside_limit = (
            self.filtered_pitch is None
            or abs(math.degrees(self.filtered_pitch)) > self.max_pitch_deg
        )
        if pitch_outside_limit:
            return False, 'pitch safety limit exceeded'
        if require_backend:
            return self._backend_ready()
        return True, ''

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self.mode != 'active':
                response.success = False
                response.message = 'enable is only available in active mode'
                return response
            ready, reason = self._prerequisites(self._now(), True)
            if not ready or not self._capture_nominal():
                response.success = False
                response.message = reason or 'nominal stance is invalid'
                return response
            self.fault_latched = ''
            self.integral = 0.0
            self.correction_angle = 0.0
            self.enabled = True
            self.disabled_ramp_pending = False
            response.success = True
            response.message = 'active correction enabled with zero ramp-in'
            self.get_logger().info(response.message)
        else:
            self.enabled = False
            self.enable_pending = False
            self.disabled_ramp_pending = abs(self.correction_angle) > 1.0e-8
            self.integral = 0.0
            response.success = True
            response.message = 'disabled; correction is ramping to neutral'
            self.get_logger().info(response.message)
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.integral = 0.0
        self.filtered_pitch = self.raw_pitch
        self.last_control_time = None
        self.saturation_start = None
        self.ik_failures = 0
        self.fault_latched = ''
        # Preserve an already captured neutral stance. Re-basing while a
        # correction is held would turn that corrected pose into the new zero.
        response.success = (
            True if self.nominal_feet
            else self._capture_nominal() if self.latest_joints else True
        )
        response.message = (
            'controller state reset'
            if response.success else 'current stance is invalid'
        )
        return response

    def _on_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name not in TUNABLE_PARAMETERS:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason='tunable values must be numeric'
                )
            if not math.isfinite(value):
                return SetParametersResult(
                    successful=False, reason='tunable values must be finite'
                )
            if parameter.name in ('kp', 'ki', 'kd') and value < 0.0:
                return SetParametersResult(successful=False, reason='PID gains cannot be negative')
            limits = {
                'pitch_filter_cutoff_hz': (0.1, 25.0),
                'max_body_correction_deg': (0.1, 20.0),
                'max_leg_offset_m': (0.001, 0.06),
                'max_offset_rate_mps': (0.001, 0.10),
                'target_pitch_deg': (-20.0, 20.0),
            }
            if parameter.name in limits:
                lower, upper = limits[parameter.name]
                if not lower <= value <= upper:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} is outside its safe range',
                    )
        for parameter in parameters:
            if parameter.name in TUNABLE_PARAMETERS:
                setattr(self, parameter.name, float(parameter.value))
        return SetParametersResult(successful=True)

    @staticmethod
    def _rate_limit(current: float, desired: float, maximum_change: float) -> float:
        return current + max(-maximum_change, min(maximum_change, desired - current))

    def _fault(self, reason: str) -> None:
        if not self.fault_latched:
            self.get_logger().error(f'active correction fault: {reason}')
        self.fault_latched = reason
        self.enabled = False
        self.enable_pending = False
        self.disabled_ramp_pending = True
        self.integral = 0.0

    def _solve_targets(self, front: float, rear: float) -> list[float]:
        positions: list[float] = []
        proposed = JointState()
        proposed.header.stamp = self.get_clock().now().to_msg()
        for leg in LEGS:
            target = list(self.nominal_feet[leg])
            target[2] += front if leg.startswith('F') else rear
            solution = self.ik[leg].solve(target, self.previous_solutions.get(leg))
            self.previous_solutions[leg] = solution
            proposed.name.extend(f'{leg}_{joint}_joint' for joint in ('hip', 'thigh', 'calf'))
            proposed.position.extend(solution.as_tuple())
            positions.extend(solution.as_tuple())
        self.target_publisher.publish(proposed)
        return positions

    def _control(self) -> None:
        now = self._now()
        dt = (
            1.0 / self.control_rate_hz
            if self.last_control_time is None else now - self.last_control_time
        )
        self.last_control_time = now
        if dt <= 0.0 or dt > self.max_message_dt_sec:
            return
        require_backend = self.mode == 'active' and (
            self.enabled or self.enable_pending
        )
        ready, reason = self._prerequisites(now, require_backend)
        if not ready:
            self.last_state = reason
            if self.enabled:
                self._fault(reason)
            self._publish_state_only()
            return
        if self.enable_pending and self.mode == 'active':
            if self._capture_nominal():
                self.enabled = True
                self.enable_pending = False
            else:
                self._fault('nominal stance is invalid')
        if self.mode == 'monitor':
            self.last_state = 'monitor'
            error = math.radians(self.target_pitch_deg) - self.filtered_pitch
            self._publish_monitoring(error, 0.0, 0.0, 0.0, 0.0, 0.0)
            return
        if not self.nominal_feet and not self._capture_nominal():
            self.last_state = 'waiting for valid nominal stance'
            self._publish_state_only()
            return

        target = math.radians(self.target_pitch_deg)
        error = target - self.filtered_pitch
        p_term = self.kp * error
        d_term = -self.kd * self.pitch_rate
        unsaturated_without_i = p_term + self.ki * self.integral + d_term
        output_limit = math.radians(self.max_body_correction_deg)
        desired = max(-output_limit, min(output_limit, unsaturated_without_i))
        self.saturated = abs(unsaturated_without_i) > output_limit
        if self.enabled and (not self.saturated or error * unsaturated_without_i < 0.0):
            self.integral = max(
                -self.integral_limit,
                min(self.integral_limit, self.integral + error * dt),
            )
            updated_output = p_term + self.ki * self.integral + d_term
            desired = max(-output_limit, min(output_limit, updated_output))
        if not self.enabled and self.mode == 'active':
            desired = 0.0
        hip_x = abs(self.geometries['FL'].hip_x)
        max_angle_rate = self.max_offset_rate_mps / max(hip_x, 1.0e-6)
        self.correction_angle = self._rate_limit(
            self.correction_angle, desired, max_angle_rate * dt
        )
        front = max(-self.max_leg_offset_m, min(
            self.max_leg_offset_m,
            self.geometries['FL'].hip_x * math.tan(self.correction_angle),
        ))
        rear = max(-self.max_leg_offset_m, min(
            self.max_leg_offset_m,
            self.geometries['RL'].hip_x * math.tan(self.correction_angle),
        ))
        try:
            positions = self._solve_targets(front, rear)
            self.ik_valid = True
            self.ik_failures = 0
        except (IKError, ValueError, KeyError) as error_message:
            self.ik_valid = False
            self.ik_failures += 1
            self.last_state = f'IK unreachable: {error_message}'
            if self.ik_failures >= self.max_consecutive_ik_failures and self.enabled:
                self._fault(self.last_state)
            self._publish_monitoring(error, p_term, self.ki * self.integral, d_term, front, rear)
            return

        if self.saturated:
            self.saturation_start = self.saturation_start or now
            if self.enabled and now - self.saturation_start > self.max_saturation_sec:
                self._fault('output saturation timeout')
        else:
            self.saturation_start = None
        should_command = self.mode == 'active' and (self.enabled or self.disabled_ramp_pending)
        if should_command and not self.fault_latched:
            message = Float64MultiArray()
            message.data = positions
            self.command_publisher.publish(message)
        if self.disabled_ramp_pending and abs(self.correction_angle) <= 1.0e-8:
            self.disabled_ramp_pending = False
        enabled_suffix = ' enabled' if self.enabled else ' disabled'
        self.last_state = self.fault_latched or self.mode + enabled_suffix
        self._publish_monitoring(error, p_term, self.ki * self.integral, d_term, front, rear)

    def _publish_monitoring(self, error: float, p_term: float, i_term: float,
                            d_term: float, front: float, rear: float) -> None:
        output = self.correction_angle
        values = {
            'raw': math.degrees(self.raw_pitch),
            'filtered': math.degrees(self.filtered_pitch),
            'target': self.target_pitch_deg,
            'error': math.degrees(error),
            'rate': self.pitch_rate,
            'p': p_term,
            'i': i_term,
            'd': d_term,
            'output': math.degrees(output),
            'front': front,
            'rear': rear,
        }
        if all(math.isfinite(value) for value in values.values()):
            for key, value in values.items():
                self.scalar_publishers[key].publish(Float64(data=value))
        self.last_terms = (p_term, i_term, d_term, output)
        self.last_offsets = (front, rear)
        self._publish_state_only()

    def _publish_state_only(self) -> None:
        self.enabled_publisher.publish(Bool(data=self.enabled))
        self.saturation_publisher.publish(Bool(data=self.saturated))
        self.ik_valid_publisher.publish(Bool(data=self.ik_valid))

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = 'go2w_imu_ik_controller'
        status.hardware_id = 'go2w'
        status.level = DiagnosticStatus.ERROR if self.fault_latched else (
            DiagnosticStatus.WARN if self.last_state.startswith(('waiting', 'stale', 'invalid'))
            else DiagnosticStatus.OK
        )
        status.message = self.last_state
        status.values = [
            KeyValue(key='mode', value=self.mode),
            KeyValue(key='enabled', value=str(self.enabled).lower()),
            KeyValue(key='backend', value=self.command_backend),
            KeyValue(key='ik_valid', value=str(self.ik_valid).lower()),
            KeyValue(key='saturated', value=str(self.saturated).lower()),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def neutralize_for_shutdown(self) -> None:
        can_neutralize = (
            self.mode == 'active'
            and self.nominal_feet
            and self.command_publisher.get_subscription_count()
        )
        if can_neutralize:
            try:
                positions = self._solve_targets(0.0, 0.0)
                message = Float64MultiArray()
                message.data = positions
                self.command_publisher.publish(message)
            except (IKError, ValueError, KeyError):
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2WImuIkController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.neutralize_for_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
