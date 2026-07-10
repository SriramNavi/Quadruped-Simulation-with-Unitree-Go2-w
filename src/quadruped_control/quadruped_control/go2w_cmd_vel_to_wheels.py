"""Safely convert Go2-W body velocity commands into wheel velocities."""

from __future__ import annotations

from dataclasses import dataclass
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray


DEFAULT_CMD_VEL_TOPIC = '/cmd_vel'
DEFAULT_WHEEL_COMMAND_TOPIC = '/go2w_wheel_velocity_controller/commands'
DEFAULT_EMERGENCY_STOP_TOPIC = '/go2w/emergency_stop'
ZERO_COMMAND = [0.0, 0.0, 0.0, 0.0]


@dataclass(frozen=True)
class DriveLimits:
    """Physical geometry and configurable body/wheel safety limits."""

    wheel_radius_m: float = 0.086
    track_width_m: float = 0.3802
    max_linear_velocity_mps: float = 1.5
    max_reverse_velocity_mps: float = 1.0
    max_angular_velocity_radps: float = 1.5
    max_wheel_angular_velocity_radps: float = 10.0
    max_linear_acceleration_mps2: float = 0.8
    max_linear_deceleration_mps2: float = 1.2
    max_angular_acceleration_radps2: float = 1.0
    max_wheel_acceleration_radps2: float = 8.0
    minimum_turning_radius_m: float = 0.75
    max_angular_velocity_at_zero_speed_radps: float = 1.0
    max_angular_velocity_at_max_speed_radps: float = 0.35
    allow_pivot_turn: bool = True
    pivot_turn_max_radps: float = 0.5
    pivot_speed_threshold_mps: float = 0.10
    deadband: float = 0.01


def apply_deadband(value: float, deadband: float) -> float:
    """Return zero when a value is inside the symmetric deadband."""
    return 0.0 if abs(value) < abs(deadband) else value


def clamp(value: float, limit: float) -> float:
    """Clamp a value to a symmetric limit."""
    limit = abs(limit)
    return max(-limit, min(limit, value))


def clamp_linear_velocity(
    value: float,
    max_forward_velocity: float,
    max_reverse_velocity: float,
) -> float:
    """Clamp forward and reverse velocity using their separate limits."""
    return max(-abs(max_reverse_velocity), min(abs(max_forward_velocity), value))


def rate_limit(current: float, target: float, maximum_rate: float, elapsed_sec: float) -> float:
    """Move current toward target by no more than rate multiplied by elapsed time."""
    maximum_change = abs(maximum_rate) * max(0.0, elapsed_sec)
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + maximum_change * (1.0 if difference > 0.0 else -1.0)


def dynamic_angular_velocity_limit(linear_velocity: float, limits: DriveLimits) -> float:
    """Compute the speed-dependent yaw-rate limit before the radius constraint."""
    if limits.max_linear_velocity_mps <= 0.0:
        speed_ratio = 1.0
    else:
        speed_ratio = min(
            1.0,
            abs(linear_velocity) / limits.max_linear_velocity_mps,
        )
    low_speed_limit = abs(limits.max_angular_velocity_at_zero_speed_radps)
    high_speed_limit = abs(limits.max_angular_velocity_at_max_speed_radps)
    return low_speed_limit + speed_ratio * (high_speed_limit - low_speed_limit)


def limit_angular_velocity(
    linear_velocity: float,
    angular_velocity: float,
    limits: DriveLimits,
) -> float:
    """Apply global, pivot, speed-dependent, and turning-radius yaw limits."""
    requested = clamp(angular_velocity, limits.max_angular_velocity_radps)
    speed = abs(linear_velocity)

    if speed <= limits.pivot_speed_threshold_mps:
        if not limits.allow_pivot_turn:
            return 0.0
        allowed = min(
            abs(limits.max_angular_velocity_radps),
            abs(limits.pivot_turn_max_radps),
        )
        return clamp(requested, allowed)

    allowed = min(
        abs(limits.max_angular_velocity_radps),
        max(0.0, dynamic_angular_velocity_limit(linear_velocity, limits)),
    )
    if limits.minimum_turning_radius_m > 0.0:
        allowed = min(allowed, speed / limits.minimum_turning_radius_m)
    return clamp(requested, allowed)


def shape_body_command(
    linear_velocity: float,
    angular_velocity: float,
    limits: DriveLimits,
) -> tuple[float, float]:
    """Apply body-command deadbands and all configured body velocity limits."""
    linear = apply_deadband(float(linear_velocity), limits.deadband)
    angular = apply_deadband(float(angular_velocity), limits.deadband)
    linear = clamp_linear_velocity(
        linear,
        limits.max_linear_velocity_mps,
        limits.max_reverse_velocity_mps,
    )
    angular = limit_angular_velocity(linear, angular, limits)
    return linear, angular


def differential_drive_wheel_speeds(
    linear_velocity: float,
    angular_velocity: float,
    wheel_radius_m: float,
    track_width_m: float,
) -> tuple[float, float]:
    """Convert body velocity to left/right wheel angular velocity in rad/s."""
    if wheel_radius_m <= 0.0:
        raise ValueError('wheel_radius_m must be greater than zero')
    half_track = track_width_m / 2.0
    left_linear = linear_velocity - angular_velocity * half_track
    right_linear = linear_velocity + angular_velocity * half_track
    return left_linear / wheel_radius_m, right_linear / wheel_radius_m


def curvature_preserving_saturation(
    left: float,
    right: float,
    maximum_wheel_speed: float,
) -> tuple[float, float]:
    """Scale both wheel speeds together so their ratio and curvature are preserved."""
    maximum = abs(maximum_wheel_speed)
    max_abs = max(abs(left), abs(right))
    if max_abs == 0.0 or max_abs <= maximum:
        return left, right
    if maximum == 0.0:
        return 0.0, 0.0
    scale = maximum / max_abs
    return left * scale, right * scale


def calculate_wheel_target(
    linear_velocity: float,
    angular_velocity: float,
    limits: DriveLimits,
) -> tuple[float, float]:
    """Shape a body command and return its saturated left/right wheel target."""
    linear, angular = shape_body_command(linear_velocity, angular_velocity, limits)
    left, right = differential_drive_wheel_speeds(
        linear,
        angular,
        limits.wheel_radius_m,
        limits.track_width_m,
    )
    return curvature_preserving_saturation(
        left,
        right,
        limits.max_wheel_angular_velocity_radps,
    )


class DriveCommandState:
    """Testable state machine implementing command shaping and rate limits."""

    def __init__(self, limits: DriveLimits) -> None:
        self.limits = limits
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0
        self.current_left_wheel_command = 0.0
        self.current_right_wheel_command = 0.0
        self.emergency_stop_active = False

    def set_target(self, linear_velocity: float, angular_velocity: float) -> None:
        """Set a safely shaped target body velocity unless emergency stop is active."""
        if self.emergency_stop_active:
            return
        linear, angular = shape_body_command(linear_velocity, angular_velocity, self.limits)
        self.target_linear_velocity = linear
        self.target_angular_velocity = angular

    def set_emergency_stop(self, active: bool) -> None:
        """Latch or clear emergency stop, resetting all motion state either way."""
        self.emergency_stop_active = bool(active)
        self.reset_motion()

    def reset_motion(self) -> None:
        """Reset body and wheel commands to exactly zero."""
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0
        self.current_left_wheel_command = 0.0
        self.current_right_wheel_command = 0.0

    def update(self, elapsed_sec: float, command_stale: bool = False) -> tuple[float, float]:
        """Advance the controller by the actual elapsed time and return wheel commands."""
        if self.emergency_stop_active:
            self.reset_motion()
            return 0.0, 0.0

        if command_stale:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

        linear_rate = self._linear_rate_limit()
        self.commanded_linear_velocity = rate_limit(
            self.commanded_linear_velocity,
            self.target_linear_velocity,
            linear_rate,
            elapsed_sec,
        )
        self.commanded_angular_velocity = rate_limit(
            self.commanded_angular_velocity,
            self.target_angular_velocity,
            self.limits.max_angular_acceleration_radps2,
            elapsed_sec,
        )
        self.commanded_angular_velocity = limit_angular_velocity(
            self.commanded_linear_velocity,
            self.commanded_angular_velocity,
            self.limits,
        )

        desired_left, desired_right = differential_drive_wheel_speeds(
            self.commanded_linear_velocity,
            self.commanded_angular_velocity,
            self.limits.wheel_radius_m,
            self.limits.track_width_m,
        )
        desired_left, desired_right = curvature_preserving_saturation(
            desired_left,
            desired_right,
            self.limits.max_wheel_angular_velocity_radps,
        )
        self.current_left_wheel_command = rate_limit(
            self.current_left_wheel_command,
            desired_left,
            self.limits.max_wheel_acceleration_radps2,
            elapsed_sec,
        )
        self.current_right_wheel_command = rate_limit(
            self.current_right_wheel_command,
            desired_right,
            self.limits.max_wheel_acceleration_radps2,
            elapsed_sec,
        )
        return self.current_left_wheel_command, self.current_right_wheel_command

    def _linear_rate_limit(self) -> float:
        current = self.commanded_linear_velocity
        target = self.target_linear_velocity
        slowing_or_reversing = (
            current != 0.0
            and (current * target <= 0.0 or abs(target) < abs(current))
        )
        if slowing_or_reversing:
            return self.limits.max_linear_deceleration_mps2
        return self.limits.max_linear_acceleration_mps2


class Go2WCmdVelToWheels(Node):
    """Bridge geometry_msgs/Twist into the Go2-W wheel velocity controller."""

    def __init__(self) -> None:
        super().__init__('go2w_cmd_vel_to_wheels')

        self.declare_parameter('wheel_command_topic', DEFAULT_WHEEL_COMMAND_TOPIC)
        self.declare_parameter('cmd_vel_topic', DEFAULT_CMD_VEL_TOPIC)
        self.declare_parameter('emergency_stop_topic', DEFAULT_EMERGENCY_STOP_TOPIC)
        self.declare_parameter('wheel_radius_m', 0.086)
        self.declare_parameter('track_width_m', 0.3802)
        self.declare_parameter('max_linear_velocity_mps', 1.5)
        self.declare_parameter('max_reverse_velocity_mps', 1.0)
        self.declare_parameter('max_angular_velocity_radps', 1.5)
        self.declare_parameter('max_wheel_angular_velocity_radps', 10.0)
        self.declare_parameter('max_linear_acceleration_mps2', 0.8)
        self.declare_parameter('max_linear_deceleration_mps2', 1.2)
        self.declare_parameter('max_angular_acceleration_radps2', 1.0)
        self.declare_parameter('max_wheel_acceleration_radps2', 8.0)
        self.declare_parameter('minimum_turning_radius_m', 0.75)
        self.declare_parameter('max_angular_velocity_at_zero_speed_radps', 1.0)
        self.declare_parameter('max_angular_velocity_at_max_speed_radps', 0.35)
        self.declare_parameter('allow_pivot_turn', True)
        self.declare_parameter('pivot_turn_max_radps', 0.5)
        self.declare_parameter('pivot_speed_threshold_mps', 0.10)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('deadband', 0.01)

        self.wheel_command_topic = self._string_parameter('wheel_command_topic')
        self.cmd_vel_topic = self._string_parameter('cmd_vel_topic')
        self.emergency_stop_topic = self._string_parameter('emergency_stop_topic')
        self.command_timeout_sec = self._nonnegative_parameter('command_timeout_sec')
        self.control_rate_hz = self._positive_parameter('control_rate_hz')
        self.limits = DriveLimits(
            wheel_radius_m=self._positive_parameter('wheel_radius_m'),
            track_width_m=self._positive_parameter('track_width_m'),
            max_linear_velocity_mps=self._positive_parameter('max_linear_velocity_mps'),
            max_reverse_velocity_mps=self._positive_parameter('max_reverse_velocity_mps'),
            max_angular_velocity_radps=self._positive_parameter(
                'max_angular_velocity_radps'
            ),
            max_wheel_angular_velocity_radps=self._positive_parameter(
                'max_wheel_angular_velocity_radps'
            ),
            max_linear_acceleration_mps2=self._positive_parameter(
                'max_linear_acceleration_mps2'
            ),
            max_linear_deceleration_mps2=self._positive_parameter(
                'max_linear_deceleration_mps2'
            ),
            max_angular_acceleration_radps2=self._positive_parameter(
                'max_angular_acceleration_radps2'
            ),
            max_wheel_acceleration_radps2=self._positive_parameter(
                'max_wheel_acceleration_radps2'
            ),
            minimum_turning_radius_m=self._positive_parameter(
                'minimum_turning_radius_m'
            ),
            max_angular_velocity_at_zero_speed_radps=self._nonnegative_parameter(
                'max_angular_velocity_at_zero_speed_radps'
            ),
            max_angular_velocity_at_max_speed_radps=self._nonnegative_parameter(
                'max_angular_velocity_at_max_speed_radps'
            ),
            allow_pivot_turn=bool(self.get_parameter('allow_pivot_turn').value),
            pivot_turn_max_radps=self._nonnegative_parameter('pivot_turn_max_radps'),
            pivot_speed_threshold_mps=self._nonnegative_parameter(
                'pivot_speed_threshold_mps'
            ),
            deadband=self._nonnegative_parameter('deadband'),
        )
        self._state = DriveCommandState(self.limits)
        self._last_cmd_time = self.get_clock().now()
        self._last_control_time = self._last_cmd_time
        self._have_cmd_vel = False

        self._publisher = self.create_publisher(
            Float64MultiArray,
            self.wheel_command_topic,
            10,
        )
        self._cmd_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self._estop_subscription = self.create_subscription(
            Bool,
            self.emergency_stop_topic,
            self._emergency_stop_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / self.control_rate_hz, self._control_cycle)

        self.get_logger().info(
            f'Bridging {self.cmd_vel_topic} to {self.wheel_command_topic} at '
            f'{self.control_rate_hz:.1f} Hz; wheel radius={self.limits.wheel_radius_m:.4f} m, '
            f'track={self.limits.track_width_m:.4f} m'
        )

    @property
    def target_linear_velocity(self) -> float:
        """Expose current target linear velocity for runtime inspection."""
        return self._state.target_linear_velocity

    @property
    def target_angular_velocity(self) -> float:
        """Expose current target angular velocity for runtime inspection."""
        return self._state.target_angular_velocity

    @property
    def commanded_linear_velocity(self) -> float:
        """Expose rate-limited linear velocity for runtime inspection."""
        return self._state.commanded_linear_velocity

    @property
    def commanded_angular_velocity(self) -> float:
        """Expose rate-limited angular velocity for runtime inspection."""
        return self._state.commanded_angular_velocity

    def _cmd_vel_callback(self, msg: Twist) -> None:
        if self._state.emergency_stop_active:
            return
        self._state.set_target(msg.linear.x, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()
        self._have_cmd_vel = True

    def _emergency_stop_callback(self, msg: Bool) -> None:
        self._state.set_emergency_stop(msg.data)
        self._have_cmd_vel = False
        self._last_cmd_time = self.get_clock().now()
        self._publish_wheels(ZERO_COMMAND)
        if msg.data:
            self.get_logger().warn('Emergency stop active; ignoring cmd_vel')
        else:
            self.get_logger().info('Emergency stop cleared; motion targets reset')

    def _control_cycle(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = max(
            0.0,
            (now.nanoseconds - self._last_control_time.nanoseconds) * 1.0e-9,
        )
        self._last_control_time = now

        if self._state.emergency_stop_active:
            self._state.reset_motion()
            self._publish_wheels(ZERO_COMMAND)
            return

        left, right = self._state.update(
            elapsed_sec,
            command_stale=self._command_is_stale(now),
        )
        self._publish_wheels([left, right, left, right])

    def _command_is_stale(self, now) -> bool:
        if not self._have_cmd_vel:
            return True
        age_sec = max(0.0, (now.nanoseconds - self._last_cmd_time.nanoseconds) * 1.0e-9)
        return age_sec > self.command_timeout_sec

    def publish_zero(self) -> None:
        """Immediately reset and publish zero wheel velocities."""
        self._state.reset_motion()
        self._publish_wheels(ZERO_COMMAND)

    def _publish_wheels(self, values: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        self._publisher.publish(msg)

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value


def main(args=None) -> None:
    """Run the Go2-W cmd_vel-to-wheel controller."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2WCmdVelToWheels()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
