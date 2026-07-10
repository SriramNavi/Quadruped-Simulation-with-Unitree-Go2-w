"""Safe terminal keyboard teleoperation publishing Go2-W body velocities."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from typing import TextIO

from geometry_msgs.msg import Twist
from quadruped_control.go2w_cmd_vel_to_wheels import (
    calculate_wheel_target,
    DEFAULT_CMD_VEL_TOPIC,
    DEFAULT_EMERGENCY_STOP_TOPIC,
    DriveLimits,
    shape_body_command,
)
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


KEY_UP = 'up'
KEY_DOWN = 'down'
KEY_LEFT = 'left'
KEY_RIGHT = 'right'
KEY_STOP = 'stop'
KEY_ESTOP = 'estop'
KEY_RESUME = 'resume'
KEY_RECENTER = 'recenter'
KEY_HELP = 'help'
KEY_QUIT = 'quit'

KEY_SEQUENCES = {
    '\x1b[A': KEY_UP,
    '\x1bOA': KEY_UP,
    '\x1b[B': KEY_DOWN,
    '\x1bOB': KEY_DOWN,
    '\x1b[D': KEY_LEFT,
    '\x1bOD': KEY_LEFT,
    '\x1b[C': KEY_RIGHT,
    '\x1bOC': KEY_RIGHT,
}
KEY_SEQUENCE_BYTES = sorted(
    (sequence.encode('utf-8') for sequence in KEY_SEQUENCES),
    key=len,
    reverse=True,
)


class TerminalKeyReader:
    """Read single keypresses from a Linux terminal without requiring Enter."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._settings: list[object] | None = None
        self._tty_file: TextIO | None = None
        self._buffer = b''

    def __enter__(self) -> 'TerminalKeyReader':
        self._fd = sys.stdin.fileno()
        if not os.isatty(self._fd):
            try:
                self._tty_file = open('/dev/tty', 'r', encoding='utf-8')
            except OSError as exc:
                raise RuntimeError('keyboard teleop requires a terminal') from exc
            self._fd = self._tty_file.fileno()

        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._fd is not None and self._settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
        if self._tty_file is not None:
            self._tty_file.close()

    def read_key(self, timeout_sec: float) -> str | None:
        """Read one key or return None when the timeout expires."""
        if self._fd is None:
            raise RuntimeError('terminal reader is not active')

        if not self._buffer:
            readable, _, _ = select.select([self._fd], [], [], timeout_sec)
            if not readable:
                return None
            self._buffer = os.read(self._fd, 1)

        if self._buffer.startswith(b'\x1b'):
            self._read_available()
            data = self._pop_escape_sequence()
        else:
            data = self._buffer[:1]
            self._buffer = self._buffer[1:]
        return data.decode('utf-8', errors='ignore')

    def _read_available(self) -> None:
        if self._fd is None:
            return
        while True:
            readable, _, _ = select.select([self._fd], [], [], 0.02)
            if not readable:
                break
            self._buffer += os.read(self._fd, 64)

    def _pop_escape_sequence(self) -> bytes:
        for sequence in KEY_SEQUENCE_BYTES:
            if self._buffer.startswith(sequence):
                self._buffer = self._buffer[len(sequence):]
                return sequence

        end = 1
        while end < len(self._buffer):
            byte = self._buffer[end]
            end += 1
            if end == 2 and byte in (ord('['), ord('O')):
                continue
            if 0x40 <= byte <= 0x7e:
                break

        sequence = self._buffer[:end]
        self._buffer = self._buffer[end:]
        return sequence


class Go2WKeyboardCmdVel(Node):
    """Maintain safe target body velocities and publish them as cmd_vel."""

    def __init__(self) -> None:
        super().__init__('go2w_keyboard_cmd_vel')
        self.declare_parameter('cmd_vel_topic', DEFAULT_CMD_VEL_TOPIC)
        self.declare_parameter('emergency_stop_topic', DEFAULT_EMERGENCY_STOP_TOPIC)
        self.declare_parameter('linear_step_mps', 0.10)
        self.declare_parameter('angular_step_radps', 0.10)
        self.declare_parameter('minimum_linear_step_mps', 0.05)
        self.declare_parameter('maximum_linear_step_mps', 0.50)
        self.declare_parameter('minimum_angular_step_radps', 0.05)
        self.declare_parameter('maximum_angular_step_radps', 0.50)
        self.declare_parameter('increment_adjustment', 0.05)
        self.declare_parameter('max_linear_velocity_mps', 1.5)
        self.declare_parameter('max_reverse_velocity_mps', 1.0)
        self.declare_parameter('max_keyboard_angular_velocity_radps', 1.5)
        self.declare_parameter('wheel_radius_m', 0.086)
        self.declare_parameter('track_width_m', 0.3802)
        self.declare_parameter('max_wheel_angular_velocity_radps', 10.0)
        self.declare_parameter('minimum_turning_radius_m', 0.75)
        self.declare_parameter('max_angular_velocity_at_zero_speed_radps', 1.0)
        self.declare_parameter('max_angular_velocity_at_max_speed_radps', 0.35)
        self.declare_parameter('allow_pivot_turn', True)
        self.declare_parameter('pivot_turn_max_radps', 0.5)
        self.declare_parameter('pivot_speed_threshold_mps', 0.10)
        self.declare_parameter('deadband', 0.01)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.emergency_stop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self.linear_step = self._positive_parameter('linear_step_mps')
        self.angular_step = self._positive_parameter('angular_step_radps')
        self.minimum_linear_step = self._positive_parameter('minimum_linear_step_mps')
        self.maximum_linear_step = self._positive_parameter('maximum_linear_step_mps')
        self.minimum_angular_step = self._positive_parameter('minimum_angular_step_radps')
        self.maximum_angular_step = self._positive_parameter('maximum_angular_step_radps')
        self.increment_adjustment = self._positive_parameter('increment_adjustment')
        publish_rate_hz = self._positive_parameter('publish_rate_hz')
        self.publish_period_sec = 1.0 / publish_rate_hz
        self.limits = DriveLimits(
            wheel_radius_m=self._positive_parameter('wheel_radius_m'),
            track_width_m=self._positive_parameter('track_width_m'),
            max_linear_velocity_mps=self._positive_parameter('max_linear_velocity_mps'),
            max_reverse_velocity_mps=self._positive_parameter('max_reverse_velocity_mps'),
            max_angular_velocity_radps=self._positive_parameter(
                'max_keyboard_angular_velocity_radps'
            ),
            max_wheel_angular_velocity_radps=self._positive_parameter(
                'max_wheel_angular_velocity_radps'
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

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.emergency_stop_active = False
        self._status_lines = 0
        self._cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._estop_publisher = self.create_publisher(
            Bool,
            self.emergency_stop_topic,
            10,
        )

    def print_help(self) -> None:
        """Print the keyboard control reference."""
        self._clear_status()
        print(
            f"""
Go2-W safe velocity teleop

Movement: W/Up forward, S/Down reverse, A/Left left, D/Right right
Increments: E/Q linear up/down, Shift+E/Shift+Q steering up/down
Steering: C recenter
Stop: X controlled stop, Space emergency stop, R clear emergency stop
Utility: H help, Z emergency stop and quit

Publishes: {self.cmd_vel_topic}
Emergency stop: {self.emergency_stop_topic}
""",
            flush=True,
        )

    def handle_key(self, sequence: str) -> bool:
        """Apply one keypress, returning False only when quit is requested."""
        key = self._normalize_key(sequence)
        if key is None:
            return True

        movement_key = key in (KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_RECENTER)
        if self.emergency_stop_active and movement_key:
            self.display_status()
            return True

        if key == KEY_UP:
            self.linear_velocity += self.linear_step
        elif key == KEY_DOWN:
            self.linear_velocity -= self.linear_step
        elif key == KEY_LEFT:
            self.angular_velocity += self.angular_step
        elif key == KEY_RIGHT:
            self.angular_velocity -= self.angular_step
        elif key == 'linear_step_up':
            self.linear_step = min(
                self.maximum_linear_step,
                self.linear_step + self.increment_adjustment,
            )
        elif key == 'linear_step_down':
            self.linear_step = max(
                self.minimum_linear_step,
                self.linear_step - self.increment_adjustment,
            )
        elif key == 'angular_step_up':
            self.angular_step = min(
                self.maximum_angular_step,
                self.angular_step + self.increment_adjustment,
            )
        elif key == 'angular_step_down':
            self.angular_step = max(
                self.minimum_angular_step,
                self.angular_step - self.increment_adjustment,
            )
        elif key == KEY_RECENTER:
            self.angular_velocity = 0.0
        elif key == KEY_STOP:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        elif key == KEY_ESTOP:
            self.emergency_stop()
            self.display_status()
            return True
        elif key == KEY_RESUME:
            self.clear_emergency_stop()
            self.display_status()
            return True
        elif key == KEY_HELP:
            self.print_help()
        elif key == KEY_QUIT:
            self.emergency_stop()
            self.display_status()
            return False

        self._shape_targets()
        self.publish_cmd_vel()
        self.display_status()
        return True

    def publish_cmd_vel(self) -> None:
        """Publish the current body velocity target in physical SI units."""
        msg = Twist()
        msg.linear.x = float(self.linear_velocity)
        msg.angular.z = float(self.angular_velocity)
        self._cmd_publisher.publish(msg)

    def emergency_stop(self) -> None:
        """Publish emergency stop true and an immediate zero body command."""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.emergency_stop_active = True
        self._publish_emergency_stop(True)
        self.publish_cmd_vel()

    def clear_emergency_stop(self) -> None:
        """Deliberately clear emergency stop while leaving all targets at zero."""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.emergency_stop_active = False
        self._publish_emergency_stop(False)
        self.publish_cmd_vel()

    def display_status(self) -> None:
        """Refresh a compact status block after a user-visible state change."""
        left, right = calculate_wheel_target(
            self.linear_velocity,
            self.angular_velocity,
            self.limits,
        )
        turn_radius = self._estimated_turn_radius()
        estop_state = 'ACTIVE' if self.emergency_stop_active else 'clear'
        lines = [
            f'Target linear velocity:  {self.linear_velocity:6.2f} m/s',
            f'Target angular velocity: {self.angular_velocity:6.2f} rad/s',
            f'Linear increment:         {self.linear_step:6.2f} m/s',
            f'Angular increment:        {self.angular_step:6.2f} rad/s',
            f'Estimated turn radius:    {turn_radius}',
            f'Requested motion:         {self._requested_motion()}',
            f'Saturated wheel target:   [{left:.2f}, {right:.2f}, '
            f'{left:.2f}, {right:.2f}] rad/s',
            f'Emergency stop:           {estop_state}',
        ]
        self._clear_status()
        print('\n'.join(lines), flush=True)
        self._status_lines = len(lines)

    def _shape_targets(self) -> None:
        self.linear_velocity, self.angular_velocity = shape_body_command(
            self.linear_velocity,
            self.angular_velocity,
            self.limits,
        )
        self.linear_step = round(self.linear_step, 2)
        self.angular_step = round(self.angular_step, 2)

    def _publish_emergency_stop(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._estop_publisher.publish(msg)

    def _estimated_turn_radius(self) -> str:
        if abs(self.angular_velocity) <= self.limits.deadband:
            return 'INF m'
        if abs(self.linear_velocity) <= self.limits.pivot_speed_threshold_mps:
            return 'PIVOT'
        return f'{abs(self.linear_velocity / self.angular_velocity):.2f} m'

    def _requested_motion(self) -> str:
        if self.emergency_stop_active:
            return 'STOPPED'
        linear_is_zero = abs(self.linear_velocity) <= self.limits.deadband
        angular_is_zero = abs(self.angular_velocity) <= self.limits.deadband
        if linear_is_zero and angular_is_zero:
            return 'STOPPED'
        if linear_is_zero:
            return 'PIVOT'
        if not angular_is_zero:
            return 'LEFT ARC' if self.angular_velocity > 0.0 else 'RIGHT ARC'
        return 'FORWARD' if self.linear_velocity > 0.0 else 'REVERSE'

    def _clear_status(self) -> None:
        if self._status_lines and sys.stdout.isatty():
            print(f'\033[{self._status_lines}A\r\033[J', end='', flush=True)
        self._status_lines = 0

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

    @staticmethod
    def _normalize_key(sequence: str) -> str | None:
        if sequence == '\x03':
            raise KeyboardInterrupt
        if sequence in KEY_SEQUENCES:
            return KEY_SEQUENCES[sequence]
        if sequence == '\x1b' or sequence == ' ':
            return KEY_ESTOP
        if len(sequence) != 1:
            return None
        if sequence == 'E':
            return 'angular_step_up'
        if sequence == 'Q':
            return 'angular_step_down'
        key_map = {
            'w': KEY_UP,
            's': KEY_DOWN,
            'a': KEY_LEFT,
            'd': KEY_RIGHT,
            'e': 'linear_step_up',
            'q': 'linear_step_down',
            'c': KEY_RECENTER,
            'x': KEY_STOP,
            'r': KEY_RESUME,
            'h': KEY_HELP,
            'z': KEY_QUIT,
        }
        return key_map.get(sequence.lower())


def main(args=None) -> None:
    """Run the terminal keyboard teleop."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2WKeyboardCmdVel()

    try:
        with TerminalKeyReader() as reader:
            node.print_help()
            node.publish_cmd_vel()
            node.display_status()
            next_publish_time = time.monotonic() + node.publish_period_sec
            while rclpy.ok():
                now = time.monotonic()
                timeout = max(0.0, min(node.publish_period_sec, next_publish_time - now))
                key = reader.read_key(timeout)
                if key is not None and not node.handle_key(key):
                    break

                if time.monotonic() >= next_publish_time:
                    node.publish_cmd_vel()
                    next_publish_time = time.monotonic() + node.publish_period_sec
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        print('\nCtrl+C received.', flush=True)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        if rclpy.ok():
            node.emergency_stop()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
