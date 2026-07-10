"""Terminal keyboard teleoperation for the Unitree Go2-W wheel controller."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from typing import TextIO

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray


WHEEL_COMMAND_TOPIC = '/go2w_wheel_velocity_controller/commands'

DEFAULT_LINEAR_SPEED = 1.0
MAX_LINEAR_SPEED = 10.0
MIN_LINEAR_SPEED = 0.5
LINEAR_SPEED_INCREMENT = 0.5

DEFAULT_TURN_SPEED = 1.0
MAX_TURN_SPEED = 10.0
TURN_SPEED_INCREMENT = 0.5

ZERO_COMMAND = [0.0, 0.0, 0.0, 0.0]

KEY_UP = 'up'
KEY_DOWN = 'down'
KEY_LEFT = 'left'
KEY_RIGHT = 'right'
KEY_SHIFT_UP = 'shift_up'
KEY_SHIFT_DOWN = 'shift_down'
KEY_SHIFT_LEFT = 'shift_left'
KEY_SHIFT_RIGHT = 'shift_right'
KEY_STOP = 'stop'
KEY_ESTOP = 'estop'
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
    '\x1b[1;2A': KEY_SHIFT_UP,
    '\x1b[2A': KEY_SHIFT_UP,
    '\x1bO2A': KEY_SHIFT_UP,
    '\x1b[a': KEY_SHIFT_UP,
    '\x1b[1;2B': KEY_SHIFT_DOWN,
    '\x1b[2B': KEY_SHIFT_DOWN,
    '\x1bO2B': KEY_SHIFT_DOWN,
    '\x1b[b': KEY_SHIFT_DOWN,
    '\x1b[1;2D': KEY_SHIFT_LEFT,
    '\x1b[2D': KEY_SHIFT_LEFT,
    '\x1bO2D': KEY_SHIFT_LEFT,
    '\x1b[d': KEY_SHIFT_LEFT,
    '\x1b[1;2C': KEY_SHIFT_RIGHT,
    '\x1b[2C': KEY_SHIFT_RIGHT,
    '\x1bO2C': KEY_SHIFT_RIGHT,
    '\x1b[c': KEY_SHIFT_RIGHT,
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

    def read_key(self) -> str:
        if self._fd is None:
            raise RuntimeError('terminal reader is not active')

        if not self._buffer:
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


class Go2WKeyboardTeleop(Node):
    """Publish wheel velocity arrays from keyboard commands."""

    def __init__(self) -> None:
        super().__init__('go2w_keyboard_teleop')
        self._publisher = self.create_publisher(Float64MultiArray, WHEEL_COMMAND_TOPIC, 10)
        self.linear_speed = DEFAULT_LINEAR_SPEED
        self.turn_speed = DEFAULT_TURN_SPEED

    def print_help(self) -> None:
        print(
            f"""
====================================
Go2-W Keyboard Teleop
====================================

↑        Forward
↓        Reverse
←        Rotate Left
→        Rotate Right

Shift + ↑    Increase Forward Speed
Shift + ↓    Increase Reverse Speed
Shift + ←    Increase Left Rotation Speed
Shift + →    Increase Right Rotation Speed

S        Stop immediately
Space    Emergency Stop
H        Print help
Q        Quit

Current speed: {self.linear_speed:.1f}
Current turn speed: {self.turn_speed:.1f}

====================================
""",
            flush=True,
        )

    def handle_key(self, sequence: str) -> bool:
        key = self._normalize_key(sequence)
        if key == KEY_UP:
            self._publish_wheels([self.linear_speed] * 4, 'Forward')
        elif key == KEY_DOWN:
            self._publish_wheels([-self.linear_speed] * 4, 'Reverse')
        elif key == KEY_LEFT:
            w = self.turn_speed
            self._publish_wheels([-w, w, -w, w], 'Rotate left')
        elif key == KEY_RIGHT:
            w = self.turn_speed
            self._publish_wheels([w, -w, w, -w], 'Rotate right')
        elif key in (KEY_SHIFT_UP, KEY_SHIFT_DOWN):
            self.linear_speed = min(
                MAX_LINEAR_SPEED,
                max(MIN_LINEAR_SPEED, self.linear_speed + LINEAR_SPEED_INCREMENT),
            )
            print(f'Current speed: {self.linear_speed:.1f}', flush=True)
        elif key in (KEY_SHIFT_LEFT, KEY_SHIFT_RIGHT):
            self.turn_speed = min(MAX_TURN_SPEED, self.turn_speed + TURN_SPEED_INCREMENT)
            print(f'Current turn speed: {self.turn_speed:.1f}', flush=True)
        elif key == KEY_STOP:
            self.stop('Stop')
        elif key == KEY_ESTOP:
            self.stop('Emergency stop')
        elif key == KEY_HELP:
            self.print_help()
        elif key == KEY_QUIT:
            print('Quit requested.', flush=True)
            return False

        return True

    def stop(self, label: str = 'Stop') -> None:
        self._publish_wheels(ZERO_COMMAND, label)

    def _publish_wheels(self, velocities: list[float], label: str) -> None:
        msg = Float64MultiArray()
        command = [float(value) for value in velocities]
        msg.data = command
        self._publisher.publish(msg)
        print(f'{label}: {command}', flush=True)

    @staticmethod
    def _normalize_key(sequence: str) -> str | None:
        if sequence in KEY_SEQUENCES:
            return KEY_SEQUENCES[sequence]
        if sequence == ' ':
            return KEY_ESTOP
        if len(sequence) == 1:
            key = sequence.lower()
            if key == 's':
                return KEY_STOP
            if key == 'h':
                return KEY_HELP
            if key == 'q':
                return KEY_QUIT
        return None


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2WKeyboardTeleop()

    try:
        with TerminalKeyReader() as reader:
            node.print_help()
            while rclpy.ok():
                if not node.handle_key(reader.read_key()):
                    break
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        print('\nCtrl+C received.', flush=True)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        if rclpy.ok():
            node.stop('Exit stop')
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
