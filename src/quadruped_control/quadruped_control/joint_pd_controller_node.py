"""Minimal joint-space PD controller node."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quadruped_control.defaults import DEFAULT_JOINT_NAMES
from quadruped_control.torque_interface import TorqueCommandInterface
from quadruped_utils.math_utils import saturate_sequence


class JointPdControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_pd_controller')

        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('kp', 30.0)
        self.declare_parameter('kd', 0.8)
        self.declare_parameter('torque_limit', 20.0)
        self.declare_parameter('update_rate_hz', 200.0)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)

        self._positions = {name: 0.0 for name in self.joint_names}
        self._velocities = {name: 0.0 for name in self.joint_names}
        self._targets = {name: 0.0 for name in self.joint_names}
        self._target_velocities = {name: 0.0 for name in self.joint_names}

        self._torque_interface = TorqueCommandInterface(self)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(JointState, '/quadruped_control/joint_targets', self._on_joint_target, 10)

        update_rate = max(float(self.get_parameter('update_rate_hz').value), 1.0)
        self.create_timer(1.0 / update_rate, self._update)
        self.get_logger().info(f'Joint PD controller ready for {len(self.joint_names)} joints')

    def _on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if name not in self._positions:
                continue
            if index < len(msg.position):
                self._positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])

    def _on_joint_target(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if name not in self._targets:
                continue
            if index < len(msg.position):
                self._targets[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._target_velocities[name] = float(msg.velocity[index])

    def _update(self) -> None:
        efforts = []
        for name in self.joint_names:
            position_error = self._targets[name] - self._positions[name]
            velocity_error = self._target_velocities[name] - self._velocities[name]
            efforts.append(self.kp * position_error + self.kd * velocity_error)

        saturated_efforts = saturate_sequence(efforts, self.torque_limit)
        self._torque_interface.publish(self.joint_names, saturated_efforts)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointPdControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
