"""Placeholder gravity compensation node.

For now this publishes zero torques at a fixed rate. Robot-specific mass and
link parameters can later be injected behind the same torque interface.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from quadruped_control.defaults import DEFAULT_JOINT_NAMES
from quadruped_control.torque_interface import TorqueCommandInterface


class GravityCompensationNode(Node):
    def __init__(self) -> None:
        super().__init__('gravity_compensation')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('update_rate_hz', 100.0)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self._torque_interface = TorqueCommandInterface(
            self,
            topic='/quadruped_control/gravity_compensation_torques',
        )
        update_rate = max(float(self.get_parameter('update_rate_hz').value), 1.0)
        self.create_timer(1.0 / update_rate, self._publish_zero_torques)
        self.get_logger().info('Gravity compensation placeholder publishing zero torques')

    def _publish_zero_torques(self) -> None:
        self._torque_interface.publish(self.joint_names, [0.0] * len(self.joint_names))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityCompensationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
