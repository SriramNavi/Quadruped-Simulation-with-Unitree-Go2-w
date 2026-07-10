"""Torque command publishing abstraction.

The first hardware port can replace this class with a bridge to a vendor SDK or
ros2_control command interface while controller nodes keep the same boundary.
"""

from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TorqueCommandInterface:
    """Publish torque commands in a simple robot-agnostic format."""

    def __init__(self, node: Node, topic: str = '/quadruped_control/torque_commands') -> None:
        self._node = node
        self._array_publisher = node.create_publisher(Float64MultiArray, topic, 10)
        self._joint_state_publisher = node.create_publisher(
            JointState,
            '/quadruped_control/torque_joint_state',
            10,
        )

    def publish(self, joint_names: list[str], efforts: list[float]) -> None:
        array_msg = Float64MultiArray()
        array_msg.data = [float(effort) for effort in efforts]
        self._array_publisher.publish(array_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self._node.get_clock().now().to_msg()
        joint_msg.name = list(joint_names)
        joint_msg.effort = [float(effort) for effort in efforts]
        self._joint_state_publisher.publish(joint_msg)
