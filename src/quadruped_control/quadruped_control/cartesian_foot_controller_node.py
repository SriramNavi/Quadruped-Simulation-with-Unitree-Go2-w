"""Convert desired foot targets into joint targets using the generic leg IK."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quadruped_control.defaults import JOINT_SUFFIXES, LEG_NAMES
from quadruped_kinematics.leg_model import LegGeometry, ThreeDofLeg
from quadruped_msgs.msg import FootTarget


class CartesianFootControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('cartesian_foot_controller')

        self.declare_parameter('abduction_offset', 0.05)
        self.declare_parameter('thigh_length', 0.20)
        self.declare_parameter('calf_length', 0.20)

        abduction_offset = float(self.get_parameter('abduction_offset').value)
        thigh_length = float(self.get_parameter('thigh_length').value)
        calf_length = float(self.get_parameter('calf_length').value)

        self._legs = {
            leg_name: ThreeDofLeg(
                LegGeometry(
                    abduction_offset=abduction_offset,
                    thigh_length=thigh_length,
                    calf_length=calf_length,
                    side_sign=1.0 if 'left' in leg_name else -1.0,
                )
            )
            for leg_name in LEG_NAMES
        }
        self._latest_targets: dict[str, FootTarget] = {}

        self._joint_target_publisher = self.create_publisher(
            JointState,
            '/quadruped_control/joint_targets',
            10,
        )
        self.create_subscription(FootTarget, '/quadruped_control/foot_targets', self._on_foot_target, 10)
        self.create_timer(0.01, self._publish_joint_targets)
        self.get_logger().info('Cartesian foot controller ready')

    def _on_foot_target(self, msg: FootTarget) -> None:
        if msg.leg_name not in self._legs:
            self.get_logger().warn(f'Ignoring foot target for unknown leg: {msg.leg_name}')
            return
        self._latest_targets[msg.leg_name] = msg

    def _publish_joint_targets(self) -> None:
        if not self._latest_targets:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        for leg_name in LEG_NAMES:
            target = self._latest_targets.get(leg_name)
            if target is None:
                continue

            position = (
                float(target.position.x),
                float(target.position.y),
                float(target.position.z),
            )
            joints = self._legs[leg_name].inverse_kinematics(position)
            joint_msg.name.extend(f'{leg_name}_{suffix}' for suffix in JOINT_SUFFIXES)
            joint_msg.position.extend(joints)
            joint_msg.velocity.extend([0.0, 0.0, 0.0])

        if joint_msg.name:
            self._joint_target_publisher.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianFootControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
