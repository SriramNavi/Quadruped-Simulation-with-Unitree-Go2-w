"""Minimal IMU/joint-state based robot state publisher."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from quadruped_msgs.msg import ControllerMode, RobotState
from quadruped_state_estimation.base_estimator import BaseEstimator


class BaseStateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__('base_state_estimator')
        self._estimator = BaseEstimator()
        self._robot_state_publisher = self.create_publisher(RobotState, '/quadruped/state', 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_timer(0.02, self._publish_state)
        self.get_logger().info('Base state estimator placeholder ready')

    def _on_imu(self, msg: Imu) -> None:
        self._estimator.update_from_imu(msg)

    def _on_joint_state(self, msg: JointState) -> None:
        self._estimator.update_from_joint_state(msg)

    def _publish_state(self) -> None:
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.base_pose = self._estimator.base_pose
        msg.base_twist = self._estimator.base_twist
        msg.joint_state = self._estimator.joint_state
        msg.controller_mode = ControllerMode(
            mode=ControllerMode.MODE_IDLE,
            label='idle',
        )
        self._robot_state_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseStateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
