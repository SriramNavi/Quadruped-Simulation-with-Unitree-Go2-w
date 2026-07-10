"""Minimal gait scheduler that publishes per-leg foot targets."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.parameter import Parameter

from quadruped_gait.gait_patterns import LEG_NAMES, duty_factor_for_gait, phase_offset_for_gait
from quadruped_gait.swing_trajectory import SwingTrajectory
from quadruped_msgs.msg import FootTarget, GaitCommand


class GaitSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__('gait_scheduler')

        self.declare_parameter('gait_name', 'trot')
        self.declare_parameter('cycle_time_s', 0.6)
        self.declare_parameter('step_length', 0.08)
        self.declare_parameter('step_height', 0.04)
        self.declare_parameter('nominal_body_height', 0.30)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._gait_name = str(self.get_parameter('gait_name').value)
        self._cycle_time_s = max(float(self.get_parameter('cycle_time_s').value), 0.1)
        self._enabled = True
        self._phase = 0.0
        self._foot_targets_pub = self.create_publisher(FootTarget, '/quadruped_control/foot_targets', 10)
        self.create_subscription(GaitCommand, '/quadruped_gait/command', self._on_gait_command, 10)

        publish_rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self._dt = 1.0 / publish_rate
        self.create_timer(self._dt, self._update)
        self.get_logger().info(f'Gait scheduler ready with gait={self._gait_name}')

    def _on_gait_command(self, msg: GaitCommand) -> None:
        if msg.gait_name:
            self._gait_name = msg.gait_name
        self._enabled = bool(msg.enabled)
        if msg.step_height > 0.0:
            self.set_parameters([
                Parameter(
                    'step_height',
                    Parameter.Type.DOUBLE,
                    float(msg.step_height),
                )
            ])

    def _update(self) -> None:
        if not self._enabled:
            return

        self._phase = (self._phase + self._dt / self._cycle_time_s) % 1.0
        duty_factor = duty_factor_for_gait(self._gait_name)
        trajectory = SwingTrajectory(
            step_length=float(self.get_parameter('step_length').value),
            step_height=float(self.get_parameter('step_height').value),
            nominal_z=-float(self.get_parameter('nominal_body_height').value),
        )

        for leg_name in LEG_NAMES:
            leg_phase = (self._phase + phase_offset_for_gait(self._gait_name, leg_name)) % 1.0
            nominal_x = 0.18 if 'front' in leg_name else -0.18
            nominal_y = 0.11 if 'left' in leg_name else -0.11

            if leg_phase >= duty_factor:
                swing_phase = (leg_phase - duty_factor) / max(1.0 - duty_factor, 1e-6)
                x, y, z = trajectory.sample(swing_phase, nominal_x, nominal_y)
                contact_probability = 0.0
            else:
                stance_phase = leg_phase / max(duty_factor, 1e-6)
                x = nominal_x - trajectory.step_length * (stance_phase - 0.5)
                y = nominal_y
                z = trajectory.nominal_z
                contact_probability = 1.0

            msg = FootTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.leg_name = leg_name
            msg.position = Point(x=x, y=y, z=z)
            msg.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            msg.contact_probability = contact_probability
            self._foot_targets_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitSchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
