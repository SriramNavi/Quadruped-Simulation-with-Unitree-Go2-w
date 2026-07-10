"""Executable sanity check for the generic leg model."""

import rclpy
from rclpy.node import Node

from quadruped_kinematics.leg_model import LegGeometry, ThreeDofLeg


class KinematicsSanityNode(Node):
    def __init__(self) -> None:
        super().__init__('kinematics_sanity')
        leg = ThreeDofLeg(LegGeometry(side_sign=1.0))
        target = (0.05, 0.05, -0.32)
        joints = leg.inverse_kinematics(target)
        foot = leg.forward_kinematics(joints)
        self.get_logger().info(
            'IK/FK sanity target=%s joints=%s foot=%s' % (
                tuple(round(v, 4) for v in target),
                tuple(round(v, 4) for v in joints),
                tuple(round(v, 4) for v in foot),
            )
        )
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self) -> None:
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicsSanityNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
