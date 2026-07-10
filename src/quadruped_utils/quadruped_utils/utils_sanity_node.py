"""Tiny executable sanity check for the utility package."""

import rclpy
from rclpy.node import Node

from quadruped_utils.math_utils import saturate_sequence


class UtilsSanityNode(Node):
    def __init__(self) -> None:
        super().__init__('utils_sanity')
        saturated = saturate_sequence([-2.0, 0.5, 3.0], 1.0)
        self.get_logger().info(f'quadruped_utils sanity check: {saturated}')
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self) -> None:
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UtilsSanityNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
