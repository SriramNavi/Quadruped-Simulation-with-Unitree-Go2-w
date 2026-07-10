#!/usr/bin/env python3
import xml.etree.ElementTree as ElementTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self):
        super().__init__("go2w_zero_joint_state_publisher")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("publish_rate", 30.0)

        robot_description = self.get_parameter("robot_description").value
        self.joint_names = self._joint_names_from_urdf(robot_description)
        self.publisher = self.create_publisher(JointState, "joint_states", 10)

        publish_rate = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(1.0 / float(publish_rate), self.publish_joint_state)

        self.get_logger().info(
            f"Publishing zero state for {len(self.joint_names)} movable Go2-W joints."
        )

    @staticmethod
    def _joint_names_from_urdf(robot_description):
        root = ElementTree.fromstring(robot_description)
        joint_names = []
        for joint in root.findall("joint"):
            if joint.attrib.get("type") == "fixed":
                continue
            name = joint.attrib.get("name")
            if name:
                joint_names.append(name)
        return joint_names

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ZeroJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
