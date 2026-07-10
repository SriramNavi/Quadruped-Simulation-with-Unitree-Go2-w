"""Base pose and velocity estimator placeholder."""

from __future__ import annotations

from dataclasses import dataclass, field

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu, JointState


@dataclass
class BaseEstimator:
    """Hold the latest sensor-derived base estimate.

    The current implementation is intentionally conservative: it mirrors IMU
    orientation and angular velocity without integrating drift-prone position.
    """

    base_pose: Pose = field(default_factory=Pose)
    base_twist: Twist = field(default_factory=Twist)
    joint_state: JointState = field(default_factory=JointState)

    def update_from_imu(self, imu_msg: Imu) -> None:
        self.base_pose.orientation = imu_msg.orientation
        self.base_twist.angular = imu_msg.angular_velocity

    def update_from_joint_state(self, joint_state_msg: JointState) -> None:
        self.joint_state = joint_state_msg
