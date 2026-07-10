from sensor_msgs.msg import Imu, JointState

from quadruped_state_estimation.base_estimator import BaseEstimator


def test_estimator_stores_latest_joint_state() -> None:
    estimator = BaseEstimator()
    msg = JointState()
    msg.name = ['joint_a']
    msg.position = [1.2]
    estimator.update_from_joint_state(msg)
    assert estimator.joint_state.name == ['joint_a']
    assert list(estimator.joint_state.position) == [1.2]


def test_estimator_updates_orientation_from_imu() -> None:
    estimator = BaseEstimator()
    imu = Imu()
    imu.orientation.w = 1.0
    estimator.update_from_imu(imu)
    assert estimator.base_pose.orientation.w == 1.0
