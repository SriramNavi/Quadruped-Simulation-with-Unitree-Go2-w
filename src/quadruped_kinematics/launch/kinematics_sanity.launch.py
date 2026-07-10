from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_kinematics',
            executable='kinematics_sanity',
            name='kinematics_sanity',
            output='screen',
        ),
    ])
