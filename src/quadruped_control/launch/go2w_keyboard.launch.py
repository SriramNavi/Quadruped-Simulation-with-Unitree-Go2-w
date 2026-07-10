from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_control',
            executable='go2w_keyboard_teleop',
            name='go2w_keyboard_teleop',
            output='screen',
            emulate_tty=True,
        ),
    ])
