"""Launch safe Go2-W cmd_vel keyboard teleoperation without Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the Go2-W teleop launch description."""
    control_share = get_package_share_directory('quadruped_control')
    controller_config = os.path.join(
        control_share,
        'config',
        'go2w_drive_controller.yaml',
    )

    max_linear_velocity = LaunchConfiguration('max_linear_velocity_mps')
    max_reverse_velocity = LaunchConfiguration('max_reverse_velocity_mps')
    max_wheel_velocity = LaunchConfiguration('max_wheel_angular_velocity_radps')
    minimum_turning_radius = LaunchConfiguration('minimum_turning_radius_m')
    allow_pivot_turn = LaunchConfiguration('allow_pivot_turn')

    common_overrides = {
        'max_linear_velocity_mps': ParameterValue(
            max_linear_velocity,
            value_type=float,
        ),
        'max_reverse_velocity_mps': ParameterValue(
            max_reverse_velocity,
            value_type=float,
        ),
        'max_wheel_angular_velocity_radps': ParameterValue(
            max_wheel_velocity,
            value_type=float,
        ),
        'minimum_turning_radius_m': ParameterValue(
            minimum_turning_radius,
            value_type=float,
        ),
        'allow_pivot_turn': ParameterValue(allow_pivot_turn, value_type=bool),
    }

    keyboard_node = Node(
        package='quadruped_control',
        executable='go2w_keyboard_cmd_vel',
        name='go2w_keyboard_cmd_vel',
        output='screen',
        emulate_tty=True,
        parameters=[controller_config, common_overrides],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'max_linear_velocity_mps',
            default_value='1.5',
            description='Maximum forward body velocity in m/s.',
        ),
        DeclareLaunchArgument(
            'max_reverse_velocity_mps',
            default_value='1.0',
            description='Maximum reverse body velocity magnitude in m/s.',
        ),
        DeclareLaunchArgument(
            'max_wheel_angular_velocity_radps',
            default_value='10.0',
            description='Maximum wheel angular velocity magnitude in rad/s.',
        ),
        DeclareLaunchArgument(
            'minimum_turning_radius_m',
            default_value='0.75',
            description='Minimum non-pivot turning radius in metres.',
        ),
        DeclareLaunchArgument(
            'allow_pivot_turn',
            default_value='true',
            description='Allow limited pivot turns near zero linear speed.',
        ),
        Node(
            package='quadruped_control',
            executable='go2w_cmd_vel_to_wheels',
            name='go2w_cmd_vel_to_wheels',
            output='screen',
            parameters=[controller_config, common_overrides],
        ),
        keyboard_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=keyboard_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason='Go2-W keyboard teleop exited'),
                    ),
                ],
            ),
        ),
    ])
