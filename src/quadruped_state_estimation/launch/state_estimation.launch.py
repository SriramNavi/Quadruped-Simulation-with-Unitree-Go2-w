from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when true.',
        ),
        Node(
            package='quadruped_state_estimation',
            executable='base_state_estimator',
            name='base_state_estimator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
