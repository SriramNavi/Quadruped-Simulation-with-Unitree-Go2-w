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
            package='quadruped_control',
            executable='cartesian_foot_controller',
            name='cartesian_foot_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='quadruped_control',
            executable='joint_pd_controller',
            name='joint_pd_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
