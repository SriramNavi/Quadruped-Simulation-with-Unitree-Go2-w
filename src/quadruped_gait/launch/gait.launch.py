from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gait_name = LaunchConfiguration('gait_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when true.',
        ),
        DeclareLaunchArgument(
            'gait_name',
            default_value='trot',
            description='Initial gait name. Supported placeholders: trot, crawl.',
        ),
        Node(
            package='quadruped_gait',
            executable='gait_scheduler',
            name='gait_scheduler',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'gait_name': gait_name},
            ],
        ),
    ])
