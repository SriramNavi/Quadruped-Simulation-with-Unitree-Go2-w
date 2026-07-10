import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_xacro_path = os.path.join(
        get_package_share_directory('quadruped_description'),
        'urdf',
        'generic_quadruped.xacro',
    )
    xacro_file = LaunchConfiguration('xacro_file', default=default_xacro_path)

    robot_description = Command([
        'xacro ',
        xacro_file,
        ' robot_name:=',
        robot_name,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='custom_surivona_quad',
            description='Robot name passed to the generic quadruped Xacro.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when true.',
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('quadruped_description'),
                'urdf',
                'generic_quadruped.xacro',
            ]),
            description='Xacro file to publish as robot_description.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
