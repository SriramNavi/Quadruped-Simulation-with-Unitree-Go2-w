from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_launch = PathJoinSubstitution([
        FindPackageShare('quadruped_description'),
        'launch',
        'robot_state_publisher.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='custom_surivona_quad',
            description='Robot configuration name to bring up.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when true.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_publisher_launch),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='quadruped_gait',
            executable='gait_scheduler',
            name='gait_scheduler',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='quadruped_state_estimation',
            executable='base_state_estimator',
            name='base_state_estimator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
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
