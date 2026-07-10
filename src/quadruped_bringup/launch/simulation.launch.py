from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='custom_surivona_quad',
            description='Robot configuration name for future Gazebo/Ignition simulation.',
        ),
        LogInfo(msg=[
            'Simulation launch placeholder for ',
            robot_name,
            '. Add Gazebo/Ignition system launch here when the simulator model is ready.',
        ]),
    ])
