from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    unitree_go2_launch = PathJoinSubstitution([
        FindPackageShare('unitree_go2_sim'),
        'launch',
        'unitree_go2_launch.py',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_champ = LaunchConfiguration('use_champ')
    use_rendering_sensors = LaunchConfiguration('use_rendering_sensors')
    rviz = LaunchConfiguration('rviz')
    robot_name = LaunchConfiguration('robot_name')
    world_init_x = LaunchConfiguration('world_init_x')
    world_init_y = LaunchConfiguration('world_init_y')
    world_init_z = LaunchConfiguration('world_init_z')
    world_init_heading = LaunchConfiguration('world_init_heading')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'use_champ',
            default_value='true',
            description='Launch CHAMP gait/control nodes when true.',
        ),
        DeclareLaunchArgument(
            'use_rendering_sensors',
            default_value='true',
            description='Include Gazebo camera and GPU lidar sensors in the robot model.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start the upstream RViz configuration with the simulation.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='go2',
            description='Robot name passed to the upstream Go2 launch.',
        ),
        DeclareLaunchArgument(
            'world_init_x',
            default_value='0.0',
            description='Initial Gazebo x position.',
        ),
        DeclareLaunchArgument(
            'world_init_y',
            default_value='0.0',
            description='Initial Gazebo y position.',
        ),
        DeclareLaunchArgument(
            'world_init_z',
            default_value='0.375',
            description='Initial Gazebo z position.',
        ),
        DeclareLaunchArgument(
            'world_init_heading',
            default_value='0.0',
            description='Initial Gazebo heading.',
        ),
        LogInfo(msg='Including Unitree Go2 reference simulation launch.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unitree_go2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_champ': use_champ,
                'use_rendering_sensors': use_rendering_sensors,
                'rviz': rviz,
                'robot_name': robot_name,
                'world_init_x': world_init_x,
                'world_init_y': world_init_y,
                'world_init_z': world_init_z,
                'world_init_heading': world_init_heading,
            }.items(),
        ),
    ])
