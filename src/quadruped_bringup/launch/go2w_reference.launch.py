import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_available(package_name):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        return False
    return True


def generate_launch_description():
    go2w_share = get_package_share_directory("unitree_go2w_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    urdf_path = os.path.join(go2w_share, "urdf", "go2w_description.urdf")
    control_xacro_path = os.path.join(go2w_share, "urdf", "go2w_gazebo_control.urdf.xacro")
    controllers_path = os.path.join(go2w_share, "config", "go2w_ros2_control.yaml")
    rviz_config = os.path.join(go2w_share, "rviz", "go2w.rviz")
    gazebo_resource_root = os.path.dirname(go2w_share)

    gazebo = LaunchConfiguration("gazebo")
    gui = LaunchConfiguration("gui")
    manual_mode = LaunchConfiguration("manual_mode")
    rviz = LaunchConfiguration("rviz")
    use_reference_controller = LaunchConfiguration("use_reference_controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    jsp_package = "true" if package_available("joint_state_publisher") else "false"

    gazebo_with_gui = PythonExpression([
        "'", gazebo, "' == 'true' and '", gui, "' == 'true'",
    ])
    gazebo_headless = PythonExpression([
        "'", gazebo, "' == 'true' and '", gui, "' != 'true'",
    ])
    gazebo_manual_mode = PythonExpression([
        "'", gazebo, "' == 'true' and '", manual_mode, "' == 'true'",
    ])
    jsp_condition = PythonExpression([
        "'", manual_mode, "' == 'true' and '", gazebo, "' != 'true' and '",
        jsp_package, "' == 'true'",
    ])
    fallback_jsp_condition = PythonExpression([
        "'", manual_mode, "' == 'true' and '", gazebo, "' != 'true' and '",
        jsp_package, "' != 'true'",
    ])

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        control_xacro_path,
        " ",
        "base_urdf_file:=",
        urdf_path,
        " ",
        "controllers_file:=",
        controllers_path,
    ])

    robot_description_param = {
        "robot_description": ParameterValue(robot_description, value_type=str),
        "use_sim_time": use_sim_time,
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r"]}.items(),
        condition=IfCondition(gazebo_with_gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r -s --headless-rendering"]}.items(),
        condition=IfCondition(gazebo_headless),
    )

    spawn_go2w = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-topic",
                    "robot_description",
                    "-x",
                    LaunchConfiguration("world_init_x"),
                    "-y",
                    LaunchConfiguration("world_init_y"),
                    "-z",
                    LaunchConfiguration("world_init_z"),
                    "-Y",
                    LaunchConfiguration("world_init_heading"),
                ],
            )
        ],
        condition=IfCondition(gazebo),
    )

    spawn_manual_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "go2w_wheel_velocity_controller",
                    "go2w_leg_position_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                    "--switch-timeout",
                    "60",
                    "--activate-as-group",
                ],
            )
        ],
        condition=IfCondition(gazebo_manual_mode),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz."),
        DeclareLaunchArgument("gazebo", default_value="true", description="Start Gazebo Sim."),
        DeclareLaunchArgument("manual_mode", default_value="true", description="Manual Gazebo control mode."),
        DeclareLaunchArgument(
            "use_reference_controller",
            default_value="false",
            description="Reserved for future Go2-W reference controllers.",
        ),
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world file."),
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock."),
        DeclareLaunchArgument("robot_name", default_value="go2w", description="Spawned model name."),
        DeclareLaunchArgument("world_init_x", default_value="0.0", description="Initial x position."),
        DeclareLaunchArgument("world_init_y", default_value="0.0", description="Initial y position."),
        DeclareLaunchArgument("world_init_z", default_value="0.45", description="Initial z position."),
        DeclareLaunchArgument("world_init_heading", default_value="0.0", description="Initial yaw."),
        LogInfo(msg="Go2-W reference launch: Gazebo manual mode starts controllers but no motion commands."),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                gazebo_resource_root,
                ":",
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        LogInfo(
            msg="use_reference_controller is reserved for future work; no autonomous Go2-W locomotion controller is started.",
            condition=IfCondition(use_reference_controller),
        ),
        gz_sim,
        gz_sim_headless,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="go2w_joint_state_publisher",
            output="screen",
            condition=IfCondition(jsp_condition),
        ),
        LogInfo(
            msg="joint_state_publisher package unavailable; using Go2-W zero-state fallback.",
            condition=IfCondition(fallback_jsp_condition),
        ),
        Node(
            package="unitree_go2w_description",
            executable="go2w_zero_joint_state_publisher.py",
            name="go2w_zero_joint_state_publisher",
            output="screen",
            parameters=[robot_description_param],
            condition=IfCondition(fallback_jsp_condition),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="go2w_clock_bridge",
            output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            condition=IfCondition(gazebo),
        ),
        spawn_go2w,
        spawn_manual_controllers,
        Node(
            package="rviz2",
            executable="rviz2",
            name="go2w_rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
