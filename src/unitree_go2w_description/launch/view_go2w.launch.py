import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_available(package_name):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        return False
    return True


def generate_launch_description():
    pkg_share = get_package_share_directory("unitree_go2w_description")
    urdf_path = os.path.join(pkg_share, "urdf", "go2w_description.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "go2w.rviz")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui_package = "true" if package_available("joint_state_publisher_gui") else "false"
    jsp_package = "true" if package_available("joint_state_publisher") else "false"

    gui_condition = PythonExpression([
        "'", gui, "' == 'true' and '", gui_package, "' == 'true'",
    ])
    jsp_condition = PythonExpression([
        "('", gui, "' != 'true' or '", gui_package, "' != 'true') and '",
        jsp_package, "' == 'true'",
    ])
    fallback_condition = PythonExpression([
        "('", gui, "' != 'true' or '", gui_package, "' != 'true') and '",
        jsp_package, "' != 'true'",
    ])

    robot_description_param = {
        "robot_description": ParameterValue(robot_description, value_type=str),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Use joint_state_publisher_gui for manual joint inspection.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz with the Go2-W display config.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui_condition),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=IfCondition(jsp_condition),
        ),
        LogInfo(
            msg="joint_state_publisher package unavailable; using Go2-W zero-state fallback.",
            condition=IfCondition(fallback_condition),
        ),
        Node(
            package="unitree_go2w_description",
            executable="go2w_zero_joint_state_publisher.py",
            name="go2w_zero_joint_state_publisher",
            output="screen",
            parameters=[robot_description_param],
            condition=IfCondition(fallback_condition),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
