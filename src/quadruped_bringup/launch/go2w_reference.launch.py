import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
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
from quadruped_control.go2w_friction_presets import (
    ALL_PARAMETERS,
    generate_runtime_descriptions,
    load_configuration,
    resolve_friction,
)


FRICTION_PARAMETERS = ALL_PARAMETERS
GO2W_CONTROLLER_MANAGER = "/go2w_controller_manager"
GO2W_ROBOT_DESCRIPTION_TOPIC = "/go2w/robot_description"


def _configure_contact(
    context,
    *,
    preset_file,
    control_xacro_path,
    urdf_path,
    controllers_path,
):
    """Resolve contact parameters and render isolated runtime descriptions."""
    configuration = load_configuration(preset_file)
    preset_name = LaunchConfiguration("friction_preset").perform(context)
    overrides = {
        name: LaunchConfiguration(name).perform(context)
        for name in FRICTION_PARAMETERS
    }
    try:
        friction = resolve_friction(configuration, preset_name, overrides)
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    values = friction.values

    actions = [
        SetLaunchConfiguration(f"resolved_{name}", str(values[name]))
        for name in FRICTION_PARAMETERS
    ]
    actions.append(SetLaunchConfiguration("resolved_friction_preset", preset_name))

    try:
        runtime = generate_runtime_descriptions(
            friction=friction,
            source_world=LaunchConfiguration("world").perform(context),
            control_xacro=control_xacro_path,
            source_urdf=urdf_path,
            controllers_file=controllers_path,
            ramp_angle_deg=15.0,
        )
    except (OSError, ValueError) as error:
        raise RuntimeError(str(error)) from error
    actions.extend((
        SetLaunchConfiguration("world", str(runtime.world_sdf)),
        SetLaunchConfiguration("generated_contact_robot", str(runtime.robot_sdf)),
        SetLaunchConfiguration("generated_contact_world", str(runtime.world_sdf)),
        SetLaunchConfiguration("generated_contact_directory", str(runtime.directory)),
        SetLaunchConfiguration(
            "effective_friction_snapshot", str(runtime.effective_parameters),
        ),
        LogInfo(msg=(
            f"Selected friction preset: {preset_name}\n"
            f"Effective wheel mu: longitudinal={values['wheel_mu_longitudinal']} "
            f"lateral={values['wheel_mu_lateral']}\n"
            f"Effective terrain mu: longitudinal={values['terrain_mu_longitudinal']} "
            f"lateral={values['terrain_mu_lateral']}\n"
            f"Effective wheel slip: longitudinal={values['wheel_slip_longitudinal']} "
            f"lateral={values['wheel_slip_lateral']}\n"
            f"Runtime robot SDF: {runtime.robot_sdf}\n"
            f"Runtime world SDF: {runtime.world_sdf}\n"
            f"Effective parameter snapshot: {runtime.effective_parameters}"
        )),
    ))
    return actions


def _remove_generated_files(context):
    """Remove this launch's uniquely owned runtime directory."""
    import shutil

    path = LaunchConfiguration("generated_contact_directory").perform(context)
    if path:
        shutil.rmtree(path, ignore_errors=True)


def package_available(package_name):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        return False
    return True


def generate_launch_description():
    bringup_share = get_package_share_directory("quadruped_bringup")
    go2w_share = get_package_share_directory("unitree_go2w_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    slope_world_path = os.path.join(bringup_share, "worlds", "go2w_slope_test.sdf")
    urdf_path = os.path.join(go2w_share, "urdf", "go2w_description.urdf")
    control_xacro_path = os.path.join(go2w_share, "urdf", "go2w_gazebo_control.urdf.xacro")
    controllers_path = os.path.join(go2w_share, "config", "go2w_ros2_control.yaml")
    preset_file = os.path.join(go2w_share, "config", "go2w_friction_presets.yaml")
    rviz_config = os.path.join(go2w_share, "rviz", "go2w.rviz")
    bringup_resource_root = os.path.dirname(bringup_share)
    go2w_resource_root = os.path.dirname(go2w_share)

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
        " ",
        "wheel_mu_longitudinal:=",
        LaunchConfiguration("resolved_wheel_mu_longitudinal"),
        " ",
        "wheel_mu_lateral:=",
        LaunchConfiguration("resolved_wheel_mu_lateral"),
        " ",
        "wheel_slip_longitudinal:=",
        LaunchConfiguration("resolved_wheel_slip_longitudinal"),
        " ",
        "wheel_slip_lateral:=",
        LaunchConfiguration("resolved_wheel_slip_lateral"),
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
                    "-file",
                    LaunchConfiguration("generated_contact_robot"),
                    "-x",
                    LaunchConfiguration("world_init_x"),
                    "-y",
                    LaunchConfiguration("world_init_y"),
                    "-z",
                    LaunchConfiguration("world_init_z"),
                    "-R",
                    LaunchConfiguration("spawn_roll"),
                    "-P",
                    LaunchConfiguration("spawn_pitch"),
                    "-Y",
                    LaunchConfiguration("world_init_heading"),
                ],
            )
        ],
        condition=IfCondition(gazebo),
    )

    spawn_manual_controllers = TimerAction(
        period=4.5,
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
                    GO2W_CONTROLLER_MANAGER,
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
        DeclareLaunchArgument(
            "world_file",
            default_value=slope_world_path,
            description="Gazebo SDF world file; accepts an absolute path.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=LaunchConfiguration("world_file"),
            description="Legacy alias for world_file.",
        ),
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock."),
        DeclareLaunchArgument("robot_name", default_value="go2w", description="Spawned model name."),
        DeclareLaunchArgument("spawn_x", default_value="-3.0", description="Initial x position."),
        DeclareLaunchArgument("spawn_y", default_value="-6.0", description="Initial y position."),
        DeclareLaunchArgument("spawn_z", default_value="0.45", description="Initial z position."),
        DeclareLaunchArgument("spawn_roll", default_value="0.0", description="Initial roll."),
        DeclareLaunchArgument("spawn_pitch", default_value="0.0", description="Initial pitch."),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Initial yaw."),
        DeclareLaunchArgument(
            "friction_preset",
            default_value="dry_concrete",
            description=(
                "Named preset providing defaults; source YAML is read-only and "
                "runtime descriptions are generated under /tmp."
            ),
        ),
        *(
            DeclareLaunchArgument(
                name,
                default_value="",
                description=(
                    f"Optional {name} override; empty uses the preset, while an "
                    "explicit zero is valid and takes precedence. Source files "
                    "are not modified."
                ),
            )
            for name in FRICTION_PARAMETERS
        ),
        DeclareLaunchArgument(
            "world_init_x",
            default_value=LaunchConfiguration("spawn_x"),
            description="Legacy alias for spawn_x.",
        ),
        DeclareLaunchArgument(
            "world_init_y",
            default_value=LaunchConfiguration("spawn_y"),
            description="Legacy alias for spawn_y.",
        ),
        DeclareLaunchArgument(
            "world_init_z",
            default_value=LaunchConfiguration("spawn_z"),
            description="Legacy alias for spawn_z.",
        ),
        DeclareLaunchArgument(
            "world_init_heading",
            default_value=LaunchConfiguration("spawn_yaw"),
            description="Legacy alias for spawn_yaw.",
        ),
        OpaqueFunction(
            function=_configure_contact,
            kwargs={
                "preset_file": preset_file,
                "control_xacro_path": control_xacro_path,
                "urdf_path": urdf_path,
                "controllers_path": controllers_path,
            },
        ),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=_remove_generated_files)])
        ),
        LogInfo(msg="Go2-W reference launch: Gazebo manual mode starts controllers but no motion commands."),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                bringup_resource_root,
                ":",
                go2w_resource_root,
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
            name="go2w_robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
            remappings=[
                ("robot_description", GO2W_ROBOT_DESCRIPTION_TOPIC),
            ],
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
            name="go2w_gazebo_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
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
