"""Launch only the Go2-W slope monitor and ground-truth pose plumbing."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetLaunchConfiguration, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from quadruped_control.go2w_friction_presets import (
    ALL_PARAMETERS,
    load_configuration,
    resolve_friction,
)


FRICTION_PARAMETERS = ALL_PARAMETERS


def _resolve_friction(context, *, preset_file):
    configuration = load_configuration(preset_file)
    preset = LaunchConfiguration('friction_preset').perform(context)
    try:
        friction = resolve_friction(configuration, preset, {
            name: LaunchConfiguration(name).perform(context)
            for name in FRICTION_PARAMETERS
        })
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    return [
        *(SetLaunchConfiguration(
            f'resolved_{name}', str(friction.values[name]),
        )
          for name in FRICTION_PARAMETERS),
        SetLaunchConfiguration(
            'resolved_effective_friction_label', friction.effective_label,
        ),
        LogInfo(msg=(
            f'Go2-W monitor effective friction metadata: '
            f'{friction.effective_label}'
        )),
    ]


def generate_launch_description():
    """Create the monitoring-only launch description."""
    control_share = get_package_share_directory('quadruped_control')
    go2w_share = get_package_share_directory('unitree_go2w_description')
    config = os.path.join(control_share, 'config', 'go2w_slope_test_monitor.yaml')
    preset_file = os.path.join(go2w_share, 'config', 'go2w_friction_presets.yaml')

    ramp_angle = LaunchConfiguration('ramp_angle_deg')
    lane_y = LaunchConfiguration('lane_y')
    trial_id = LaunchConfiguration('trial_id')
    target_speed = LaunchConfiguration('target_speed_mps')
    operator_note = LaunchConfiguration('operator_note')
    log_directory = LaunchConfiguration('log_directory')
    trial_timeout = LaunchConfiguration('trial_timeout_sec')
    auto_stop = LaunchConfiguration('auto_stop_logging_on_result')
    auto_drive = LaunchConfiguration('auto_drive_enabled')
    auto_start = LaunchConfiguration('auto_start')
    acceleration = LaunchConfiguration('acceleration_mps2')
    deceleration = LaunchConfiguration('deceleration_mps2')
    countdown = LaunchConfiguration('countdown_sec')
    precheck_timeout = LaunchConfiguration('precheck_timeout_sec')
    monitor_start_delay = LaunchConfiguration('monitor_start_delay_sec')
    expected_start_x = LaunchConfiguration('expected_start_x_m')
    expected_start_y = LaunchConfiguration('expected_start_y_m')
    expected_start_yaw = LaunchConfiguration('expected_start_yaw_deg')
    shutdown_after_result = LaunchConfiguration('shutdown_after_result')
    stop_on_warning = LaunchConfiguration('stop_on_warning')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_pose_topic = LaunchConfiguration('gazebo_pose_topic')
    model_pose_index = LaunchConfiguration('ground_truth_model_pose_index')
    spawn_tolerance = LaunchConfiguration('ground_truth_spawn_tolerance_m')
    friction_preset = LaunchConfiguration('friction_preset')
    pose_array_topic = '/go2w/ground_truth/pose_array'
    odometry_topic = '/go2w/ground_truth/odom'
    monitor_node = Node(
        package='quadruped_control',
        executable='go2w_slope_test_monitor',
        name='go2w_slope_test_monitor',
        output='screen',
        parameters=[
            config,
            {
                'ramp_angle_deg': ParameterValue(ramp_angle, value_type=float),
                'lane_y': ParameterValue(lane_y, value_type=float),
                'trial_id': trial_id,
                'target_speed_mps': ParameterValue(target_speed, value_type=float),
                'operator_note': operator_note,
                'log_directory': log_directory,
                'friction_preset': friction_preset,
                'effective_friction_label': LaunchConfiguration(
                    'resolved_effective_friction_label'
                ),
                **{
                    name: ParameterValue(
                        LaunchConfiguration(f'resolved_{name}'), value_type=float,
                    )
                    for name in FRICTION_PARAMETERS
                },
                'trial_timeout_sec': ParameterValue(trial_timeout, value_type=float),
                'auto_stop_logging_on_result': ParameterValue(
                    auto_stop, value_type=bool,
                ),
                'auto_drive_enabled': ParameterValue(auto_drive, value_type=bool),
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'acceleration_mps2': ParameterValue(acceleration, value_type=float),
                'deceleration_mps2': ParameterValue(deceleration, value_type=float),
                'countdown_sec': ParameterValue(countdown, value_type=float),
                'precheck_timeout_sec': ParameterValue(
                    precheck_timeout, value_type=float,
                ),
                'expected_start_x_m': ParameterValue(
                    expected_start_x, value_type=float,
                ),
                'expected_start_y_m': ParameterValue(
                    expected_start_y, value_type=float,
                ),
                'expected_start_yaw_deg': ParameterValue(
                    expected_start_yaw, value_type=float,
                ),
                'shutdown_after_result': ParameterValue(
                    shutdown_after_result, value_type=bool,
                ),
                'stop_on_warning': ParameterValue(stop_on_warning, value_type=bool),
                'odometry_topic': odometry_topic,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ramp_angle_deg', default_value='15'),
        DeclareLaunchArgument('lane_y', default_value='-6.0'),
        DeclareLaunchArgument('trial_id', default_value='trial_01'),
        DeclareLaunchArgument('target_speed_mps', default_value='0.25'),
        DeclareLaunchArgument('operator_note', default_value=''),
        DeclareLaunchArgument(
            'log_directory', default_value='',
        ),
        DeclareLaunchArgument('trial_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument(
            'auto_stop_logging_on_result', default_value='true',
        ),
        DeclareLaunchArgument('auto_drive_enabled', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('acceleration_mps2', default_value='0.20'),
        DeclareLaunchArgument('deceleration_mps2', default_value='0.35'),
        DeclareLaunchArgument('countdown_sec', default_value='3.0'),
        DeclareLaunchArgument('precheck_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('monitor_start_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('expected_start_x_m', default_value='-3.0'),
        DeclareLaunchArgument('expected_start_y_m', default_value=lane_y),
        DeclareLaunchArgument('expected_start_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('shutdown_after_result', default_value='false'),
        DeclareLaunchArgument('stop_on_warning', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gazebo_pose_topic',
            default_value='/world/go2w_slope_test/dynamic_pose/info',
            description='Gazebo Pose_V topic for the active world.',
        ),
        DeclareLaunchArgument(
            'ground_truth_model_pose_index',
            default_value='0',
            description=(
                'PoseArray index verified as go2w in go2w_slope_test; the bridge '
                'does not preserve Gazebo Pose_V entity names.'
            ),
        ),
        DeclareLaunchArgument(
            'ground_truth_spawn_tolerance_m',
            default_value='0.75',
            description='Maximum initial XY error allowed for the selected pose.',
        ),
        DeclareLaunchArgument(
            'friction_preset',
            default_value='dry_concrete',
            description='Named preset providing defaults; source YAML is read-only.',
        ),
        *(
            DeclareLaunchArgument(
                name,
                default_value='',
                description=(
                    f'Optional {name} override; explicit zero is valid and takes '
                    'precedence over the preset.'
                ),
            )
            for name in FRICTION_PARAMETERS
        ),
        OpaqueFunction(
            function=_resolve_friction,
            kwargs={'preset_file': preset_file},
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='go2w_ground_truth_pose_bridge',
            output='screen',
            arguments=[[
                gazebo_pose_topic,
                '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            ]],
            remappings=[(gazebo_pose_topic, pose_array_topic)],
        ),
        Node(
            package='quadruped_control',
            executable='go2w_ground_truth_adapter',
            name='go2w_ground_truth_adapter',
            output='screen',
            parameters=[{
                'pose_array_topic': pose_array_topic,
                'odometry_topic': odometry_topic,
                'model_pose_index': ParameterValue(model_pose_index, value_type=int),
                'expected_spawn_x_m': ParameterValue(
                    expected_start_x, value_type=float,
                ),
                'expected_spawn_y_m': ParameterValue(
                    expected_start_y, value_type=float,
                ),
                'spawn_position_tolerance_m': ParameterValue(
                    spawn_tolerance, value_type=float,
                ),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        TimerAction(
            period=monitor_start_delay,
            actions=[monitor_node],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=monitor_node,
                on_exit=[EmitEvent(event=Shutdown(reason='slope monitor exited'))],
            )
        ),
    ])
