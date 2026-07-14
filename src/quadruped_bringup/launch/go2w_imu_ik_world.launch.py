"""Two-terminal Go2-W IMU/IK development world (controller not included)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


PLOT_TOPICS = [
    '/go2w/imu_ik/pitch/raw_deg/data',
    '/go2w/imu_ik/pitch/filtered_deg/data',
    '/go2w/imu_ik/pitch/target_deg/data',
    '/go2w/imu_ik/pitch/error_deg/data',
    '/go2w/imu_ik/pid/output_deg/data',
    '/go2w/imu_ik/offset/front_m/data',
    '/go2w/imu_ik/offset/rear_m/data',
]

# VS Code's snap exports GTK module paths from its private core runtime. Native
# ROS Qt processes must not load those ABI-incompatible modules.
GUI_ENVIRONMENT = {
    'GTK_PATH': '',
    'GTK_EXE_PREFIX': '',
    'GDK_PIXBUF_MODULE_FILE': '',
    'GDK_PIXBUF_MODULEDIR': '',
    'GSETTINGS_SCHEMA_DIR': '',
    'GTK_IM_MODULE_FILE': '',
    'GIO_MODULE_DIR': '',
    'LOCPATH': '',
    'XDG_DATA_DIRS': os.environ.get(
        'XDG_DATA_DIRS_VSCODE_SNAP_ORIG', '/usr/local/share:/usr/share'
    ),
    'XDG_CONFIG_DIRS': os.environ.get(
        'XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG', '/etc/xdg'
    ),
}


def generate_launch_description():
    bringup_share = get_package_share_directory('quadruped_bringup')
    spawn_on_ramp = LaunchConfiguration('spawn_on_ramp')
    reference_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'go2w_reference.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'friction_preset': LaunchConfiguration('friction_preset'),
            'spawn_x': PythonExpression([
                "'3.5' if '", spawn_on_ramp, "' == 'true' else '-3.0'",
            ]),
            'spawn_y': '-6.0',
            'spawn_z': PythonExpression([
                "'0.95' if '", spawn_on_ramp, "' == 'true' else '0.45'",
            ]),
            'spawn_pitch': '0.0',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('friction_preset', default_value='dry_concrete'),
        DeclareLaunchArgument('spawn_on_ramp', default_value='true'),
        DeclareLaunchArgument('launch_rqt_graph', default_value='true'),
        DeclareLaunchArgument('launch_rqt_plot', default_value='true'),
        reference_launch,
        Node(
            package='rqt_graph', executable='rqt_graph', name='go2w_rqt_graph',
            output='screen', additional_env=GUI_ENVIRONMENT,
            condition=IfCondition(LaunchConfiguration('launch_rqt_graph')),
        ),
        Node(
            package='rqt_plot', executable='rqt_plot', name='go2w_rqt_plot',
            output='screen', arguments=['--empty', *PLOT_TOPICS],
            additional_env=GUI_ENVIRONMENT,
            condition=IfCondition(LaunchConfiguration('launch_rqt_plot')),
        ),
    ])
