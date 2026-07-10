# Unitree Go2 ROS 2 Integration

## Source

- Repository: https://github.com/khaledgabr77/unitree_go2_ros2.git
- Cloned to: `~/quad_ws/src/unitree_go2_ros2`
- Role: first reference robot model for quadruped algorithm development in this workspace.

## Packages Found

- `champ`
- `champ_base`
- `champ_msgs`
- `unitree_go2_description`
- `unitree_go2_sim`

## Important Launch Files

- `unitree_go2_sim/launch/unitree_go2_launch.py`
  - Starts Gazebo Sim through `ros_gz_sim`.
  - Starts `robot_state_publisher`.
  - Starts CHAMP controller and state estimation nodes.
  - Starts `robot_localization` EKF nodes.
  - Starts Gazebo bridges and ros2_control controller spawners.
  - Optionally starts RViz with `rviz:=true`.

Workspace bridge:

- `quadruped_bringup/launch/go2_reference.launch.py`
  - Includes the upstream Go2 launch file.
  - Defaults to `rviz:=false`.
  - Forwards `use_sim_time`, `robot_name`, and initial pose arguments.

## Important URDF/Xacro Files

- `unitree_go2_description/urdf/unitree_go2_robot.xacro`
- `unitree_go2_description/urdf/unitree_go2_gazebo.xacro`
- `unitree_go2_description/urdf/leg.xacro`
- `unitree_go2_description/urdf/const.xacro`
- `unitree_go2_description/urdf/materials.xacro`
- `unitree_go2_description/urdf/velodyne.xacro`
- `unitree_go2_description/urdf/lidar_4D_lidar.xacro`
- `unitree_go2_description/urdf/realsense_d455.xacro`

World file:

- `unitree_go2_description/worlds/default.sdf`

## Important Controller and Runtime Config Files

- `unitree_go2_sim/config/ros_control/ros_control.yaml`
  - Defines `joint_states_controller`.
  - Defines `joint_group_effort_controller`.
- `unitree_go2_sim/config/joints/joints.yaml`
- `unitree_go2_sim/config/links/links.yaml`
- `unitree_go2_sim/config/gait/gait.yaml`
- `champ_base/config/ekf/base_to_footprint.yaml`
- `champ_base/config/ekf/footprint_to_odom.yaml`
- `champ_base/config/velocity_smoother/velocity_smoother.yaml`
- `unitree_go2_sim/rviz/rviz.rviz`

## Dependencies Identified

From `package.xml`, README, and launch files:

- `ament_cmake`
- `launch_ros`
- `rclcpp`
- `rclpy`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `builtin_interfaces`
- `common_interfaces`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `trajectory_msgs`
- `visualization_msgs`
- `nav_msgs`
- `tf2_ros`
- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `ros_gz_sim`
- `ros_gz_bridge`
- `gz_ros2_control`
- `controller_manager`
- `robot_localization`
- `rviz2`
- `realsense2_description`
- `velodyne_description`
- `teleop_twist_keyboard`

Runtime dependency check after the first successful build:

- Found: `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `controller_manager`, `rviz2`, `xacro`, `teleop_twist_keyboard`
- Missing: `robot_localization`, `realsense2_description`, `velodyne_description`, `joint_state_publisher`, `joint_state_publisher_gui`

`robot_localization` is required by `unitree_go2_launch.py`. `realsense2_description` is required by `unitree_go2_robot.xacro` through `realsense_d455.xacro`.
The local `quadruped_bringup` bridge declares `robot_localization` so future `rosdep install --from-paths src --ignore-src -r -y` runs can install it, even though the upstream `unitree_go2_sim/package.xml` does not currently declare it.

## Build Notes

Initial `rosdep update` failed because rosdep has not been initialized on this machine:

```bash
sudo rosdep init
rosdep update
```

Passwordless sudo was not available in this session, so dependency installation could not be completed here.

The first `colcon build --symlink-install` failed on a stale generated build artifact in `build/quadruped_msgs`. Cleaning only generated outputs for that package fixed the issue:

```bash
rm -rf ~/quad_ws/build/quadruped_msgs ~/quad_ws/install/quadruped_msgs
```

After that cleanup, the workspace built successfully:

```bash
cd ~/quad_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Verification Status

Completed:

- `colcon build --symlink-install` succeeded for all 13 packages.
- `ros2 launch unitree_go2_sim unitree_go2_launch.py --show-args` succeeded.
- `ros2 launch quadruped_bringup go2_reference.launch.py --show-args` succeeded.

Blocked until runtime dependencies are installed:

- Xacro expansion of `unitree_go2_robot.xacro` fails because `realsense2_description` is not installed.
- Starting the full Gazebo/RViz launch was not attempted because it would fail during robot description expansion and/or EKF node startup.

## Launch Commands

Gazebo Harmonic simulation without RViz:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
```

Gazebo Harmonic simulation with RViz:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=true
```

Workspace bridge launch:

```bash
ros2 launch quadruped_bringup go2_reference.launch.py
```

The cloned repository does not currently provide a standalone RViz-only or robot_state_publisher-only launch file. Its RViz entrypoint is the simulation launch with `rviz:=true`.

## Helper Scripts

- `~/quad_ws/scripts/build_quad_ws.sh`
- `~/quad_ws/scripts/launch_go2_gazebo.sh`
- `~/quad_ws/scripts/launch_go2_rviz.sh`

## Next Recommended Test

After initializing rosdep and installing the missing dependencies:

```bash
cd ~/quad_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch quadruped_bringup go2_reference.launch.py rviz:=false
```
