# Quadruped ROS 2 Control Foundation

This workspace contains a modular ROS 2 Jazzy foundation for developing
quadruped control algorithms on a market-available platform first, then porting
the same stack to a custom Surivona quadruped.

The design keeps robot-specific geometry, names, hardware bridges, and simulator
models outside the controller logic wherever possible.

## Packages

| Package | Build type | Responsibility |
| --- | --- | --- |
| `quadruped_description` | `ament_python` | Xacro/URDF, meshes, and robot configuration YAML for `unitree_go2`, `mini_cheetah`, and `custom_surivona_quad`. |
| `quadruped_bringup` | `ament_python` | Top-level launch orchestration for robot description, gait, state estimation, and controllers. |
| `quadruped_control` | `ament_python` | Low-level control nodes: joint PD, Cartesian foot target conversion, gravity compensation placeholder, and torque command abstraction. |
| `quadruped_kinematics` | `ament_python` | Generic 3-DOF leg FK, IK, and Jacobian utilities. |
| `quadruped_gait` | `ament_python` | Gait scheduler, trot/crawl placeholders, and swing trajectory generation. |
| `quadruped_state_estimation` | `ament_python` | IMU/joint-state based base state estimator placeholder. |
| `quadruped_msgs` | `ament_cmake` | Custom ROS interfaces for gait commands, foot targets, robot state, and controller mode. |
| `quadruped_utils` | `ament_python` | Shared math, YAML config loading, and parameter helpers. |

## Initial ROS Graph

```text
/quadruped_gait/command
        |
        v
gait_scheduler ---> /quadruped_control/foot_targets
        |
        v
cartesian_foot_controller ---> /quadruped_control/joint_targets
        |
        v
joint_pd_controller ---> /quadruped_control/torque_commands

/imu, /joint_states ---> base_state_estimator ---> /quadruped/state
```

`TorqueCommandInterface` currently publishes simple ROS topics. When a vendor
SDK, `ros2_control`, or a custom motor bus is added, that boundary is the first
place to adapt hardware-specific commands.

## Robot Configs

Robot-specific placeholders live in:

- `quadruped_description/config/unitree_go2.yaml`
- `quadruped_description/config/mini_cheetah.yaml`
- `quadruped_description/config/custom_surivona_quad.yaml`

Each config records body dimensions, generic 3-DOF leg dimensions, joint names,
and initial controller parameters. These values are scaffolding only; validate
them against vendor documentation, CAD, or measured hardware before closed-loop
use.

## Build

From the workspace root:

```bash
cd ~/quad_ws
colcon build
source install/setup.bash
```

## Useful Launch Commands

Publish the generic robot description:

```bash
ros2 launch quadruped_description robot_state_publisher.launch.py robot_name:=custom_surivona_quad
```

Start the foundational control stack:

```bash
ros2 launch quadruped_bringup bringup.launch.py robot_name:=custom_surivona_quad
```

Run the future simulator integration placeholder:

```bash
ros2 launch quadruped_bringup simulation.launch.py robot_name:=custom_surivona_quad
```

Run the kinematics sanity node:

```bash
ros2 run quadruped_kinematics kinematics_sanity
```

## Next Integration Steps

1. Replace placeholder dimensions with verified values for the first target
   market robot.
2. Add a hardware or simulator adapter behind `TorqueCommandInterface`.
3. Add `ros2_control` controller manager integration when actuator interfaces
   are known.
4. Add Gazebo/Ignition launch and model assets in `quadruped_bringup` and
   `quadruped_description`.
5. Expand tests around IK limits, gait timing, command saturation, and estimator
   message contracts before hardware trials.
