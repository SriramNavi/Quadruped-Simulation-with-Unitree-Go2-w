# Unitree Go2-W Integration

## Source

- Official repo: https://github.com/unitreerobotics/unitree_ros
- Source folder: https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description
- Imported commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- Import method: sparse checkout of `robots/go2w_description`, then copied into `src/unitree_go2w_description`.

## Local Package

- Package name: `unitree_go2w_description`
- Local path: `src/unitree_go2w_description`
- URDF: `src/unitree_go2w_description/urdf/go2w_description.urdf`
- Gazebo control xacro: `src/unitree_go2w_description/urdf/go2w_gazebo_control.urdf.xacro`
- Gazebo controller config: `src/unitree_go2w_description/config/go2w_ros2_control.yaml`
- Meshes: `src/unitree_go2w_description/dae`
- Joint config: `src/unitree_go2w_description/config/joint_names_go2w_description.yaml`
- RViz config: `src/unitree_go2w_description/rviz/go2w.rviz`
- Zero-state fallback publisher: `src/unitree_go2w_description/scripts/go2w_zero_joint_state_publisher.py`

## ROS 2 Conversion

- Replaced ROS 1/catkin metadata with a minimal ROS 2 `ament_cmake` package.
- Installed `config`, `dae`, `rviz`, and `urdf`.
- Installed only ROS 2 Python launch files from `launch`.
- Updated URDF mesh paths from `package://go2w_description/...` to `package://unitree_go2w_description/...`.
- Upstream ROS 1 launch files remain in source for reference but are not the primary ROS 2 launch path.
- If `joint_state_publisher_gui` or `joint_state_publisher` is unavailable, launch files fall back to the local zero-state publisher for visualization only.
- The Gazebo wrapper sets `GZ_SIM_RESOURCE_PATH` so Gazebo Harmonic can resolve installed Go2-W mesh assets.

## Joint Names

Leg joints from the URDF:

- `FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint`
- `FR_hip_joint`, `FR_thigh_joint`, `FR_calf_joint`
- `RL_hip_joint`, `RL_thigh_joint`, `RL_calf_joint`
- `RR_hip_joint`, `RR_thigh_joint`, `RR_calf_joint`

Wheel joints from the URDF:

- `FL_foot_joint`
- `FR_foot_joint`
- `RL_foot_joint`
- `RR_foot_joint`

Note: the upstream YAML lists `Fl_thigh_joint`, but the URDF uses `FL_thigh_joint`. Treat the URDF spelling as authoritative until the controller config is formalized.

## Gazebo Manual Control

The original upstream URDF is preserved. Gazebo manual control uses the wrapper xacro:

- `src/unitree_go2w_description/urdf/go2w_gazebo_control.urdf.xacro`

Controllers from `src/unitree_go2w_description/config/go2w_ros2_control.yaml`:

- `joint_state_broadcaster`
- `go2w_wheel_velocity_controller`
- `go2w_leg_position_controller`

Wheel velocity controller joints:

- `FL_foot_joint`
- `FR_foot_joint`
- `RL_foot_joint`
- `RR_foot_joint`

Command topic:

- `/go2w_wheel_velocity_controller/commands`
- Message type: `std_msgs/msg/Float64MultiArray`
- Joint order: `FL_foot_joint`, `FR_foot_joint`, `RL_foot_joint`, `RR_foot_joint`

The leg controller claims the 12 hip/thigh/calf position interfaces and holds the xacro initial stance. The wheel controller is active but receives no command until a user publishes one.

## Velocity Teleop Architecture

Go2-W teleop preserves the standard body-velocity command path:

```text
go2w_keyboard_cmd_vel -> /cmd_vel -> go2w_cmd_vel_to_wheels
                                            |
                                            +-> /go2w_wheel_velocity_controller/commands
go2w_keyboard_cmd_vel -> /go2w/emergency_stop -+
```

The old mapping directly used `left = linear.x - angular.z` and
`right = linear.x + angular.z`. It mixed values with different units, omitted wheel
geometry, and clamped the wheels independently. Large combined inputs could therefore
stop one side while driving the other side aggressively, causing abrupt skid pivots and
roll instability.

### Geometry and Units

The controller uses SI units throughout:

- `Twist.linear.x`: metres per second
- `Twist.angular.z`: radians per second
- wheel controller output: radians per second
- geometry dimensions: metres

The local upstream Go2-W URDF uses the same wheel DAE mesh for visual and collision
geometry. The mesh has a maximum rolling-plane radius of `0.0860003 m`, giving the
configured `wheel_radius_m = 0.086 m`.

In the neutral leg pose, each wheel joint chain is `0.0465 + 0.0955 = 0.142 m`
from the base centreline. The centre of the wheel tread in the mesh is another
`0.048092 m` outboard. The nominal tread-centre track is therefore:

```text
track_width_m = 2 * (0.0465 + 0.0955 + 0.048092) = 0.380184 m
```

The configured rounded value is `track_width_m = 0.3802 m`. Both geometry values are
ROS parameters and can be tuned without modifying the imported description.

### Differential-Drive Conversion

For body velocity `v`, yaw rate `omega`, wheel radius `r`, and track width `T`:

```text
v_left     = v - omega * T / 2
v_right    = v + omega * T / 2
omega_left = v_left / r
omega_right = v_right / r

[FL, FR, RL, RR] = [omega_left, omega_right, omega_left, omega_right]
```

If either wheel target exceeds `max_wheel_angular_velocity_radps`, both targets are
multiplied by the same scale factor. This preserves the left/right ratio and requested
turning curvature.

### Command Shaping and Safety

The drive controller runs at `control_rate_hz` and uses elapsed ROS clock time, not an
assumed fixed period. Each cycle:

1. Rate-limits body linear velocity, using separate acceleration and deceleration limits.
2. Rate-limits body angular velocity.
3. Applies speed-dependent steering and minimum-turning-radius limits.
4. Converts the body command using the differential-drive equations.
5. Applies curvature-preserving wheel saturation.
6. Rate-limits the final left and right wheel commands and publishes `[FL, FR, RL, RR]`.

The allowed yaw rate interpolates from
`max_angular_velocity_at_zero_speed_radps` to
`max_angular_velocity_at_max_speed_radps` as absolute linear speed increases. Above
`pivot_speed_threshold_mps`, it is also limited to `abs(v) / minimum_turning_radius_m`.
Near zero speed, pivot commands are disabled when `allow_pivot_turn` is false; otherwise
they are capped by `pivot_turn_max_radps`.

`/go2w/emergency_stop` uses `std_msgs/msg/Bool`. A true value immediately resets all
body and wheel state and continuously publishes exact zeros while ignoring `/cmd_vel`.
A false value clears the latch but also resets every target, so a new movement command is
required. Emergency stop bypasses all rate limits; a normal stop remains rate-limited.

If `/cmd_vel` is stale for `command_timeout_sec`, the target is set to zero and the robot
decelerates under the configured body and wheel limits. The keyboard republishes its
active target at 10 Hz while alive.

### Conservative Defaults

The parameter file is
`src/quadruped_control/config/go2w_drive_controller.yaml`. Important defaults are:

- body velocity: `+1.5 m/s` forward, `-1.0 m/s` reverse, `1.5 rad/s` global yaw
- wheel velocity: `10.0 rad/s`
- body acceleration/deceleration: `0.8 / 1.2 m/s^2`
- angular acceleration: `1.0 rad/s^2`
- wheel acceleration: `8.0 rad/s^2`
- minimum turning radius: `0.75 m`
- dynamic yaw endpoints: `1.0 rad/s` at zero speed and `0.35 rad/s` at maximum speed
- pivot: enabled, `0.5 rad/s` cap below `0.10 m/s`
- deadman timeout: `0.5 s`
- controller rate: `50 Hz`
- deadband: `0.01`

The imported Gazebo control xacro currently declares a further `+/-4.0 rad/s` command
interface limit. The drive node's independent `10.0 rad/s` safety limit remains useful
for other control backends, but the simulation interface can impose the lower limit.

### Keyboard Teleop

- Package/executable: `quadruped_control/go2w_keyboard_cmd_vel`
- Publishes: `geometry_msgs/msg/Twist` on `/cmd_vel`
- Publishes: `std_msgs/msg/Bool` on `/go2w/emergency_stop`
- Displays body targets in physical units plus the final shaped and saturated wheel target
- Refreshes status only after input/state changes; the 10 Hz republish does not print

Keyboard controls:

- `W`/Up: increase target forward velocity
- `S`/Down: decrease target velocity toward reverse
- `A`/Left: increase left steering/yaw target
- `D`/Right: increase right steering/yaw target
- `E` / `Q`: increase / decrease the linear increment
- `Shift+E` / `Shift+Q`: increase / decrease the angular increment
- `C`: recenter steering without changing linear velocity
- `X`: set both targets to zero for controlled deceleration
- `Space`: emergency stop immediately
- `R`: clear emergency stop and reset both targets to zero
- `H`: help
- `Z`: emergency stop, then exit cleanly

The keyboard applies the same steering and radius constraints as the drive node before
publishing. This prevents independently accumulated targets from forming unrealistic
full-speed/full-yaw combinations.

## Commands

Build:

```bash
cd ~/quad_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

RViz:

```bash
ros2 launch unitree_go2w_description view_go2w.launch.py rviz:=true gui:=true
```

Gazebo Harmonic manual-control spawn:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py rviz:=false gazebo:=true manual_mode:=true use_reference_controller:=false
```

Inspect Gazebo controllers:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep controller
```

Short manual wheel-drive test:

```bash
./scripts/go2w_drive_forward_test.sh
```

Velocity teleop:

```bash
ros2 launch quadruped_bringup go2w_teleop.launch.py
```

Optional launch overrides:

```bash
ros2 launch quadruped_bringup go2w_teleop.launch.py \
  max_linear_velocity_mps:=1.0 \
  max_reverse_velocity_mps:=0.7 \
  max_wheel_angular_velocity_radps:=8.0 \
  minimum_turning_radius_m:=1.0 \
  allow_pivot_turn:=false
```

Helper script:

```bash
./scripts/run_go2w_teleop.sh
```

Runtime inspection:

```bash
./scripts/inspect_go2w_runtime.sh
```

## Current Limitations

- Manual Gazebo mode has minimal `gz_ros2_control` support only.
- No autonomous locomotion or reference controller is launched.
- Velocity teleop is open-loop wheel velocity control; it is not odometry- or stability-aware.
- The controller setup is intended for cautious Gazebo bringup, not validated real-robot control.
- Wheel direction is based on the URDF joint axes and may need sign tuning after visual testing.
- Track width is nominal for the neutral leg pose; changing hip roll changes wheel spacing.
- The Go2-W has no mechanically steered wheels, so physical turning still relies on
  tyre-ground slip. This implementation makes skid steering smooth and controlled rather
  than eliminating slip completely.
