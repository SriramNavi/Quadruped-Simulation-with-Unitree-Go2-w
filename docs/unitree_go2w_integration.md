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

The imported wheel DAE remains the visual geometry. Contact uses a measured cylinder
with radius `0.086 m`, length `0.052 m`, and tread centre `0.048092 m` outboard.
This removes mesh-facet contact noise while preserving the rendered model. The source
mesh has a maximum rolling-plane radius of `0.0860003 m`.

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

## Go2-W Slope Stability Test World

The dedicated world is stored at
`src/quadruped_bringup/worlds/go2w_slope_test.sdf` and installed as
`share/quadruped_bringup/worlds/go2w_slope_test.sdf`. It is the default world for
`go2w_reference.launch.py`.

All ramps are 5.0 m long, 2.5 m wide, and 0.12 m thick. Each has a flat 2.0 x
2.5 x 0.12 m top platform with a 0.02 m overlap at the ramp endpoint. Ramp and
platform contact surfaces use `mu=1.0` and `mu2=1.0`. The visual-only approach
centre lines have no collision geometry.

| Angle | Lane centre y | Colour |
| --- | ---: | --- |
| 15 degrees | -6.0 m | Blue |
| 30 degrees | -2.0 m | Green |
| 35 degrees | 2.0 m | Orange |
| 45 degrees | 6.0 m | Red |

The default spawn pose is `x=-3.0`, `y=-6.0`, `z=0.45`, `yaw=0.0`, aligned
with the 15-degree lane and facing world +X. Launch the default slope world with:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py \
  rviz:=false gazebo:=true gui:=true \
  manual_mode:=true use_reference_controller:=false
```

Select the initial lane with `spawn_y`:

```bash
# 15 degrees
ros2 launch quadruped_bringup go2w_reference.launch.py spawn_y:=-6.0 \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false

# 30 degrees
ros2 launch quadruped_bringup go2w_reference.launch.py spawn_y:=-2.0 \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false

# 35 degrees
ros2 launch quadruped_bringup go2w_reference.launch.py spawn_y:=2.0 \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false

# 45 degrees
ros2 launch quadruped_bringup go2w_reference.launch.py spawn_y:=6.0 \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false
```

Launch the previous Gazebo Harmonic empty world or a custom absolute SDF path with:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py world_file:=empty.sdf \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false

ros2 launch quadruped_bringup go2w_reference.launch.py \
  world_file:=/absolute/path/to/custom.sdf \
  rviz:=false gazebo:=true gui:=true manual_mode:=true use_reference_controller:=false
```

Validate the installed SDF with:

```bash
gz sdf -k \
  "$(ros2 pkg prefix quadruped_bringup)/share/quadruped_bringup/worlds/go2w_slope_test.sdf"
```

For repeatable manual trials:

1. Start from the same spawn distance.
2. Align the robot with the ramp centre.
3. Reset the simulation between trials.
4. Use the same leg posture.
5. Use the same acceleration limits.
6. Use the same commanded approach speed.
7. Test lower angles before higher angles.
8. Record whether the robot climbs successfully, wheel-slips, loses heading,
   touches its body on the ramp, rolls or pitches beyond recovery, or topples.
9. Repeat each angle several times; one run is not a final stability threshold.

The world remains a simulation-specific comparison. It does not automate motion or
resets. Results depend on the current Go2-W collision model, controller posture,
tyre contact model, physics step size, and commanded approach profile. The
open-ended platforms have no rails and therefore do not prevent a robot from
falling after reaching the top.

## Quantitative Slope Stability Testing

`go2w_slope_test_monitor` retains a passive measurement mode. With the default
`auto_drive_enabled:=false`, the operator starts any desired manual command source
independently and the monitor never publishes motion or emergency-stop commands.
The monitor launch never starts Gazebo or keyboard teleop.

The monitor reads `/joint_states`, `/cmd_vel`,
`/go2w_wheel_velocity_controller/commands`, `/go2w/emergency_stop`, `/imu/data`,
and normalized Gazebo ground truth on `/go2w/ground_truth/odom`. Automated trials
require IMU readiness; passive monitoring can still start before IMU data arrives.
The monitoring launch bridges the verified Gazebo Harmonic
`/world/go2w_slope_test/dynamic_pose/info` Pose_V stream to a timestamped
`PoseArray`; a small adapter selects the verified model pose at index zero, derives
body-frame velocity using elapsed simulation time, and publishes Odometry. It does
not publish TF.

Outputs are `/go2w/slope_test/state` (String), `/go2w/slope_test/toppled` (Bool),
`/go2w/slope_test/orientation_deg` (Vector3Stamped), and
`/go2w/slope_test/status` (String). A 20 Hz CSV records pose, orientation, measured
twist, command observations, optional IMU fields, all 16 joints by name, and event
flags. Missing IMU or effort feedback remains `NaN`. A matching JSON file records
the terminal result and extrema.

Toppling requires roll, pitch, low-base, or body-up-alignment criteria to persist
for `topple_hold_sec`; the default high pitch limit does not classify normal steep
ramp pitch as a fall. Success requires the base to remain upright, within the lane,
above the platform top and inside its SDF-derived inner x region for
`success_hold_sec`. The safe region excludes 0.25 m at each platform end.
Slide-back is reported separately and is not itself a topple result.

Start the simulator without teleop, then start a 15-degree monitoring trial:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py \
  rviz:=false gazebo:=true gui:=true manual_mode:=true

ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  ramp_angle_deg:=15 lane_y:=-6.0 trial_id:=15deg_run01 \
  target_speed_mps:=0.25
```

For another world name, override `gazebo_pose_topic` with that world's verified
`dynamic_pose/info` topic. The helper `./scripts/run_go2w_slope_monitor.sh 30 -2.0
30deg_run01 0.25` starts only monitoring. CSV and JSON results are written under
`~/quad_ws/logs/slope_tests` by default. Ctrl+C closes the CSV and records ABORTED
when no terminal result has already been reached.

Reset the simulation and keep spawn pose, leg posture, controller limits, speed,
and physics settings identical between trials. Run at least three trials per angle
and compare the summaries; a single trial does not establish a reliable maximum
slope. This feature is monitoring and classification only, not active body
stabilization or posture correction.

## Automated Slope Test Runner

Set `auto_drive_enabled:=true` to make the same monitor own one deterministic
straight-line trial. Its phases are `INITIALIZING`, `PRECHECK`, `COUNTDOWN`,
`ACCELERATING`, `CRUISING`, `DECELERATING`, `STOPPING`, and `COMPLETE`; result
states remain independent (`READY`, `RUNNING`, `WARNING`, `TOPPLING`, `TOPPLED`,
`SUCCESS`, `TIMEOUT`, or `ABORTED`).

Before clearing emergency stop and beginning the countdown, the runner requires
fresh odometry and joint states, a `/cmd_vel` consumer, an unasserted emergency
stop, the configured start position and +X heading, an upright body, and a
stationary base. A failed precheck holds a zero command and times out without
moving. `expected_start_y_m` defaults to the selected ramp lane.

The runner publishes only straight `geometry_msgs/msg/Twist` commands on
`/cmd_vel`; every angular component is zero. Speed increases using
`acceleration_mps2 * dt` and decreases using `deceleration_mps2 * dt`, where `dt`
is actual ROS/simulation time. It never publishes wheel-controller commands.
Deceleration begins at the earlier of the inner platform entry and
`platform_end_x - stop_distance_before_platform_end_m - v^2/(2a)`. Success is
declared only after the base is upright and stationary in the success zone for
`success_hold_sec`.

Topple criteria, timeout, slide-back, lane or heading deviation, stale state data,
loss of the command consumer, an external emergency stop, Ctrl+C, and internal
errors force an immediate zero Twist. Emergency failures also publish `true` on
`/go2w/emergency_stop`; zero and true are repeated through `stop_hold_sec`.
After a safe precheck the runner publishes `false` once to intentionally clear the
stop for that trial, and shutdown publishes zero then true. Clear the emergency
stop intentionally before the next trial.

CSV rows retain the passive fields and add the automation phase, precheck status,
generated command, freshness, subscriber count, lane/heading errors, abort reason,
and braking geometry. JSON summaries add the expected/actual start pose, final
pose, motion profile statistics, result phase, data timeouts, and abort reason.
Both files are written to `~/quad_ws/logs/slope_tests` unless `log_directory` is
overridden.

Start Gazebo and the drive converter separately, reset the robot to the configured
start pose, then run an automated 15-degree trial:

```bash
ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  ramp_angle_deg:=15 lane_y:=-6.0 trial_id:=15deg_run01 \
  auto_drive_enabled:=true target_speed_mps:=0.25 \
  acceleration_mps2:=0.20 deceleration_mps2:=0.35
```

Passive monitoring remains available with:

```bash
ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  ramp_angle_deg:=15 lane_y:=-6.0 trial_id:=15deg_passive01 \
  auto_drive_enabled:=false target_speed_mps:=0.25
```

**Do not launch keyboard teleop while `auto_drive_enabled` is true. Two
`/cmd_vel` publishers can conflict.** Reset Gazebo between trials, and run at
least three trials per angle before drawing conclusions.

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

## Effort State Feedback

Effort was previously `.nan` because `gz_ros2_control` exported only position and
velocity state interfaces. The URDF joint `effort` limits constrain actuators; they
are not live feedback. The Gazebo control xacro now declares an effort state
interface for all 12 leg joints and four wheel joints.

Build with `colcon build --symlink-install --packages-select
unitree_go2w_description quadruped_bringup`. Verify with `ros2 control
list_hardware_interfaces`, `ros2 topic echo /dynamic_joint_states --once`, and
`ros2 topic echo /joint_states --once`.

With Gazebo Harmonic and the current `gz_ros2_control` plugin, all 16 effort
interfaces were exported, all dynamic joint states contained position, velocity,
and effort, and `/joint_states.effort` contained finite simulated values. These
values are physics-engine joint feedback and are not validated actuator-torque
measurements.

## Explicit Wheel and Terrain Contact

The previous simulation was not necessarily frictionless: Gazebo supplied implicit
contact defaults where the robot had no explicit wheel parameters. The current model
makes wheel and terrain behaviour reproducible. Gazebo Harmonic (Sim 8) loads Gazebo
Physics 7's default DART 6.13 engine. DART reads the SDF `ode` fields `mu`, `mu2`,
`fdir1`, `slip1`, and `slip2`; the naming is an SDFormat compatibility convention and
does not mean that Gazebo Classic or the ODE engine is active.

The wheel-local +X direction is longitudinal, and +Y is the wheel axis. The collision
cylinder is rotated by `rpy="pi/2 0 0"`, so its cylinder axis aligns with the joint's
`0 1 0` axis. DART combines two contacting surfaces using the smaller coefficient in
each direction and adds their force-dependent slip compliances. Slip compliance has
units `m/s/N`. The dry-concrete defaults are wheel `mu/mu2=1.2/1.0`, terrain
`1.0/1.0`, and wheel `slip1/slip2=0.0001/0.0001 m/s/N`.

DART's rigid-contact path does not consume ODE `kp` or `kd`; the requested
`1.0e6 N/m` stiffness and `100 N·s/m` damping values are retained only in experiment
metadata for reproducibility and possible future ODE-compatible use. They are not
claimed as active DART parameters.

Presets live in the read-only authoritative file
`unitree_go2w_description/config/go2w_friction_presets.yaml`. The runtime path is:

```text
preset YAML -> launch-time resolution -> optional overrides
            -> unique /tmp robot.sdf + world.sdf + effective_friction.yaml
            -> Gazebo
```

The preset is resolved first and each non-empty terminal override is applied second.
An explicit `0.0` is an override, not an empty value. Values must be finite and
non-negative. Unknown presets and malformed values fail before Gazebo starts.
The source URDF, Xacro, world, preset YAML, and launch files are never rewritten.
The unique runtime directory is removed on normal launch shutdown; a directory left
after a crash cannot collide with a later launch.

Inspect the installed authoritative configuration with:

```bash
ros2 run quadruped_control go2w_friction_presets list
ros2 run quadruped_control go2w_friction_presets show dry_concrete
ros2 run quadruped_control go2w_friction_presets validate
ros2 run quadruped_control go2w_friction_presets path
```

Select any named condition without editing a file:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=dry_concrete
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=smooth_tile
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=wet_tile
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=loose_soil
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=low_friction
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=zero_friction

ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=dry_concrete \
  wheel_mu_longitudinal:=0.35 wheel_mu_lateral:=0.30 \
  terrain_mu_longitudinal:=0.35 terrain_mu_lateral:=0.30
```

The snapshot records the requested preset, effective label, every effective contact
value, fixed wheel collision dimensions, DART, ramp angle, generated paths, and run
identifier. CSV rows and summary JSON receive the same resolved values. In the
automated runner, `EXTRA_LAUNCH_ARGS` forwards friction overrides to both simulation
and logger. A supplied output directory must not already exist:

```bash
EXTRA_LAUNCH_ARGS="wheel_mu_longitudinal:=0.35 terrain_mu_longitudinal:=0.35" \
GUI=false ./scripts/run_go2w_friction_experiment.sh \
  dry_concrete logs/custom 65 3 0.25 custom_01
```

To verify immutability independently, hash the authoritative files and compare
`git status --short` before and after a bounded launch. The automated contact
generation test performs this check for dry, zero, custom, and explicit-zero cases.

### Friction-aware logging and plots

The existing slope monitor now logs forward-positive measured wheel angular velocity,
tread velocity, and the observable rigid-body slip ratio

```text
(r * omega_forward - base_vx) / max(|r * omega_forward|, |base_vx|, epsilon)
```

Zero is near pure rolling, positive means wheel spin, and negative means the wheel is
slower than the body (braking/skidding). This is not an exact deformable-tyre metric.
All current wheel axes use the same forward sign; four sign parameters are available
if the URDF changes. Default output is written under
`~/quad_ws/logs/slope_15deg/<preset>/` without overwriting older logs.

```bash
ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  friction_preset:=dry_concrete trial_id:=trial_01

ros2 run quadruped_control go2w_friction_plots single RUN.csv \
  --output-directory plots/dry_concrete

ros2 run quadruped_control go2w_friction_plots compare RUN1.csv RUN2.csv \
  --output-directory plots/comparison
```

For a bounded automated baseline (Gazebo, controller readiness checks, drive converter,
logger, graceful stop), run dry concrete manually before attempting a matrix:

```bash
./scripts/run_go2w_friction_experiment.sh dry_concrete '' 60 3 0.25 trial_01
./scripts/run_go2w_friction_experiment.sh smooth_tile logs/smooth 65 3 0.25 smooth_01
./scripts/run_go2w_friction_experiment.sh wet_tile logs/wet 65 3 0.25 wet_01
./scripts/run_go2w_friction_experiment.sh loose_soil logs/soil 65 3 0.25 soil_01
./scripts/run_go2w_friction_experiment.sh low_friction logs/low 65 3 0.25 low_01
./scripts/run_go2w_friction_experiment.sh zero_friction logs/zero 65 3 0.25 zero_01
```

An explicit all-zero override is accepted independently of the named zero preset:

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py \
  friction_preset:=dry_concrete \
  wheel_mu_longitudinal:=0.0 wheel_mu_lateral:=0.0 \
  terrain_mu_longitudinal:=0.0 terrain_mu_lateral:=0.0 \
  wheel_slip_longitudinal:=0.0 wheel_slip_lateral:=0.0
```

The summary derives climb success from the existing platform pose/height/upright state
condition, not elapsed time. Rigid Coulomb contact cannot reproduce tyre deformation or
deformable mud, sand, grass, and loose-soil terramechanics.

### Restored Gazebo IMU path

The friction-aware pipeline originally rendered `go2w_gazebo_control.urdf.xacro`
without the IMU sensor and `go2w_reference.launch.py` bridged only `/clock`. The
earlier Unitree Go2 launch worked because its Gazebo Xacro defined `imu_sensor` and
its parameter bridge included `/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU`.

The restored sensor is defined on link `imu` in
`unitree_go2w_description/urdf/go2w_gazebo_control.urdf.xacro`. It publishes
Gazebo Transport `/imu/data` at 100 Hz with frame `imu`; the reference launch
bridges that stream to ROS `/imu/data`. The previous gyro and accelerometer noise
settings are preserved exactly. Automated monitoring now requires a finite,
normalized, timestamp-increasing IMU sample before motion and aborts after a
0.5 s IMU timeout. Orientation covariance remains zero because Gazebo does not
populate it; gyro and accelerometer covariances reflect the configured noise.

```bash
ros2 launch quadruped_bringup go2w_reference.launch.py \
  rviz:=false gui:=false friction_preset:=dry_concrete
ros2 launch quadruped_bringup go2w_slope_monitor.launch.py \
  ramp_angle_deg:=15 lane_y:=-6.0 friction_preset:=dry_concrete \
  trial_id:=dry_concrete_imu_restored_01
GUI=false ./scripts/run_go2w_friction_experiment.sh \
  dry_concrete '' 65 3 0.25 dry_concrete_imu_restored_01
ros2 run quadruped_control go2w_friction_plots single RUN.csv \
  --output-directory plots/dry_concrete_imu_restored_01
```

The 2026-07-11 explicit dry-concrete rerun succeeded in 41.40 s. Its 100 Hz IMU
stream and all active-trial slip rows were valid; ramp mean absolute slip was
0.0205 and peak absolute ramp slip was 0.3058. The earlier comparison baseline
used implicit/default contact, not zero friction; this rerun uses explicit
dry-concrete wheel/terrain friction plus IMU and slip logging.

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
- DART rigid contact does not apply the logged stiffness/damping metadata.
- The cylinder represents the overall measured wheel envelope, not tread-block or tyre
  deformation geometry.
- SDFormat lumps the existing zero-mass fixed `imu` link into `base` internally;
  the sensor remains defined against `imu` and publishes frame ID `imu`.
- Gazebo supplies zero orientation covariance for this simulated IMU.

## IMU pitch leveling with analytical IK

`go2w_imu_ik_controller` is a standalone outer-loop pitch controller. It filters
the Gazebo `/imu/data` pitch and evaluates
`u = kp * (target - pitch) + ki * integral - kd * angular_velocity_y`. Positive
correction raises the front wheel targets and lowers the rear targets in the body
frame. The four resulting targets are solved by analytical three-DOF leg IK using
the URDF dimensions (0.0955 m hip offset, 0.213 m thigh, and 0.2264 m to the wheel
centre).

The selected backend publishes the twelve leg positions directly to
`/go2w_leg_position_controller/commands`; it never commands wheel joints. This is
safe only because `go2w_reference.launch.py` does not launch CHAMP or another leg
command publisher. Active mode refuses to start, and faults, if the position
controller is absent or another publisher appears. Monitor and shadow modes never
send commands. Shadow is the default.

Terminal 1 runs the world and visualization tools only:

```bash
ros2 launch quadruped_bringup go2w_imu_ik_world.launch.py \
  gui:=true rviz:=false friction_preset:=dry_concrete spawn_on_ramp:=true \
  launch_rqt_graph:=true launch_rqt_plot:=true
```

Terminal 2 runs the independently controlled node:

```bash
CONFIG=$(ros2 pkg prefix quadruped_control)/share/quadruped_control/config/go2w_imu_ik_controller.yaml
ros2 run quadruped_control go2w_imu_ik_controller --ros-args \
  --params-file "$CONFIG" -p mode:=shadow -p target_pitch_deg:=0.0
ros2 service call /go2w/imu_ik/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /go2w/imu_ik/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /go2w/imu_ik/reset std_srvs/srv/Trigger "{}"
```

`rqt_graph` shows node/topic connectivity, not values. `rqt_plot` is preloaded with
raw, filtered, target, and error pitch plus PID output and front/rear offsets under
`/go2w/imu_ik/...`. Diagnostics report stale input, invalid IMU data, fall limits,
IK failure, saturation timeout, backend loss, and publisher conflicts. A bounded
2026-07-13 static dry-concrete test tuned `kp=7.0`, `ki=0`, `kd=0.15`, a 15 degree
correction limit, and a 0.055 m leg-offset limit. Pitch settled at approximately
−1.874 degrees from approximately −15 degrees without observed IK failure or
saturation. Dynamic-climb behavior has not yet been validated with these gains.
