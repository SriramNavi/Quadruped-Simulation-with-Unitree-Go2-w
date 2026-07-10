# quadruped_description

This package keeps robot-agnostic quadruped description assets for the local control architecture.

## Reference Robot Models

Unitree Go2 is the first reference robot model for this workspace. The Go2 description is intentionally not duplicated here. The source of truth is the cloned upstream package:

- `~/quad_ws/src/unitree_go2_ros2/unitree_go2_description`

Use that package for the Go2 URDF/Xacro, meshes, sensors, and Gazebo-specific description files. Local algorithm packages should treat it as a reference robot model while keeping generic quadruped assets in this package.

