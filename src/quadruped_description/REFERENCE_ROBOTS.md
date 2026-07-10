# Reference Robots

- Unitree Go2 remains the legged quadruped reference robot through the existing Go2 packages.
- Unitree Go2-W is the primary wheeled-quadruped reference robot for this workspace.
- The Go2-W description lives in `unitree_go2w_description`.
- Do not duplicate Go2-W URDFs or meshes into `quadruped_description` unless a derived custom robot requires it.
- Our custom wheeled-quadruped robot should later get its own robot config and description assets.
