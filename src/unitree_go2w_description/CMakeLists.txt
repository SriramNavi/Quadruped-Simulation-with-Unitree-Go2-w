cmake_minimum_required(VERSION 3.8)
project(unitree_go2w_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  dae
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(PROGRAMS
  scripts/go2w_zero_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
