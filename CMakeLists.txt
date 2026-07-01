cmake_minimum_required(VERSION 3.8)
project(test_feetech_r2c_dynamics_limits)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/gripper_command.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    gripper_command
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
