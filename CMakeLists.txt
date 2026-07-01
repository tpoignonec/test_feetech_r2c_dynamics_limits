cmake_minimum_required(VERSION 3.8)
project(test_feetech_r2c_dynamics_limits)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(range-v3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# tl::expected is referenced by the imported feetech_driver targets.
find_package(tl-expected QUIET)
if(NOT tl-expected_FOUND)
  find_package(tl_expected REQUIRED)
  if(NOT TARGET tl::expected)
    add_library(tl::expected ALIAS tl_expected::tl_expected)
  endif()
endif()

# libserial include dirs are needed because serial_port.hpp includes
# <libserial/SerialPort.h> (the imported targets don't propagate them).
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

# feetech_driver provides the core libraries (communication_protocol/serial_port);
# it must be found before feetech_ros2_driver, whose config references those
# imported targets. feetech_ros2_driver provides the joint_config.hpp header.
find_package(feetech_driver REQUIRED)
find_package(feetech_ros2_driver REQUIRED)

# The vendored feetech_driver package does not install its headers (SMS_STS.h,
# communication_protocol.hpp, ...), even though its exported config advertises
# ${prefix}/include. To avoid patching the external package, point at the
# in-source headers (sibling package in the colcon workspace).
get_filename_component(_feetech_driver_src_include
  "${CMAKE_CURRENT_SOURCE_DIR}/../external/feetech_ros2_driver/feetech_driver/include"
  ABSOLUTE)

# Standalone single-motor node (configure + activate + open/close demo)
add_executable(single_motor_node src/single_motor_node.cpp)
target_include_directories(single_motor_node SYSTEM PRIVATE
                           ${SERIAL_INCLUDE_DIRS}
                           ${_feetech_driver_src_include})
ament_target_dependencies(single_motor_node rclcpp feetech_ros2_driver range-v3)
target_link_libraries(single_motor_node
                      feetech_driver::communication_protocol
                      feetech_driver::serial_port
                      yaml-cpp spdlog::spdlog fmt::fmt
                      ${SERIAL_LIBRARIES})
install(TARGETS single_motor_node DESTINATION lib/${PROJECT_NAME})

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
