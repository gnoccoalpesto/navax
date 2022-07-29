cmake_minimum_required(VERSION 3.0.2)
project(mapping3d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Setpoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/odom_compat_node
    scripts/zed2_imu_transform_broadcaster
    scripts/pdu_node
    scripts/rover_regulator.py
    scripts/rover_supervisor.py
    scripts/navigation_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime
)
