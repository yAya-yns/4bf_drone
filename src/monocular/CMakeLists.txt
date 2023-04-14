################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(monocular)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/monocular.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
