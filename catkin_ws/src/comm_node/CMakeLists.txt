################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(comm_node)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/comm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
