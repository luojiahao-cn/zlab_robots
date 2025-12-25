cmake_minimum_required(VERSION 3.0.2)
project(fr5_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  moveit_core
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib moveit_core moveit_ros_planning_interface
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

