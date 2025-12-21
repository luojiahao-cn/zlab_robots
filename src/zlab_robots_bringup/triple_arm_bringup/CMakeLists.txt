cmake_minimum_required(VERSION 3.0.2)
project(triple_arm_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)