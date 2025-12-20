cmake_minimum_required(VERSION 3.0.2)
project(fr5_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  moveit_core
  moveit_ros_planning_interface
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib moveit_core moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(environment_loader src/environment_loader.cpp)
target_link_libraries(environment_loader ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS environment_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

