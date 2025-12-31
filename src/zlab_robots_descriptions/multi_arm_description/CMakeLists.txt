cmake_minimum_required(VERSION 3.0.2)
project(multi_arm_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zlab_multi_arm_description
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
