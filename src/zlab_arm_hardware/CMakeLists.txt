cmake_minimum_required(VERSION 3.0.2)
project(zlab_arm_hardware)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  urdf
  joint_limits_interface
  transmission_interface
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/zlab_arm_hw_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(zlab_arm_hw_main src/zlab_arm_hw_main.cpp)
add_dependencies(zlab_arm_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(zlab_arm_hw_main ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} zlab_arm_hw_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

