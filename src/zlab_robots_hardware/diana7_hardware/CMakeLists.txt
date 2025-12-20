cmake_minimum_required(VERSION 3.0.2)
project(diana7_hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  joint_limits_interface
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diana7_hardware
  CATKIN_DEPENDS roscpp hardware_interface controller_manager joint_limits_interface sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/DianaApi/include
)

link_directories(
  /opt/DianaApi/lib
)

add_library(${PROJECT_NAME}
  src/diana7_hw_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  DianaApi
)

add_executable(diana7_hw_main src/diana7_hw_main.cpp)
target_link_libraries(diana7_hw_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} diana7_hw_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
