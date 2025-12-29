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

# Check if DianaAPI.h exists
set(DIANA_API_HEADER "/opt/DianaApi/include/DianaAPI.h")
set(DIANA_API_AVAILABLE FALSE)
if(EXISTS ${DIANA_API_HEADER})
  set(DIANA_API_AVAILABLE TRUE)
else()
  message(WARNING "DianaAPI.h not found at ${DIANA_API_HEADER}. Skipping Diana7 hardware compilation.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(DIANA_API_AVAILABLE)
  include_directories(
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

  add_executable(test_diana_api src/test_diana_api.cpp)
  target_link_libraries(test_diana_api
    ${catkin_LIBRARIES}
    DianaApi
  )

  install(TARGETS ${PROJECT_NAME} diana7_hw_main test_diana_api
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  # Create empty stub library to satisfy catkin_package requirements
  # This allows the package to be built without DianaAPI, but functionality will be disabled
  add_library(${PROJECT_NAME} INTERFACE)
  target_link_libraries(${PROJECT_NAME} INTERFACE
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
