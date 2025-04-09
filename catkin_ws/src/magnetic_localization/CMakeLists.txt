cmake_minimum_required(VERSION 3.0.2)
project(magnetic_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
  moveit_ros_planning_interface
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MagneticData.msg
  MagnetEstimation.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES magnetic_localization
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs serial
#  DEPENDS system_lib
)

link_directories(/opt/ros/noetic/lib)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)



add_library(magnetic_field_solver
  src/magnetic_field_solver.cpp
)


add_executable(serial_data_node src/serial_data_node.cpp)
add_executable(magnetic_move src/magnetic_move.cpp)
add_executable(magnetic_visualizer_node src/magnetic_visualizer_node.cpp)
add_executable(magnetic_field_solver_node 
  src/magnetic_field_solver_node.cpp
  src/magnetic_field_solver.cpp
)
add_executable(magnetic_solver_node 
  src/magnetic_field_solver_node.cpp
)
add_executable(magnetic_simulation_node 
    src/magnetic_simulation_node.cpp
    src/magnetic_simulation.cpp)


add_dependencies(serial_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(magnetic_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(magnetic_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(magnetic_field_solver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(serial_data_node
  ${catkin_LIBRARIES}
  serial
)
target_link_libraries(magnetic_move
  ${catkin_LIBRARIES}
)
target_link_libraries(magnetic_visualizer_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
target_link_libraries(magnetic_field_solver_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
)
target_link_libraries(magnetic_solver_node
  magnetic_field_solver
  ${catkin_LIBRARIES}
  yaml-cpp
)
target_link_libraries(magnetic_simulation_node
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES})

