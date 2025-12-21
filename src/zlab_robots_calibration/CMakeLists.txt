cmake_minimum_required(VERSION 3.0.2)
project(zlab_robots_calibration)

## Use C++14
# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  image_transport
  cv_bridge
  apriltag_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2
    image_transport
    cv_bridge
    apriltag_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibration_node
  src/calibration_node.cpp
)

target_link_libraries(calibration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(calibration_node
  ${catkin_EXPORTED_TARGETS}
)
