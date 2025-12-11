cmake_minimum_required(VERSION 3.0.2)
project(frcobot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  apriltag_ros
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    moveit_ros_planning_interface
    apriltag_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(dual_arm_calibrator src/DualArmCalibrator.cpp)

# 添加依赖项
target_link_libraries(dual_arm_calibrator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 添加编译依赖
add_dependencies(dual_arm_calibrator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装目标
install(TARGETS dual_arm_calibrator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)