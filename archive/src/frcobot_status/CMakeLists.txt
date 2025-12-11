cmake_minimum_required(VERSION 3.0.2)
project(frcobot_status)

## 添加message_generation依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## 声明要编译的ROS消息
add_message_files(
  FILES
  status.msg
)

## 生成依赖于消息的头文件
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 设置catkin包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  LIBRARIES ${PROJECT_NAME}
)

## 指定include目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 添加库
add_library(${PROJECT_NAME}
  src/frcobot_status_node.cpp
)

## 添加对生成的消息头文件的依赖
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 添加可执行文件
add_executable(frcobot_status_node src/frcobot_status_node.cpp)

## 为可执行文件添加依赖
add_dependencies(frcobot_status_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库到可执行文件
target_link_libraries(frcobot_status_node
  ${catkin_LIBRARIES}
)

## 查找Qt5组件
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

## 添加GUI可执行文件
add_executable(frcobot_status_gui src/frcobot_status_gui.cpp)

## 将此可执行文件链接到Qt库
target_link_libraries(frcobot_status_gui
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
)

## 为可执行文件添加依赖
add_dependencies(frcobot_status_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 安装可执行文件和库
install(TARGETS frcobot_status_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)