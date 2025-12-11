cmake_minimum_required(VERSION 3.0.2)
project(frcobot_tools)

find_package(catkin REQUIRED)

catkin_package()

# 安装工具包目录
install(DIRECTORY tools/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tools
  USE_SOURCE_PERMISSIONS
)

# 安装工具管理器脚本
install(PROGRAMS scripts/list_tools.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

