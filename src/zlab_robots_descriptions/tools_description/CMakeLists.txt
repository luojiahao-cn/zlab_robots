cmake_minimum_required(VERSION 3.0.2)
project(tools_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch meshes urdf tools)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()