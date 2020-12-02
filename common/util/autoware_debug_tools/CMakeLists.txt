cmake_minimum_required(VERSION 3.0.2)
project(autoware_debug_tools)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2
)

catkin_package()

install(
  DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
