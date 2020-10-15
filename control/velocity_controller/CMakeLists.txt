cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


ament_auto_add_executable(velocity_controller
  src/velocity_controller_core.cpp
  src/velocity_controller_node.cpp
  src/pid.cpp
  src/velocity_controller_mathutils.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
