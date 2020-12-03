cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(velocity_controller_SRC
  src/velocity_controller_core.cpp
  src/velocity_controller_node.cpp
  src/pid.cpp
  src/velocity_controller_mathutils.cpp
)

set(velocity_controller_HEADER
  include/velocity_controller/lowpass_filter.hpp
  include/velocity_controller/pid.hpp
  include/velocity_controller/velocity_controller_mathutils.hpp
  include/velocity_controller/velocity_controller.hpp
)

ament_auto_add_executable(velocity_controller
  ${velocity_controller_HEADER}
  ${velocity_controller_SRC}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
