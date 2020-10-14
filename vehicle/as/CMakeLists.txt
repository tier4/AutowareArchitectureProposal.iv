cmake_minimum_required(VERSION 3.5)
project(as)
add_compile_options(-std=c++14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


ament_auto_add_executable(ssc_interface
  src/ssc_interface/ssc_interface_node.cpp
  src/ssc_interface/ssc_interface.cpp
)

ament_auto_add_executable(pacmod_interface
  src/pacmod_interface/pacmod_interface.cpp
  src/pacmod_interface/pacmod_interface_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
