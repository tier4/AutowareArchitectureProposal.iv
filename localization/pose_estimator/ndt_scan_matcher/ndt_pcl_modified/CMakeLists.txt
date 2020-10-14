cmake_minimum_required(VERSION 3.5)
project(ndt_pcl_modified)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED) 
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(ndt_pcl_modified
  src/ndt.cpp
)

target_include_directories(ndt_pcl_modified
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ndt_pcl_modified PCL)

ament_auto_package()
