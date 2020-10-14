cmake_minimum_required(VERSION 3.5)
project(ndt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ndt_pcl_modified REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration io)

add_library(ndt
  src/base.cpp
  src/pcl_generic.cpp
  src/pcl_modified.cpp
  src/omp.cpp
)

target_include_directories(ndt
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ndt ndt_omp ndt_pcl_modified PCL)

ament_auto_package()