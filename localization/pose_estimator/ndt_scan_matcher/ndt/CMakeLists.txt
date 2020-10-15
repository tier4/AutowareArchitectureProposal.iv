cmake_minimum_required(VERSION 3.5)
project(ndt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# message generation
ament_auto_generate_code()


# setup targets
ament_auto_add_library(ndt
  src/base.cpp
  src/pcl_generic.cpp
  src/pcl_modified.cpp
  src/omp.cpp
)


# export information to downstream packages
ament_auto_package()
