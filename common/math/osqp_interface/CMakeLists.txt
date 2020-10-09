cmake_minimum_required(VERSION 3.5)
project(osqp_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Ofast)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${osqp_INCLUDE_DIR}
  )

ament_auto_add_library(osqp_interface src/osqp_interface.cpp src/csc_matrix_conv.cpp)
target_link_libraries(osqp_interface osqp::osqpstatic)

ament_auto_package()
