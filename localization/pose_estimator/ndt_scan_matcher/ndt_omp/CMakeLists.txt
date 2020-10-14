cmake_minimum_required(VERSION 3.5)
project(ndt_omp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# warn performance issue of ndt_omp which is NOT built in release mode
if (NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Release"))
  message(WARNING "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} may cause a serious performance degradation. Please configure with -DCMAKE_BUILD_TYPE=Release.")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

add_library(ndt_omp
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
)

target_include_directories(ndt_omp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# pcl 1.7 causes a segfault without -O2 (or -O3) flag
target_compile_options(ndt_omp
  PUBLIC
    $<$<CONFIG:Debug>:-O2 -g>
  PRIVATE
    # -mavx causes a lot of errors!!
    -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2
)

set(NDT_OMP_DEPENDENCIES
  PCL
  OpenMP
)

ament_target_dependencies(ndt_omp ${NDT_OMP_DEPENDENCIES}) 


add_executable(align
  apps/align.cpp
)

target_link_libraries(align ndt_omp ${PCL_LIBRARIES})

ament_auto_package()
