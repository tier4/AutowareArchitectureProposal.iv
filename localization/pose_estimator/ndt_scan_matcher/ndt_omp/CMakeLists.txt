cmake_minimum_required(VERSION 3.5)
project(ndt_omp)

# -mavx causes a lot of errors!!
add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# warn performance issue of ndt_omp which is NOT built in release mode
# string(TOUPPER ${CMAKE_BUILD_TYPE} uppercase_CMAKE_BUILD_TYPE)
# if (NOT (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE"))
#   message(WARNING
#     "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} may cause a serious performance degradation. Please configure with -DCMAKE_BUILD_TYPE=Release.")
# endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
)

set(NDT_OMP_DEPENDENCIES
  PCL
  OpenMP
)

# pcl 1.7 causes a segfault without -O2 (or -O3) flag
target_compile_options(ndt_omp PUBLIC
  $<$<CONFIG:Debug>:-O2 -g>
)

ament_target_dependencies(ndt_omp ${NDT_OMP_DEPENDENCIES}) 
ament_export_dependencies(${NDT_OMP_DEPENDENCIES})


add_executable(align
  apps/align.cpp
)

# get_target_property(FRED_PCL_LINK_LIBRARIES PCL::pcl LINK_INTERFACE_LIBRARIES)

# ament_target_dependencies can't depend on targets, i.e. ndt_omp
target_link_libraries(align PUBLIC
  ndt_omp
  ${PCL_LIBRARIES}
)

install(TARGETS ndt_omp
  DESTINATION lib/${PROJECT_NAME}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION install/${PROJECT_NAME}
)

ament_package()