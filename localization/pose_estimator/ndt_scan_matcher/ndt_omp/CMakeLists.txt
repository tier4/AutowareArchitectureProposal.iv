cmake_minimum_required(VERSION 3.5)
project(ndt_omp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration io)
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

target_compile_options(ndt_omp
  PRIVATE
    # -mavx causes a lot of errors!!
    -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2
)

set(NDT_OMP_DEPENDENCIES
  PCL
  OpenMP
)

ament_target_dependencies(ndt_omp ${NDT_OMP_DEPENDENCIES}) 
ament_export_targets(export_ndt_omp HAS_LIBRARY_TARGET)

add_executable(align
  apps/align.cpp
)

# Strangely, for an executable, ament_target_dependencies(align ${NDT_OMP_DEPENDENCIES})
# gives a linker error from PCL
ament_target_dependencies(align PUBLIC OpenMP)
# So we include PCL manually
target_include_directories(align PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(align PUBLIC ${PCL_LIBRARIES})
target_compile_definitions(align PUBLIC ${PCL_DEFINITIONS})
target_link_directories(align PUBLIC ${PCL_LIBRARY_DIRS})
# Also link ndt_omp separately because ament_target_dependencies doesn't support
# depending on a library target from the same package
target_link_libraries(align PUBLIC ndt_omp)

ament_export_dependencies(${NDT_OMP_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ndt_omp
  EXPORT export_ndt_omp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

