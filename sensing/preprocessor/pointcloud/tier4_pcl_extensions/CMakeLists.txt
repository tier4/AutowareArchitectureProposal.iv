cmake_minimum_required(VERSION 3.0.2)
project(tier4_pcl_extensions)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(tier4_pcl_extensions src/voxel_grid_nearest_centroid.cpp)

target_link_libraries(tier4_pcl_extensions ${catkin_LIBRARIES})

install(
  TARGETS
  tier4_pcl_extensions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
