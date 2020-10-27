cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocessor)

add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Boost REQUIRED)
find_package (PCL REQUIRED)
find_package (pcl_conversions REQUIRED)
find_package (OpenMP)


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})


# Concatenate data
ament_auto_add_library(
  concatenate_data SHARED
  src/concatenate_data/concatenate_data_component.cpp
  )

target_link_libraries(concatenate_data
                      ${Boost_LIBRARIES})
#add_dependencies(concatenate_data)


rclcpp_components_register_node(concatenate_data
  PLUGIN "autoware::pointcloud_preprocessor::ConcatenateData"
  EXECUTABLE concatenate_data_exe)

#############
## Install ##
#############

# Node

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
)
