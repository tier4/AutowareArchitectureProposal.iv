cmake_minimum_required(VERSION 3.5)
project(map_tf_generator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Add path of include dir
include_directories(include)

# Generate exe file
add_executable(map_tf_generator nodes/map_tf_generator.cpp)

# Add dependencies. use target_link_libaries() before Crystal
set(MAP_TF_GENERATOR_DEPENDENCIES
  rclcpp
  std_msgs
  tf2
  tf2_ros
  PCL
  pcl_conversions
)
ament_target_dependencies(map_tf_generator ${MAP_TF_GENERATOR_DEPENDENCIES})

## Install executables and/or libraries
install(TARGETS map_tf_generator
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install project namespaced headers
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# set at the end of cmakelists
ament_package()
