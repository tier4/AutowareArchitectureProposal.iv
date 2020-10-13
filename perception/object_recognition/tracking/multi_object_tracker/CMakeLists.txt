cmake_minimum_required(VERSION 3.5)
project(multi_object_tracker)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(unique_identifier REQUIRED)
# find_package(unique_identifier_msgs REQUIRED)

## Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(
  include
  # ${EIGEN3_INCLUDE_DIR}
)

# Generate exe file
set(MULTI_OBJECT_TRACKER_SRC
  src/multi_object_tracker_core.cpp
  src/utils/utils.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/vehicle_tracker.cpp
  src/tracker/model/bicycle_tracker.cpp
  src/tracker/model/pedestrian_tracker.cpp
  src/data_association/data_association.cpp
  src/data_association/successive_shortest_path/successive_shortest_path.cpp
)

add_executable(${PROJECT_NAME}_exe
  src/multi_object_tracker_node.cpp
  ${MULTI_OBJECT_TRACKER_SRC}
)

# Add dependencies. use target_link_libaries() before Crystal
set(MULTI_OBJECT_TRACKER_DEPENDENCIES
  rclcpp
  autoware_perception_msgs
  tf2
  tf2_ros
  # unique_identifier_msgs
  # uuid_msgs
  Eigen3
)

ament_target_dependencies(${PROJECT_NAME}_exe ${MULTI_OBJECT_TRACKER_DEPENDENCIES})

## Install executables and/or libraries
install(TARGETS ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# set at the end of cmakelists
ament_package()
