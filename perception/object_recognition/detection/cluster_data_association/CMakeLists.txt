cmake_minimum_required(VERSION 3.5)
project(cluster_data_association)

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

include_directories(
  include
)

ament_auto_add_executable(cluster_data_association_node
  src/utils/utils.cpp
  src/data_association/data_association.cpp
  src/data_association/successive_shortest_path/successive_shortest_path.cpp
  src/node.cpp
  src/main.cpp
)
target_compile_definitions(cluster_data_association_node PRIVATE ${PCL_DEFINITIONS})
target_include_directories(cluster_data_association_node PRIVATE ${PCL_INCLUDE_DIRS})
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(cluster_data_association_node ${PCL_LIBRARIES})
target_link_directories(cluster_data_association_node PRIVATE ${PCL_LIBRARY_DIRS})

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
