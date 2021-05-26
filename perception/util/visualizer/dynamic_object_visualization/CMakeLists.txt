cmake_minimum_required(VERSION 3.5)
project(dynamic_object_visualization)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(dynamic_object_visualizer SHARED
  src/dynamic_object_visualizer.cpp
)

rclcpp_components_register_node(dynamic_object_visualizer
  PLUGIN "DynamicObjectVisualizer"
  EXECUTABLE dynamic_object_visualizer_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
