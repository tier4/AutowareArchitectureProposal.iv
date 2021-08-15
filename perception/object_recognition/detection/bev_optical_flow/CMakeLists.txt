cmake_minimum_required(VERSION 3.5)
project(bev_optical_flow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# Add path of include dir
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Generate exe file
ament_auto_add_library(optical_flow SHARED
  src/node.cpp
  src/lidar_to_image.cpp
  src/flow_calculator.cpp
  src/utils.cpp
  src/debugger.cpp
)

target_link_libraries(optical_flow
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(optical_flow
  PLUGIN "bev_optical_flow::OpticalFlowNode"
  EXECUTABLE optical_flow_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
