cmake_minimum_required(VERSION 3.5)
project(object_flow_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
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
ament_auto_add_executable(object_flow_fusion_node
src/main.cpp
src/node.cpp
src/object_flow_fusion.cpp
src/utils.cpp
)

target_link_libraries(object_flow_fusion_node
  ${OpenCV_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
