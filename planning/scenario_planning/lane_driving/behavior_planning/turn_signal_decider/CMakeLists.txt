cmake_minimum_required(VERSION 3.5)
project(turn_signal_decider)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(turn_signal_decider_core SHARED
  src/turn_signal_decider_core.cpp
  src/data_manager.cpp
  src/frenet_coordinate.cpp
)

rclcpp_components_register_node(turn_signal_decider_core
  PLUGIN "turn_signal_decider::TurnSignalDecider"
  EXECUTABLE turn_signal_decider
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
