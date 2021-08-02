cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

set(velocity_controller_SRC
  src/velocity_controller_core.cpp
  src/pid.cpp
  src/velocity_controller_mathutils.cpp
)

ament_auto_add_library(velocity_controller_core SHARED
  ${velocity_controller_HEADER}
  ${velocity_controller_SRC}
)

rclcpp_components_register_node(velocity_controller_core
  PLUGIN "VelocityController"
  EXECUTABLE velocity_controller
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
