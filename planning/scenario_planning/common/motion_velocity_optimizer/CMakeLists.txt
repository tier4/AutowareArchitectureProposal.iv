# colcon build --packages-select motion_velocity_optimizer

cmake_minimum_required(VERSION 3.5)
project(motion_velocity_optimizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(MOTION_VELOCITY_OPTIMIZER_SRC
  src/motion_velocity_optimizer.cpp
  src/motion_velocity_optimizer_utils.cpp
  src/interpolate.cpp
  src/optimizer/pseudo_jerk_l2_optimizer.cpp
  src/optimizer/pseudo_jerk_linf_optimizer.cpp
)
set(MOTION_VELOCITY_OPTIMIZER_HDR
  include/motion_velocity_optimizer/motion_velocity_optimizer.hpp
  include/motion_velocity_optimizer/motion_velocity_optimizer_utils.hpp
  include/motion_velocity_optimizer/interpolate.hpp
  include/motion_velocity_optimizer/optimizer/l2_pseudo_jerk_optimizer.hpp
  include/motion_velocity_optimizer/optimizer/linf_pseudo_jerk_optimizer.hpp
  include/motion_velocity_optimizer/optimizer/optimizer_base.hpp
)

ament_auto_add_library(motion_velocity_optimizer_node SHARED
  ${MOTION_VELOCITY_OPTIMIZER_SRC}
  ${MOTION_VELOCITY_OPTIMIZER_HDR}
)

rclcpp_components_register_node(motion_velocity_optimizer_node
  PLUGIN "MotionVelocityOptimizer"
  EXECUTABLE motion_velocity_optimizer
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
install(PROGRAMS
  scripts/trajectory_visualizer.py
  scripts/closest_velocity_checker.py
  DESTINATION lib/${PROJECT_NAME}
)
