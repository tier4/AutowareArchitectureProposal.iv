cmake_minimum_required(VERSION 3.5)
project(autoware_iv_external_api_adaptor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/door.cpp
  src/emergency.cpp
  src/engage.cpp
  src/initial_pose.cpp
  src/map.cpp
  src/operator.cpp
  src/route.cpp
  src/service.cpp
  src/velocity.cpp
  src/version.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Diagnostics")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Door")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Emergency")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Engage")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::InitialPose")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Map")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Operator")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Route")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Service")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Velocity")
rclcpp_components_register_nodes(${PROJECT_NAME} "external_api::Version")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
