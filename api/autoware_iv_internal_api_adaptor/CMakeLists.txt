cmake_minimum_required(VERSION 3.5)
project(autoware_iv_internal_api_adaptor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose.cpp
  src/iv_msgs.cpp
  src/operator.cpp
  src/route.cpp
  src/velocity.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME} "internal_api::InitialPose")
rclcpp_components_register_nodes(${PROJECT_NAME} "internal_api::IVMsgs")
rclcpp_components_register_nodes(${PROJECT_NAME} "internal_api::Operator")
rclcpp_components_register_nodes(${PROJECT_NAME} "internal_api::Route")
rclcpp_components_register_nodes(${PROJECT_NAME} "internal_api::Velocity")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
