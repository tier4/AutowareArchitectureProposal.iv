cmake_minimum_required(VERSION 3.5)
project(mpc_follower)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add_compile_options(-std=c++14 -Wall -Werror)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPC_FOLLOWER_SRC
  src/mpc_utils.cpp
  src/interpolate.cpp
  src/mpc_follower_core.cpp
  src/mpc_follower_node.cpp
  src/mpc_trajectory.cpp
  src/lowpass_filter.cpp
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
  src/qp_solver/qp_solver_osqp.cpp
)
set(MPC_FOLLOWER_HDR
  include/mpc_follower/mpc_utils.hpp
  include/mpc_follower/interpolate.hpp
  include/mpc_follower/mpc_follower_core.hpp
  include/mpc_follower/mpc_trajectory.hpp
  include/mpc_follower/lowpass_filter.hpp
  include/mpc_follower/vehicle_model/vehicle_model_interface.hpp
  include/mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp
  include/mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp
  include/mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp
  include/mpc_follower/qp_solver/qp_solver_unconstr_fast.hpp
  include/mpc_follower/qp_solver/qp_solver_osqp.hpp
)

ament_auto_add_executable(mpc_follower ${MPC_FOLLOWER_SRC} ${MPC_FOLLOWER_HDR})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
