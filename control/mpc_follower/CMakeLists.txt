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
  # TODO add -Werror after all else builds
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO dynamic_reconfigure not part of dependencies in XML
# generate_dynamic_reconfigure_options(cfg/MPCFollower.cfg)

# TODO These flags shouldn't be set here but in the cmake release type. And they definitely shouldn't be set without checking the compiler; i.e., that would just fail on windows
# SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

set(MPC_FOLLOWER_SRC
  src/mpc_utils.cpp
  src/interpolate.cpp
  src/mpc_trajectory.cpp
  src/lowpass_filter.cpp
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
  src/qp_solver/qp_solver_osqp.cpp
)

# TODO remove when OSQP in CI and osqp_interface merged
# ----
find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)
include_directories(
  ${osqp_INCLUDE_DIR}
  )
# ---

ament_auto_add_executable(mpc_follower src/mpc_follower_node.cpp src/mpc_follower_core.cpp ${MPC_FOLLOWER_SRC})

# TODO do we need to do anything manually? check install/
# catkin_install_python(
#   PROGRAMS src/steer_simulator.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
